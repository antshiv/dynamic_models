/* Numerical integration implementations */
#include "utilities/numerical_solvers.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DM_DOUBLE_SIZE sizeof(double)

static int dm_is_valid_buffer(size_t state_bytes) {
    return (state_bytes != 0U) && (state_bytes % DM_DOUBLE_SIZE == 0U);
}

static void dm_state_copy(double* dest, const double* src, size_t count) {
    memcpy(dest, src, count * DM_DOUBLE_SIZE);
}

static void dm_state_axpy(double* dest, const double* src, double alpha, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        dest[i] += alpha * src[i];
    }
}

static void dm_state_combined_update(double* state,
                                     const double* k1,
                                     const double* k2,
                                     const double* k3,
                                     const double* k4,
                                     double dt,
                                     size_t count) {
    const double dt_over_6 = dt / 6.0;
    for (size_t i = 0; i < count; ++i) {
        state[i] += dt_over_6 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

void dm_integrate_euler(void* state,
                        const void* inputs,
                        double dt,
                        size_t state_bytes,
                        dm_derivative_fn derivative,
                        void* user_context,
                        void* scratch) {
    if (!state || !derivative || !scratch || !dm_is_valid_buffer(state_bytes)) {
        return;
    }

    const size_t count = state_bytes / DM_DOUBLE_SIZE;
    double* state_vec = (double*)state;
    double* deriv_vec = (double*)scratch;

    derivative(state, inputs, deriv_vec, user_context);
    dm_state_axpy(state_vec, deriv_vec, dt, count);
}

void dm_integrate_rk4(void* state,
                      const void* inputs,
                      double dt,
                      size_t state_bytes,
                      dm_derivative_fn derivative,
                      void* user_context,
                      void* scratch_k1,
                      void* scratch_k2,
                      void* scratch_k3,
                      void* scratch_k4,
                      void* scratch_temp) {
    if (!state || !derivative || !scratch_k1 || !scratch_k2 ||
        !scratch_k3 || !scratch_k4 || !scratch_temp ||
        !dm_is_valid_buffer(state_bytes)) {
        return;
    }

    const size_t count = state_bytes / DM_DOUBLE_SIZE;

    double* state_vec = (double*)state;
    double* k1 = (double*)scratch_k1;
    double* k2 = (double*)scratch_k2;
    double* k3 = (double*)scratch_k3;
    double* k4 = (double*)scratch_k4;
    double* temp = (double*)scratch_temp;

    derivative(state, inputs, k1, user_context);

    dm_state_copy(temp, state_vec, count);
    dm_state_axpy(temp, k1, 0.5 * dt, count);
    derivative(temp, inputs, k2, user_context);

    dm_state_copy(temp, state_vec, count);
    dm_state_axpy(temp, k2, 0.5 * dt, count);
    derivative(temp, inputs, k3, user_context);

    dm_state_copy(temp, state_vec, count);
    dm_state_axpy(temp, k3, dt, count);
    derivative(temp, inputs, k4, user_context);

    dm_state_combined_update(state_vec, k1, k2, k3, k4, dt, count);
}

/* TODO: add dm_integrate_rk45_adaptive (Dormand–Prince) and implicit
 * integrators once error-control and linear-solver scaffolds are in place. */

static double dm_clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int dm_integrate_rk45_adaptive(void* state,
                               const void* inputs,
                               double dt,
                               size_t state_bytes,
                               dm_derivative_fn derivative,
                               void* user_context,
                               const dm_rk45_params_t* params) {
    if (!state || !derivative || !dm_is_valid_buffer(state_bytes) || dt <= 0.0) {
        return -1;
    }

    dm_rk45_params_t cfg;
    if (params) {
        cfg = *params;
    } else {
        cfg.abs_tol = 1e-6;
        cfg.rel_tol = 1e-6;
        cfg.min_dt = 1e-6;
        cfg.max_dt = dt;
        cfg.safety = 0.9;
        cfg.shrink_limit = 0.2;
        cfg.growth_limit = 5.0;
        cfg.max_attempts = 12;
    }

    if (cfg.abs_tol <= 0.0) {
        cfg.abs_tol = 1e-9;
    }
    if (cfg.rel_tol <= 0.0) {
        cfg.rel_tol = 1e-6;
    }
    if (cfg.min_dt <= 0.0) {
        cfg.min_dt = 1e-9;
    }
    if (cfg.max_dt <= 0.0) {
        cfg.max_dt = dt;
    }
    if (cfg.safety <= 0.0) {
        cfg.safety = 0.9;
    }
    if (cfg.shrink_limit <= 0.0 || cfg.shrink_limit >= 1.0) {
        cfg.shrink_limit = 0.2;
    }
    if (cfg.growth_limit <= 1.0) {
        cfg.growth_limit = 5.0;
    }
    if (cfg.max_attempts == 0) {
        cfg.max_attempts = 12;
    }

    const size_t count = state_bytes / DM_DOUBLE_SIZE;
    double* state_vec = (double*)state;

    double* y0 = (double*)malloc(state_bytes);
    double* y_temp = (double*)malloc(state_bytes);
    double* y5 = (double*)malloc(state_bytes);
    double* y4 = (double*)malloc(state_bytes);
    double* error = (double*)malloc(state_bytes);
    double* k1 = (double*)malloc(state_bytes);
    double* k2 = (double*)malloc(state_bytes);
    double* k3 = (double*)malloc(state_bytes);
    double* k4 = (double*)malloc(state_bytes);
    double* k5 = (double*)malloc(state_bytes);
    double* k6 = (double*)malloc(state_bytes);
    double* k7 = (double*)malloc(state_bytes);

    if (!y0 || !y_temp || !y5 || !y4 || !error ||
        !k1 || !k2 || !k3 || !k4 || !k5 || !k6 || !k7) {
        free(y0);
        free(y_temp);
        free(y5);
        free(y4);
        free(error);
        free(k1);
        free(k2);
        free(k3);
        free(k4);
        free(k5);
        free(k6);
        free(k7);
        return -1;
    }

    double remaining = dt;
    double h = dm_clamp(cfg.max_dt, cfg.min_dt, cfg.max_dt);

    const double power = 0.2; /* 1/(order + 1) with order = 4 */

    int status = 0;

    while (remaining > 0.0) {
        if (h > remaining) {
            h = remaining;
        }
        if (h < cfg.min_dt) {
            h = cfg.min_dt;
        }

        int accepted = 0;
        size_t attempts = 0;
        double error_norm = 0.0;
        double step_taken = h;

        while (!accepted) {
            const double h_current = h;
            if (++attempts > cfg.max_attempts) {
                status = -2;
                goto cleanup;
            }

            dm_state_copy(y0, state_vec, count);

            derivative(y0, inputs, k1, user_context);

            for (size_t i = 0; i < count; ++i) {
                y_temp[i] = y0[i] + h * (0.2 * k1[i]); /* 1/5 */
            }
            derivative(y_temp, inputs, k2, user_context);

            for (size_t i = 0; i < count; ++i) {
                y_temp[i] = y0[i] + h * ((3.0 / 40.0) * k1[i] + (9.0 / 40.0) * k2[i]);
            }
            derivative(y_temp, inputs, k3, user_context);

            for (size_t i = 0; i < count; ++i) {
                y_temp[i] = y0[i] + h * ((44.0 / 45.0) * k1[i]
                                       - (56.0 / 15.0) * k2[i]
                                       + (32.0 / 9.0) * k3[i]);
            }
            derivative(y_temp, inputs, k4, user_context);

            for (size_t i = 0; i < count; ++i) {
                y_temp[i] = y0[i] + h * ((19372.0 / 6561.0) * k1[i]
                                       - (25360.0 / 2187.0) * k2[i]
                                       + (64448.0 / 6561.0) * k3[i]
                                       - (212.0 / 729.0) * k4[i]);
            }
            derivative(y_temp, inputs, k5, user_context);

            for (size_t i = 0; i < count; ++i) {
                y_temp[i] = y0[i] + h * ((9017.0 / 3168.0) * k1[i]
                                       - (355.0 / 33.0) * k2[i]
                                       + (46732.0 / 5247.0) * k3[i]
                                       + (49.0 / 176.0) * k4[i]
                                       - (5103.0 / 18656.0) * k5[i]);
            }
            derivative(y_temp, inputs, k6, user_context);

            for (size_t i = 0; i < count; ++i) {
                y_temp[i] = y0[i] + h * ((35.0 / 384.0) * k1[i]
                                       + (500.0 / 1113.0) * k3[i]
                                       + (125.0 / 192.0) * k4[i]
                                       - (2187.0 / 6784.0) * k5[i]
                                       + (11.0 / 84.0) * k6[i]);
            }
            derivative(y_temp, inputs, k7, user_context);

            error_norm = 0.0;
            for (size_t i = 0; i < count; ++i) {
                y5[i] = y0[i] + h * ((35.0 / 384.0) * k1[i]
                                   + (500.0 / 1113.0) * k3[i]
                                   + (125.0 / 192.0) * k4[i]
                                   - (2187.0 / 6784.0) * k5[i]
                                   + (11.0 / 84.0) * k6[i]);

                y4[i] = y0[i] + h * ((5179.0 / 57600.0) * k1[i]
                                   + (7571.0 / 16695.0) * k3[i]
                                   + (393.0 / 640.0) * k4[i]
                                   - (92097.0 / 339200.0) * k5[i]
                                   + (187.0 / 2100.0) * k6[i]
                                   + (1.0 / 40.0) * k7[i]);

                error[i] = y5[i] - y4[i];

                const double scale = cfg.abs_tol +
                    cfg.rel_tol * fmax(fabs(y0[i]), fabs(y5[i]));
                if (scale > 0.0) {
                    const double ratio = fabs(error[i]) / scale;
                    if (ratio > error_norm) {
                        error_norm = ratio;
                    }
                }
            }

            if (error_norm <= 1.0 || h_current <= cfg.min_dt * 1.01) {
                dm_state_copy(state_vec, y5, count);
                accepted = 1;
                step_taken = h_current;
            }

            const double bounded_error = error_norm > 1e-16 ? error_norm : 1e-16;
            double factor = cfg.safety * pow(bounded_error, -power);
            factor = dm_clamp(factor, cfg.shrink_limit, cfg.growth_limit);

            h = dm_clamp(h_current * factor, cfg.min_dt, cfg.max_dt);
        }

        remaining -= step_taken;
    }

cleanup:
    free(y0);
    free(y_temp);
    free(y5);
    free(y4);
    free(error);
    free(k1);
    free(k2);
    free(k3);
    free(k4);
    free(k5);
    free(k6);
    free(k7);
    return status;
}
