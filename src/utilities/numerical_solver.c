/* Numerical integration implementations */
#include "utilities/numerical_solvers.h"
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
