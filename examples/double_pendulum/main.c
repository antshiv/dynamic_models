#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "utilities/numerical_solvers.h"

typedef struct {
    double theta1;
    double theta2;
    double omega1;
    double omega2;
} double_pendulum_state_t;

typedef struct {
    double mass1;
    double mass2;
    double length1;
    double length2;
    double gravity;
} double_pendulum_params_t;

static void double_pendulum_derivative(const void* state,
                                       const void* input,
                                       void* state_dot,
                                       void* user_ctx) {
    (void)user_ctx;
    const double_pendulum_state_t* s = (const double_pendulum_state_t*)state;
    const double_pendulum_params_t* params =
        (const double_pendulum_params_t*)input;
    double_pendulum_state_t* out = (double_pendulum_state_t*)state_dot;

    const double m1 = params->mass1;
    const double m2 = params->mass2;
    const double l1 = params->length1;
    const double l2 = params->length2;
    const double g = params->gravity;

    const double theta1 = s->theta1;
    const double theta2 = s->theta2;
    const double omega1 = s->omega1;
    const double omega2 = s->omega2;

    const double delta = theta1 - theta2;
    const double sin_delta = sin(delta);
    const double cos_delta = cos(delta);

    const double denom = l1 * (2.0 * m1 + m2 - m2 * cos(2.0 * delta));
    const double denom2 = l2 * (2.0 * m1 + m2 - m2 * cos(2.0 * delta));

    const double term1 = -g * (2.0 * m1 + m2) * sin(theta1);
    const double term2 = -m2 * g * sin(theta1 - 2.0 * theta2);
    const double term3 = -2.0 * sin_delta * m2 *
                         (omega2 * omega2 * l2 + omega1 * omega1 * l1 * cos_delta);
    const double alpha1 = (term1 + term2 + term3) / denom;

    const double term4 = 2.0 * sin_delta *
        (omega1 * omega1 * l1 * (m1 + m2) +
         g * (m1 + m2) * cos(theta1) +
         omega2 * omega2 * l2 * m2 * cos_delta);
    const double alpha2 = term4 / denom2;

    out->theta1 = omega1;
    out->theta2 = omega2;
    out->omega1 = alpha1;
    out->omega2 = alpha2;
}

static void double_pendulum_energy(const double_pendulum_state_t* state,
                                   const double_pendulum_params_t* params,
                                   double* kinetic_out,
                                   double* potential_out,
                                   double* total_out) {
    const double m1 = params->mass1;
    const double m2 = params->mass2;
    const double l1 = params->length1;
    const double l2 = params->length2;
    const double g = params->gravity;

    const double theta1 = state->theta1;
    const double theta2 = state->theta2;
    const double omega1 = state->omega1;
    const double omega2 = state->omega2;

    const double cos1 = cos(theta1);
    const double cos2 = cos(theta2);

    const double v1_sq = l1 * l1 * omega1 * omega1;
    const double v2_sq = v1_sq +
        l2 * l2 * omega2 * omega2 +
        2.0 * l1 * l2 * omega1 * omega2 * cos(theta1 - theta2);

    const double kinetic = 0.5 * m1 * v1_sq + 0.5 * m2 * v2_sq;

    const double y1 = -l1 * cos1;
    const double y2 = y1 - l2 * cos2;
    const double potential = m1 * g * y1 + m2 * g * y2;

    if (kinetic_out) {
        *kinetic_out = kinetic;
    }
    if (potential_out) {
        *potential_out = potential;
    }
    if (total_out) {
        *total_out = kinetic + potential;
    }
}

int main(int argc, char** argv) {
    double_pendulum_state_t state = {
        .theta1 = M_PI / 2.0,
        .theta2 = M_PI / 2.0,
        .omega1 = 0.0,
        .omega2 = 0.0,
    };

    const double_pendulum_params_t params = {
        .mass1 = 1.0,
        .mass2 = 1.0,
        .length1 = 1.0,
        .length2 = 1.0,
        .gravity = 9.81,
    };

    const double dt = 0.002;
    const int steps = 5000;

    double_pendulum_state_t k1, k2, k3, k4, temp;

    printf("Planar double pendulum (m1=%.2f kg, m2=%.2f kg, l1=%.2f m, l2=%.2f m)\n",
           params.mass1, params.mass2, params.length1, params.length2);
    printf("%6s %12s %12s %12s %12s %14s %14s %14s\n",
           "Time", "theta1 (rad)", "theta2 (rad)",
           "omega1 (rad/s)", "omega2 (rad/s)",
           "KE (J)", "PE (J)", "Total E (J)");

    FILE* csv = NULL;
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (!csv) {
            perror("fopen");
            return 1;
        }
        fprintf(csv,
                "time_s,theta1_rad,theta2_rad,omega1_rad_per_s,omega2_rad_per_s,kinetic_j,potential_j,total_j\n");
    }

    double time = 0.0;
    double kinetic = 0.0;
    double potential = 0.0;
    double total = 0.0;

    double initial_total = 0.0;
    double_pendulum_energy(&state, &params, &kinetic, &potential, &initial_total);

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &params, dt, sizeof(state),
                         double_pendulum_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
        time += dt;

        if ((i + 1) % 100 == 0) {
            double_pendulum_energy(&state, &params, &kinetic, &potential, &total);
            printf("%6.3f %12.6f %12.6f %12.6f %12.6f %14.6f %14.6f %14.6f\n",
                   time, state.theta1, state.theta2,
                   state.omega1, state.omega2, kinetic, potential, total);
        }

        if (csv) {
            double_pendulum_energy(&state, &params, &kinetic, &potential, &total);
            fprintf(csv,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    time, state.theta1, state.theta2,
                    state.omega1, state.omega2, kinetic, potential, total);
        }
    }

    if (csv) {
        fclose(csv);
    }

    double final_total = 0.0;
    double_pendulum_energy(&state, &params, &kinetic, &potential, &final_total);
    printf("Initial total energy: %.9f J\n", initial_total);
    printf("Final total energy:   %.9f J\n", final_total);

    return 0;
}
