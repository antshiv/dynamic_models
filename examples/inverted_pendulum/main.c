#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "utilities/numerical_solvers.h"

typedef struct {
    double x;        /* cart position */
    double theta;    /* pendulum angle (0 = upright) */
    double x_dot;    /* cart velocity */
    double theta_dot;/* angular rate */
} inv_pend_state_t;

typedef struct {
    double mass_cart;
    double mass_pole;
    double length;
    double gravity;
    double cart_force; /* constant input for demo */
    double damping_cart;
    double damping_pole;
} inv_pend_params_t;

static void inverted_pendulum_derivative(const void* state,
                                         const void* input,
                                         void* state_dot,
                                         void* user_ctx) {
    (void)user_ctx;
    const inv_pend_state_t* s = (const inv_pend_state_t*)state;
    const inv_pend_params_t* p = (const inv_pend_params_t*)input;
    inv_pend_state_t* out = (inv_pend_state_t*)state_dot;

    const double M = p->mass_cart;
    const double m = p->mass_pole;
    const double l = p->length;
    const double g = p->gravity;
    const double F = p->cart_force;
    const double b_cart = p->damping_cart;
    const double b_pole = p->damping_pole;

    const double theta = s->theta;
    const double theta_dot = s->theta_dot;
    const double x_dot = s->x_dot;

    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    const double total_mass = M + m;
    const double pole_mass_length = m * l;

    const double temp = (F - b_cart * x_dot + pole_mass_length * theta_dot * theta_dot * sin_theta) / total_mass;
    const double denominator = l * (4.0 / 3.0 - (m * cos_theta * cos_theta) / total_mass);

    const double theta_double_dot =
        (g * sin_theta - cos_theta * temp - (b_pole * theta_dot) / pole_mass_length) / denominator;

    const double x_double_dot =
        temp - (pole_mass_length * theta_double_dot * cos_theta) / total_mass;

    out->x = x_dot;
    out->theta = theta_dot;
    out->x_dot = x_double_dot;
    out->theta_dot = theta_double_dot;
}

static void inverted_pendulum_energy(const inv_pend_state_t* state,
                                     const inv_pend_params_t* params,
                                     double* kinetic,
                                     double* potential,
                                     double* total) {
    const double M = params->mass_cart;
    const double m = params->mass_pole;
    const double l = params->length;
    const double g = params->gravity;

    const double x_dot = state->x_dot;
    const double theta = state->theta;
    const double theta_dot = state->theta_dot;

    const double cart_ke = 0.5 * M * x_dot * x_dot;
    const double pendulum_ke =
        0.5 * m * ( (x_dot + l * theta_dot * cos(theta)) * (x_dot + l * theta_dot * cos(theta)) +
                    (l * theta_dot * sin(theta)) * (l * theta_dot * sin(theta)) );

    const double kinetic_energy = cart_ke + pendulum_ke;
    const double potential_energy = m * g * l * (1.0 - cos(theta));

    if (kinetic) {
        *kinetic = kinetic_energy;
    }
    if (potential) {
        *potential = potential_energy;
    }
    if (total) {
        *total = kinetic_energy + potential_energy;
    }
}

int main(int argc, char** argv) {
    inv_pend_state_t state = {
        .x = 0.0,
        .theta = 5.0 * M_PI / 180.0,
        .x_dot = 0.0,
        .theta_dot = 0.0,
    };

    const inv_pend_params_t params = {
        .mass_cart = 1.0,
        .mass_pole = 0.1,
        .length = 0.5,
        .gravity = 9.81,
        .cart_force = 0.0,
        .damping_cart = 0.05,
        .damping_pole = 0.002,
    };

    const double dt = 0.002;
    const int steps = 4000;

    inv_pend_state_t k1, k2, k3, k4, temp;

    printf("Inverted pendulum on a cart (M=%.2f kg, m=%.2f kg, l=%.2f m)\n",
           params.mass_cart, params.mass_pole, params.length);
    printf("%6s %12s %12s %12s %12s %14s %14s %14s\n",
           "Time", "x (m)", "theta (rad)", "x_dot (m/s)", "theta_dot (rad/s)",
           "KE (J)", "PE (J)", "Total E (J)");

    FILE* csv = NULL;
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (!csv) {
            perror("fopen");
            return 1;
        }
        fprintf(csv,
                "time_s,x_m,theta_rad,x_dot_m_per_s,theta_dot_rad_per_s,kinetic_j,potential_j,total_j\n");
    }

    double time = 0.0;
    double kinetic = 0.0;
    double potential = 0.0;
    double total = 0.0;

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &params, dt, sizeof(state),
                         inverted_pendulum_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
        time += dt;

        if ((i + 1) % 100 == 0) {
            inverted_pendulum_energy(&state, &params, &kinetic, &potential, &total);
            printf("%6.3f %12.6f %12.6f %12.6f %12.6f %14.6f %14.6f %14.6f\n",
                   time, state.x, state.theta, state.x_dot, state.theta_dot,
                   kinetic, potential, total);
        }

        if (csv) {
            inverted_pendulum_energy(&state, &params, &kinetic, &potential, &total);
            fprintf(csv,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    time, state.x, state.theta, state.x_dot, state.theta_dot,
                    kinetic, potential, total);
        }
    }

    if (csv) {
        fclose(csv);
    }

    return 0;
}
