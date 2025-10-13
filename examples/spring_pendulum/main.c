#include <math.h>
#include <stdio.h>
#include "utilities/numerical_solvers.h"

typedef struct {
    double x;
    double y;
    double vx;
    double vy;
} spring_pendulum_state_t;

typedef struct {
    double mass;
    double stiffness;
    double damping;
    double rest_length;
    double gravity;
} spring_pendulum_params_t;

static void spring_pendulum_derivative(const void* state,
                                       const void* input,
                                       void* state_dot,
                                       void* user_ctx) {
    (void)user_ctx;
    const spring_pendulum_state_t* s = (const spring_pendulum_state_t*)state;
    const spring_pendulum_params_t* params =
        (const spring_pendulum_params_t*)input;
    spring_pendulum_state_t* out = (spring_pendulum_state_t*)state_dot;

    const double mass = params->mass;
    const double stiffness = params->stiffness;
    const double damping = params->damping;
    const double rest_length = params->rest_length;
    const double gravity = params->gravity;

    const double x = s->x;
    const double y = s->y;
    const double vx = s->vx;
    const double vy = s->vy;

    const double r_len = sqrt(x * x + y * y);
    double spring_fx = 0.0;
    double spring_fy = 0.0;

    if (r_len > 1e-9) {
        const double stretch = r_len - rest_length;
        const double scale = -stiffness * stretch / r_len;
        spring_fx = scale * x;
        spring_fy = scale * y;
    }

    const double damping_fx = -damping * vx;
    const double damping_fy = -damping * vy;
    const double gravity_fy = -mass * gravity;

    const double ax = (spring_fx + damping_fx) / mass;
    const double ay = (spring_fy + damping_fy + gravity_fy) / mass;

    out->x = vx;
    out->y = vy;
    out->vx = ax;
    out->vy = ay;
}

static void spring_pendulum_energy(const spring_pendulum_state_t* state,
                                   const spring_pendulum_params_t* params,
                                   double* kinetic_out,
                                   double* spring_potential_out,
                                   double* gravity_potential_out,
                                   double* total_out) {
    const double mass = params->mass;
    const double stiffness = params->stiffness;
    const double rest_length = params->rest_length;
    const double gravity = params->gravity;

    const double vx2 = state->vx * state->vx;
    const double vy2 = state->vy * state->vy;
    const double kinetic = 0.5 * mass * (vx2 + vy2);

    const double r_len = sqrt(state->x * state->x + state->y * state->y);
    const double stretch = r_len - rest_length;
    const double spring_pe = 0.5 * stiffness * stretch * stretch;

    const double gravity_pe = mass * gravity * state->y;
    const double total = kinetic + spring_pe + gravity_pe;

    if (kinetic_out) {
        *kinetic_out = kinetic;
    }
    if (spring_potential_out) {
        *spring_potential_out = spring_pe;
    }
    if (gravity_potential_out) {
        *gravity_potential_out = gravity_pe;
    }
    if (total_out) {
        *total_out = total;
    }
}

int main(int argc, char** argv) {
    spring_pendulum_state_t state = {
        .x = 0.2,
        .y = 0.8,
        .vx = 0.0,
        .vy = 0.0,
    };

    const spring_pendulum_params_t params = {
        .mass = 0.5,
        .stiffness = 15.0,
        .damping = 0.3,
        .rest_length = 0.5,
        .gravity = 9.81,
    };

    const double dt = 0.01;
    const int steps = 1000;

    spring_pendulum_state_t k1, k2, k3, k4, temp;

    printf("Spring pendulum (mass=%.2f kg, k=%.2f N/m, rest=%.2f m)\n",
           params.mass, params.stiffness, params.rest_length);
    printf("%6s %12s %12s %12s %12s %14s %14s %14s %14s\n",
           "Time", "X (m)", "Y (m)", "Vx (m/s)", "Vy (m/s)",
           "KE (J)", "Spring PE (J)", "Grav. PE (J)", "Total E (J)");

    double time = 0.0;
    double kinetic = 0.0;
    double spring_pe = 0.0;
    double gravity_pe = 0.0;
    double total_e = 0.0;

    FILE* csv = NULL;
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (!csv) {
            perror("fopen");
            return 1;
        }
        fprintf(csv,
                "time_s,x_m,y_m,vx_m_per_s,vy_m_per_s,kinetic_j,spring_potential_j,gravity_potential_j,total_energy_j\n");
    }

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &params, dt, sizeof(state),
                         spring_pendulum_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
        time += dt;
        if ((i + 1) % 50 == 0) {
            spring_pendulum_energy(&state, &params,
                                   &kinetic, &spring_pe, &gravity_pe, &total_e);
            printf("%6.2f %12.6f %12.6f %12.6f %12.6f %14.6f %14.6f %14.6f %14.6f\n",
                   time, state.x, state.y, state.vx, state.vy,
                   kinetic, spring_pe, gravity_pe, total_e);
        }
        if (csv) {
            spring_pendulum_energy(&state, &params,
                                   &kinetic, &spring_pe, &gravity_pe, &total_e);
            fprintf(csv,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    time, state.x, state.y, state.vx, state.vy,
                    kinetic, spring_pe, gravity_pe, total_e);
        }
    }

    if (csv) {
        fclose(csv);
    }

    return 0;
}
