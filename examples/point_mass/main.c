#include <stdio.h>
#include "utilities/numerical_solvers.h"

typedef struct {
    double position;
    double velocity;
} point_mass_state_t;

static void point_mass_derivative(const void* state,
                                  const void* input,
                                  void* state_dot,
                                  void* user_ctx) {
    (void)user_ctx;
    const point_mass_state_t* s = (const point_mass_state_t*)state;
    const double* accel = (const double*)input;
    point_mass_state_t* out = (point_mass_state_t*)state_dot;

    out->position = s->velocity;
    out->velocity = *accel;
}

int main(int argc, char** argv) {
    point_mass_state_t state = {.position = 0.0, .velocity = 0.0};
    const double accel = 2.0;  // m/s^2
    const double mass = 1.0;   // kg
    const double dt = 0.1;     // seconds
    const int steps = 20;

    point_mass_state_t k1, k2, k3, k4, temp;

    const double force = mass * accel;

    printf("Point mass under constant acceleration (a = %.2f m/s^2, m = %.2f kg)\n",
           accel, mass);
    printf("%6s %12s %12s %16s %16s\n",
           "Time", "Position (m)", "Velocity (m/s)",
           "Kinetic E (J)", "Work Done (J)");

    FILE* csv = NULL;
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (!csv) {
            perror("fopen");
            return 1;
        }
        fprintf(csv, "time_s,position_m,velocity_m_per_s,kinetic_j,work_j\n");
    }

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &accel, dt, sizeof(state),
                         point_mass_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
        double t = (i + 1) * dt;
        const double kinetic = 0.5 * mass * state.velocity * state.velocity;
        const double work = force * state.position;
        printf("%6.2f %12.6f %12.6f %16.6f %16.6f\n",
               t, state.position, state.velocity, kinetic, work);
        if (csv) {
            fprintf(csv, "%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    t, state.position, state.velocity, kinetic, work);
        }
    }

    if (csv) {
        fclose(csv);
    }

    return 0;
}
