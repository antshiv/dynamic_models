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

int main(void) {
    point_mass_state_t state = {.position = 0.0, .velocity = 0.0};
    const double accel = 2.0;  // m/s^2
    const double dt = 0.1;     // seconds
    const int steps = 20;

    point_mass_state_t k1, k2, k3, k4, temp;

    printf("Point mass under constant acceleration (a = %.2f m/s^2)\n", accel);
    printf("%6s %12s %12s\n", "Time", "Position (m)", "Velocity (m/s)");

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &accel, dt, sizeof(state),
                         point_mass_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
        double t = (i + 1) * dt;
        printf("%6.2f %12.6f %12.6f\n", t, state.position, state.velocity);
    }

    return 0;
}
