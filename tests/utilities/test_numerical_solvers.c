#include <assert.h>
#include <math.h>
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

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

static void test_euler_constant_acceleration(void) {
    point_mass_state_t state = {.position = 0.0, .velocity = 0.0};
    point_mass_state_t deriv = {.position = 0.0, .velocity = 0.0};
    const double accel = 2.0;
    const double dt = 0.01;
    const int steps = 100;

    for (int i = 0; i < steps; ++i) {
        dm_integrate_euler(&state, &accel, dt, sizeof(state),
                           point_mass_derivative, NULL, &deriv);
    }

    const double expected_velocity = accel * dt * steps;
    const double expected_position = accel * dt * dt * steps * (steps - 1) / 2.0;

    assert(nearly_equal(state.velocity, expected_velocity, 1e-12));
    assert(nearly_equal(state.position, expected_position, 1e-12));
}

static void test_rk4_constant_acceleration(void) {
    point_mass_state_t state = {.position = 0.0, .velocity = 0.0};
    point_mass_state_t k1, k2, k3, k4, temp;
    const double accel = 2.0;
    const double dt = 0.01;
    const int steps = 100;

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &accel, dt, sizeof(state),
                         point_mass_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
    }

    const double total_time = dt * steps;
    const double expected_velocity = accel * total_time;
    const double expected_position = 0.5 * accel * total_time * total_time;

    assert(nearly_equal(state.velocity, expected_velocity, 1e-12));
    assert(nearly_equal(state.position, expected_position, 1e-9));
}

static void test_rk4_missing_scratch_is_noop(void) {
    const double accel = 2.0;
    const double dt = 0.1;

    const point_mass_state_t initial = {.position = 4.0, .velocity = -1.0};
    point_mass_state_t state = initial;
    point_mass_state_t k2, k3, k4, temp;

    dm_integrate_rk4(&state, &accel, dt, sizeof(state),
                     point_mass_derivative, NULL,
                     NULL, &k2, &k3, &k4, &temp);

    assert(nearly_equal(state.position, initial.position, 1e-12));
    assert(nearly_equal(state.velocity, initial.velocity, 1e-12));
}

int main(void) {
    test_euler_constant_acceleration();
    test_rk4_constant_acceleration();
    test_rk4_missing_scratch_is_noop();
    return 0;
}
