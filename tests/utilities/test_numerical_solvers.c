#include <assert.h>
#include <math.h>
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

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

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

static double spring_total_energy(const spring_pendulum_state_t* state,
                                  const spring_pendulum_params_t* params) {
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

    return kinetic + spring_pe + gravity_pe;
}

static void test_euler_constant_acceleration(void) {
    point_mass_state_t state = {.position = 0.0, .velocity = 0.0};
    point_mass_state_t deriv = {.position = 0.0, .velocity = 0.0};
    const double accel = 2.0;
    const double mass = 1.0;
    const double force = mass * accel;
    const double dt = 0.01;
    const int steps = 100;

    for (int i = 0; i < steps; ++i) {
        dm_integrate_euler(&state, &accel, dt, sizeof(state),
                           point_mass_derivative, NULL, &deriv);
    }

    const double expected_velocity = accel * dt * steps;
    const double expected_position = accel * dt * dt * steps * (steps - 1) / 2.0;

    printf("[Euler] a=%.2f m/s^2, dt=%.3f s, steps=%d → pos=%.6f m (expected %.6f), "
           "vel=%.6f m/s (expected %.6f)\n",
           accel, dt, steps, state.position, expected_position,
           state.velocity, expected_velocity);

    const double kinetic = 0.5 * mass * state.velocity * state.velocity;
    const double work = force * state.position;
    const double energy_error = fabs(kinetic - work);
    printf("        energy check: KE=%.6f J, Work=%.6f J, error=%.6f J\n",
           kinetic, work, energy_error);

    assert(nearly_equal(state.velocity, expected_velocity, 1e-12));
    assert(nearly_equal(state.position, expected_position, 1e-12));
    assert(energy_error <= 5e-2);
}

static void test_rk4_constant_acceleration(void) {
    point_mass_state_t state = {.position = 0.0, .velocity = 0.0};
    point_mass_state_t k1, k2, k3, k4, temp;
    const double accel = 2.0;
    const double mass = 1.0;
    const double force = mass * accel;
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

    printf("[RK4] a=%.2f m/s^2, dt=%.3f s, total_time=%.2f s → pos=%.9f m "
           "(expected %.9f), vel=%.9f m/s (expected %.9f)\n",
           accel, dt, total_time, state.position, expected_position,
           state.velocity, expected_velocity);

    const double kinetic = 0.5 * mass * state.velocity * state.velocity;
    const double work = force * state.position;
    const double energy_error = fabs(kinetic - work);
    printf("       energy check: KE=%.9f J, Work=%.9f J, error=%.9f J\n",
           kinetic, work, energy_error);

    assert(nearly_equal(state.velocity, expected_velocity, 1e-12));
    assert(nearly_equal(state.position, expected_position, 1e-9));
    assert(energy_error <= 1e-9);
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

    printf("[RK4] missing scratch buffers → state unchanged (pos=%.6f m, vel=%.6f m/s)\n",
           state.position, state.velocity);

    assert(nearly_equal(state.position, initial.position, 1e-12));
    assert(nearly_equal(state.velocity, initial.velocity, 1e-12));
}

static void test_spring_pendulum_energy_decay(void) {
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

    double energies[steps + 1];
    energies[0] = spring_total_energy(&state, &params);
    double max_increase = 0.0;

    for (int i = 0; i < steps; ++i) {
        dm_integrate_rk4(&state, &params, dt, sizeof(state),
                         spring_pendulum_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
        energies[i + 1] = spring_total_energy(&state, &params);
        const double delta = energies[i + 1] - energies[i];
        if (delta > max_increase) {
            max_increase = delta;
        }
    }

    const double initial_energy = energies[0];
    const double final_energy = energies[steps];
    printf("[Spring Pendulum] energy decay: initial=%.6f J, final=%.6f J, "
           "max_step_increase=%.6e J\n",
           initial_energy, final_energy, max_increase);

    assert(final_energy < initial_energy);
    assert(max_increase <= 1e-6);
}

int main(void) {
    test_euler_constant_acceleration();
    test_rk4_constant_acceleration();
    test_rk4_missing_scratch_is_noop();
    test_spring_pendulum_energy_decay();
    return 0;
}
