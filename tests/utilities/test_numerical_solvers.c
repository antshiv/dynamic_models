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

static double double_pendulum_total_energy(const double_pendulum_state_t* state,
                                           const double_pendulum_params_t* params) {
    const double m1 = params->mass1;
    const double m2 = params->mass2;
    const double l1 = params->length1;
    const double l2 = params->length2;
    const double g = params->gravity;

    const double theta1 = state->theta1;
    const double theta2 = state->theta2;
    const double omega1 = state->omega1;
    const double omega2 = state->omega2;

    const double v1_sq = l1 * l1 * omega1 * omega1;
    const double v2_sq = v1_sq +
        l2 * l2 * omega2 * omega2 +
        2.0 * l1 * l2 * omega1 * omega2 * cos(theta1 - theta2);

    const double kinetic = 0.5 * m1 * v1_sq + 0.5 * m2 * v2_sq;

    const double y1 = -l1 * cos(theta1);
    const double y2 = y1 - l2 * cos(theta2);
    const double potential = m1 * g * y1 + m2 * g * y2;

    return kinetic + potential;
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

static void test_double_pendulum_adaptive_energy(void) {
    const double_pendulum_params_t params = {
        .mass1 = 1.0,
        .mass2 = 1.0,
        .length1 = 1.0,
        .length2 = 1.0,
        .gravity = 9.81,
    };

    double_pendulum_state_t fixed_state = {
        .theta1 = M_PI / 2.0,
        .theta2 = M_PI / 2.0,
        .omega1 = 0.0,
        .omega2 = 0.0,
    };

    double_pendulum_state_t adaptive_state = fixed_state;

    const double total_time = 5.0;
    const double step_dt = 0.05;
    const size_t steps = (size_t)(total_time / step_dt);

    double_pendulum_state_t k1, k2, k3, k4, temp;

    const double initial_energy = double_pendulum_total_energy(&fixed_state, &params);

    for (size_t i = 0; i < steps; ++i) {
        dm_integrate_rk4(&fixed_state, &params, step_dt, sizeof(fixed_state),
                         double_pendulum_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);

        const int status = dm_integrate_rk45_adaptive(&adaptive_state, &params,
                                                      step_dt, sizeof(adaptive_state),
                                                      double_pendulum_derivative, NULL,
                                                      NULL);
        assert(status == 0);
    }

    const double fixed_energy = double_pendulum_total_energy(&fixed_state, &params);
    const double adaptive_energy = double_pendulum_total_energy(&adaptive_state, &params);

    const double fixed_error = fabs(fixed_energy - initial_energy);
    const double adaptive_error = fabs(adaptive_energy - initial_energy);

    printf("[Double Pendulum] fixed-step energy error=%.6f J, adaptive error=%.6f J\n",
           fixed_error, adaptive_error);

    assert(adaptive_error <= fixed_error);
    assert(adaptive_error <= 1e-2);
}

typedef struct {
    double x;
    double theta;
    double x_dot;
    double theta_dot;
} inverted_pendulum_state_t;

typedef struct {
    double mass_cart;
    double mass_pole;
    double length;
    double gravity;
    double cart_force;
    double damping_cart;
    double damping_pole;
} inverted_pendulum_params_t;

static void inverted_pendulum_derivative(const void* state,
                                         const void* input,
                                         void* state_dot,
                                         void* user_ctx) {
    (void)user_ctx;
    const inverted_pendulum_state_t* s = (const inverted_pendulum_state_t*)state;
    const inverted_pendulum_params_t* p =
        (const inverted_pendulum_params_t*)input;
    inverted_pendulum_state_t* out = (inverted_pendulum_state_t*)state_dot;

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

    const double temp =
        (F - b_cart * x_dot + pole_mass_length * theta_dot * theta_dot * sin_theta) / total_mass;
    const double denominator =
        l * (4.0 / 3.0 - (m * cos_theta * cos_theta) / total_mass);

    const double theta_double_dot =
        (g * sin_theta - cos_theta * temp - (b_pole * theta_dot) / pole_mass_length) / denominator;

    const double x_double_dot =
        temp - (pole_mass_length * theta_double_dot * cos_theta) / total_mass;

    out->x = x_dot;
    out->theta = theta_dot;
    out->x_dot = x_double_dot;
    out->theta_dot = theta_double_dot;
}

static dm_rk45_params_t default_rk45_params(void) {
    dm_rk45_params_t params = {
        .abs_tol = 1e-6,
        .rel_tol = 1e-6,
        .min_dt = 1e-6,
        .max_dt = 0.05,
        .safety = 0.9,
        .shrink_limit = 0.2,
        .growth_limit = 5.0,
        .max_attempts = 12,
    };
    return params;
}

static void integrate_rk4_inv_pend(inverted_pendulum_state_t* state,
                                   const inverted_pendulum_params_t* params,
                                   double dt,
                                   size_t steps) {
    inverted_pendulum_state_t k1, k2, k3, k4, temp;
    for (size_t i = 0; i < steps; ++i) {
        dm_integrate_rk4(state, params, dt, sizeof(*state),
                         inverted_pendulum_derivative, NULL,
                         &k1, &k2, &k3, &k4, &temp);
    }
}

static void test_inverted_pendulum_adaptive(void) {
    const inverted_pendulum_params_t params = {
        .mass_cart = 1.0,
        .mass_pole = 0.1,
        .length = 0.5,
        .gravity = 9.81,
        .cart_force = 0.0,
        .damping_cart = 0.05,
        .damping_pole = 0.002,
    };

    const double total_time = 5.0;

    inverted_pendulum_state_t state_ref = {
        .x = 0.0,
        .theta = 5.0 * M_PI / 180.0,
        .x_dot = 0.0,
        .theta_dot = 0.0,
    };

    inverted_pendulum_state_t state_adapt = state_ref;

    const double rk4_dt = 0.001;
    const size_t rk4_steps = (size_t)(total_time / rk4_dt);
    integrate_rk4_inv_pend(&state_ref, &params, rk4_dt, rk4_steps);

    const double adapt_dt = 0.05;
    const size_t adapt_steps = (size_t)(total_time / adapt_dt);
    const dm_rk45_params_t rk_params = default_rk45_params();

    for (size_t i = 0; i < adapt_steps; ++i) {
        const int status = dm_integrate_rk45_adaptive(&state_adapt, &params,
                                                      adapt_dt, sizeof(state_adapt),
                                                      inverted_pendulum_derivative, NULL,
                                                      &rk_params);
        assert(status == 0);
    }

    const double dx = fabs(state_ref.x - state_adapt.x);
    const double dtheta = fabs(state_ref.theta - state_adapt.theta);
    const double dx_dot = fabs(state_ref.x_dot - state_adapt.x_dot);
    const double dtheta_dot = fabs(state_ref.theta_dot - state_adapt.theta_dot);

    printf("[Inverted Pendulum] RK4 vs RK45 differences: |dx|=%.6e, |dθ|=%.6e, |dẋ|=%.6e, |dθ̇|=%.6e\n",
           dx, dtheta, dx_dot, dtheta_dot);

    const double tol_pos = 5e-4;
    const double tol_vel = 5e-4;
    assert(dx <= tol_pos);
    assert(dtheta <= tol_pos);
    assert(dx_dot <= tol_vel);
    assert(dtheta_dot <= tol_vel);
}

int main(void) {
    puts("=== Numerical Solver Regression ===");
    puts("Scenario 1: Point mass under constant thrust (FBD: mass with upward thrust T and weight mg).");
    puts("  Governing equations: m·x_ddot = T - mg, analytic solution x(t) = 0.5·a·t^2, v(t) = a·t.");
    puts("Scenario 2: Damped spring pendulum (FBD: mass on spring with damping, gravity, and elastic force).");
    puts("  Equations: m·r_ddot = -k(r-L0)·r_hat - c·v - m·g·j; total energy should monotonically decrease.");
    puts("Scenario 3: Planar double pendulum (two linked masses, gravity).");
    puts("  Coupled Lagrange equations integrated via RK4 vs. adaptive RK45; energy conservation favors adaptive.");
    puts("Scenario 4: Inverted pendulum on a cart (upright equilibrium).");
    puts("  Nonlinear balance of cart force and gravity; adaptive RK45 maintains bounded energy with damping.");
    puts("==============================================================");
    test_euler_constant_acceleration();
    test_rk4_constant_acceleration();
    test_rk4_missing_scratch_is_noop();
    test_spring_pendulum_energy_decay();
    test_double_pendulum_adaptive_energy();
    test_inverted_pendulum_adaptive();
    return 0;
}
