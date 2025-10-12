# Numerical Utilities

This directory contains reusable tools that support the dynamic models library:

- `numerical_solvers.h` / `numerical_solver.c` – General-purpose ODE integrators (Forward Euler and classical Runge–Kutta 4).
- `memory_arena.h` – A minimal bump allocator for aligned, single-shot allocations.

## Numerical Solvers Overview

We integrate ordinary differential equations of the form

```
dx/dt = f(x, u, t)
```

where `x` is the state, `u` is the input. The integrators update `x` over a timestep `dt` using only function evaluations of `f`.

### Forward Euler

```
x_{k+1} = x_k + dt * f(x_k, u_k)
```

- First-order accurate (local truncation error O(dt²)).
- Fast but can become unstable for stiff systems or large `dt`.
- Implemented as `dm_integrate_euler`.

### Runge–Kutta 4 (RK4)

Samples four slopes within the interval:

```
k1 = f(x_k, u_k)
k2 = f(x_k + dt/2 * k1, u_k)
k3 = f(x_k + dt/2 * k2, u_k)
k4 = f(x_k + dt   * k3, u_k)
x_{k+1} = x_k + dt/6 * (k1 + 2k2 + 2k3 + k4)
```

- Fourth-order accurate (local error O(dt⁵)).
- Balances accuracy and cost for smooth dynamics.
- Implemented as `dm_integrate_rk4`.

Both solvers take a derivative callback:

```c
void derivative(const void* state,
                const void* inputs,
                void* state_dot,
                void* context);
```

You supply the ODE in this callback; the solver handles the integration.

## Example: Constant Acceleration Car

State: `[position, velocity]`. Input: constant acceleration `a = 2 m/s²`.

Derivative:

```
d/dt position = velocity
d/dt velocity = a
```

In C using the solvers:

```c
typedef struct {
    double position;
    double velocity;
} car_state_t;

static void car_derivative(const void* state,
                           const void* input,
                           void* state_dot,
                           void* ctx) {
    const car_state_t* s = (const car_state_t*)state;
    const double* accel = (const double*)input;
    car_state_t* out = (car_state_t*)state_dot;

    out->position = s->velocity;
    out->velocity = *accel;
}

void simulate_car() {
    car_state_t state = {.position = 0.0, .velocity = 0.0};
    double accel = 2.0;
    double dt = 0.1;
    car_state_t scratch_k1, scratch_k2, scratch_k3, scratch_k4, scratch_temp;

    for (int i = 0; i < 10; ++i) {
        dm_integrate_rk4(&state, &accel, dt, sizeof(state),
                         car_derivative, NULL,
                         &scratch_k1, &scratch_k2, &scratch_k3, &scratch_k4,
                         &scratch_temp);
        printf("t=%.1f s: pos=%.3f m, vel=%.3f m/s\n", (i+1)*dt, state.position, state.velocity);
    }
}
```

The RK4 integrator yields accurate position/velocity without needing analytic formulas.

## Extension to Angular Dynamics

Replace `position/velocity` with `angle/angular-rate`. The derivative structure remains identical:

```
d/dt angle = angular_rate
d/dt angular_rate = torque / inertia
```

Units change, but the solver usage is the same.

## Memory Arena Helper

`memory_arena.h` provides:

- `dm_arena_init(arena, buffer, size)` – Set up the arena with a raw buffer.
- `dm_arena_alloc(arena, bytes, alignment)` – Grab an aligned slice.
- Keeps allocations contiguous and releases everything at once.

This is handy for allocating aligned scratch buffers for the solvers or grouping state structures with minimal overhead.
