# State-Space Modeling Primer

This guide explains the state-space form used when we linearize the nonlinear
plants in `dynamic_models`. Keep it nearby while we add the point-mass,
pendulum, and cart-pole linear models.

## 1. From Dynamics to State Variables

Most mechanical systems are governed by differential equations involving
positions, velocities, and sometimes higher derivatives. For example:

- Point mass: `m·ẍ = F`.
- Spring pendulum: `m·r̈ = -k(r - L₀)·r̂ - c·ṙ - m·g·ĵ`.
- Inverted pendulum: coupled equations relating cart position, pole angle, and
  their derivatives.

Instead of carrying high-order derivatives explicitly, we collect the minimum
set of variables required to specify the system’s instantaneous condition:

```
x_state = [position, velocity]^T           (point mass)
x_state = [x, y, vx, vy]^T                 (spring pendulum)
x_state = [cart position, pole angle,
           cart velocity, pole rate]^T     (inverted pendulum)
```

This is the **state vector** `x`. Its time derivative `ẋ` contains first-order
equations that reference the original physics.

## 2. Continuous-Time State-Space Form

A linear time-invariant (LTI) system is written as:

```
ẋ = A x + B u
y = C x + D u
```

Where:

- `x` is the state vector.
- `u` is the input vector (forces, torques, control commands).
- `y` is an output vector (measurements or quantities we care about).
- `A` describes how the current state drives the derivative (natural dynamics).
- `B` maps control inputs into state derivatives.
- `C` picks out which parts of the state we observe.
- `D` allows direct feedthrough from inputs to outputs (often zero in mechanics).

### Why first-order form?

Any higher-order differential equation can be rewritten as a first-order system
by introducing derivative states. For example, the point mass:

```
ẋ₁ = x₂          (define x₁ = position, x₂ = velocity)
ẋ₂ = (1/m)·F
```

Written in matrix form around an equilibrium `x*`, `u*`:

```
[ẋ₁]   [0  1][x₁] + [0]F
[ẋ₂] = [0  0][x₂] + [1/m]
```

Here `A = [[0, 1],[0, 0]]` and `B = [[0],[1/m]]`.

## 3. Linearization: From Nonlinear to Linear

Most of our systems are nonlinear (sine/cosine terms, products of state and
input, etc.). To apply LTI tools, we linearize about an operating point:

1. Choose equilibrium `(x*, u*)` where `ẋ = 0`.
2. Compute Jacobians:
   - `A = ∂f/∂x |_(x*,u*)`
   - `B = ∂f/∂u |_(x*,u*)`
   where `f(x,u)` is the nonlinear state derivative function.
3. Define perturbation variables `δx = x - x*`, `δu = u - u*`.
4. The linearized dynamics become `δẋ = A·δx + B·δu`.

Outputs are linearized similarly to obtain `C` and `D`.

### Intuition

- Think of `A` as telling you how the system would evolve if you do nothing
  (`δu = 0`). Its eigenvalues describe natural modes and stability.
- `B` explains how inputs push on those modes.
- `C` and `D` describe what you can measure/control.

## 4. Relationship to Derivatives (speed, acceleration, …)

Speed is the first derivative of position, acceleration is the second
derivative. In state-space, we **explicitly store** those derivatives as new
states so that `ẋ` depends only on current states and inputs:

```
position state → derivative is velocity state
velocity state → derivative is acceleration (function of forces)
```

This makes the system first-order, which is crucial for using linear algebra
tools (matrix exponentials, controllability/observability tests, LQR, etc.).

## 5. Why State-Space Matters

- **Analysis:** Eigenvalues of `A` reveal stability and natural frequencies.
- **Control design:** Tools like LQR, pole placement, and MPC require `A`, `B`.
- **Observers:** Kalman filters and Luenberger observers use `(A, C)` pairs.
- **Simulation:** The form feeds directly into numerical integrators (`ẋ = f(x,u)`).

In our repository:
- `src/*` provide the nonlinear dynamics used by examples and tests.
- Upcoming steps will derive `(A, B, C, D)` for the point mass, pendulum, and
  inverted pendulum so the CLI can auto-generate Laplace models and controller
  benchmarks.

## 6. What’s Next

1. Compute equilibria for each example (hover, spring rest, upright cart-pole).
2. Symbolically or numerically evaluate Jacobians to obtain `A`, `B`, `C`, `D`.
3. Validate linear models against nonlinear simulations.
4. Use the state-space form to derive transfer functions and controller demos.

The rest of the documentation will reference this primer whenever we publish
new linear models or Laplace transforms. Feel free to extend it with additional
intuition as more systems are added.
