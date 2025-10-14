# Numerical Integration Methods

This primer expands on the summary table in the main README. Each section
describes the governing update rule, stability traits, and when to use the
method inside the dynamic models library.

## Forward Euler (Explicit Euler)
- **Update rule:** `x_{k+1} = x_k + dt * f(t_k, x_k, u_k)`
- **Local order / global order:** 1 / 1
- **Characteristics:** Fast to evaluate, but accuracy decays quickly unless the
  timestep is very small. Conditionally stable: overstated `dt` causes solutions
  to diverge for stiff systems.
- **Use it for:** Prototyping, back-of-the-envelope verification, or when
  implementing analytic tests. In this repository it is available through
  `dm_integrate_euler`.
- **Caveats:** For oscillatory plants (pendula, rotorcraft attitude) the phase
  error grows dramatically. Always compare against a higher-order benchmark
  before trusting results.

## Classical Runge–Kutta (RK4)
- **Update rule:** Four intermediate slopes combined as  
  `x_{k+1} = x_k + dt/6 * (k1 + 2k2 + 2k3 + k4)`  
  with `k1 = f(t_k, x_k)`, `k2 = f(t_k + dt/2, x_k + dt/2 * k1)`, etc.
- **Local order / global order:** 5 / 4
- **Characteristics:** Workhorse integrator for smooth dynamics. Good accuracy
  with fixed step size; moderate cost (four derivative evaluations per step).
- **Use it for:** Main plant propagation in the absence of severe stiffness. All
  current examples (`point_mass`, `spring_pendulum`) call
  `dm_integrate_rk4`.
- **Caveats:** Still explicit, so stability is limited for very stiff or
  discontinuous dynamics. Does not adapt the step size on its own.

## Adaptive Runge–Kutta (Dormand–Prince 5(4))
- **Update rule:** Embedded RK pair that produces both 5th- and 4th-order
  estimates to infer local truncation error. Algorithm adjusts `dt` dynamically
  to hit a user-specified tolerance.
- **Local order / global order:** 6 / 5 (with error estimator)
- **Characteristics:** Excellent balance of accuracy and performance on
  non-stiff problems spanning multiple timescales.
- **Use it for:** Planned upgrade once the utilities layer gains an error
  control scaffold. Ideal for aerodynamics models where speed, blade angle, and
  motor currents introduce disparate bandwidths.
- **Caveats:** Needs a robust step-rejection strategy and guardrails on minimum
  step size to avoid infinite refinement when the derivative is discontinuous.

## Implicit Euler and Trapezoidal (Crank–Nicolson)
- **Implicit Euler update:** `x_{k+1} = x_k + dt * f(t_{k+1}, x_{k+1}, u_{k+1})`
- **Trapezoidal update:**  
  `x_{k+1} = x_k + dt/2 * [f(t_k, x_k, u_k) + f(t_{k+1}, x_{k+1}, u_{k+1})]`
- **Local order / global order:** 2 / 1 for implicit Euler, 3 / 2 for trapezoidal.
- **Characteristics:** A-stable (implicit Euler) or L-stable variants exist, so
  stiff systems remain bounded even with large `dt`. Requires solving a set of
  nonlinear equations each step, typically via Newton–Raphson.
- **Use it for:** Contact dynamics, highly damped structural modes, inverted
  pendulum balancing when the controller bandwidth is high. Will eventually
  serve as the backbone for rigid-body attitude integration coupled with fast
  motor models.
- **Caveats:** Requires Jacobians or automatic differentiation to converge the
  implicit solve. Implementation is planned once a lightweight linear algebra
  backend is in place.

## Model Predictive / Shooting Integrators
- **Concept:** Rather than a single-step update, these methods integrate across
  a horizon (using RK or implicit techniques) and feed the result into an
  optimizer.
- **Use it for:** Planned under the “inverted pendulum” and future multirotor
  MPC demos. Not yet implemented, but the documentation space is reserved for
  the upcoming controller-in-the-loop experiments.

---

**Next steps for the codebase**
1. Add error-controlled RK (Dormand–Prince) to `src/utilities/numerical_solver.c`
   along with tests that shrink the step in high-acceleration phases.
2. Prototype an implicit Euler solver using finite-difference Jacobians so that
   stiff battery or structural models can run without sub-millisecond steps.
3. Expose method selection through the example harnesses, allowing quick
   benchmarking between explicit and implicit approaches.

_Planned additions:_ `dm_integrate_rk45_adaptive` (Dormand–Prince) with
automatic step-size control and an implicit-solver scaffold (backward Euler /
trapezoidal) once the linear algebra pieces are ready.
