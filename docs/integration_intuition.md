# Integration Intuition: Euler vs. RK4

This note explains how the forward (explicit) Euler method and the classical
Runge–Kutta fourth-order (RK4) method approximate the solution of an ordinary
 differential equation `ẋ = f(x, u, t)`.

## Baseline Concepts
- **Derivative / slope** `f` tells you the instantaneous rate of change.
- **Integration** over a timestep accumulates that rate to update the state:
  `x(t + dt) ≈ x(t) + ∫ f dt`.

## Forward Euler (1st Order)
- Uses the slope at the start of the interval only:
  `x_{k+1} = x_k + dt · f(x_k, u_k, t_k)`.
- Local truncation error `≈ O(dt²)` → global error `O(dt)`.
- Intuition: draw a straight line with slope `f` from `t_k` to `t_k + dt`; the
  area under that line is the update. If the slope changes over the interval,
  the straight-line assumption introduces error.

## Runge–Kutta 4 (4th Order)
- Samples intermediate slopes to better match the curved trajectory:
  ```
  k1 = f(x_k,             u_k, t_k)
  k2 = f(x_k + ½dt·k1,    u_k, t_k + ½dt)
  k3 = f(x_k + ½dt·k2,    u_k, t_k + ½dt)
  k4 = f(x_k + dt·k3,     u_k, t_k + dt)
  x_{k+1} = x_k + dt/6 (k1 + 2k2 + 2k3 + k4)
  ```
- Local truncation error `≈ O(dt⁵)` → global error `O(dt⁴)`.
- Intuition: estimate the area under the curve by sampling the slope at the
  start, two midpoints, and the end, then combine them (1,2,2,1 weights).

### Why “Fourth Order”?
When you expand the true solution into a Taylor series and compare it with the
RK4 update, the coefficients match through the `dt⁴` term, leaving the first
mismatch at `dt⁵`. That is why RK4 has fourth-order accuracy in `dt`, even
though it only ever evaluates the first derivative `f` (no higher derivatives).

### Geometric Picture
- **Euler:** single triangle/rectangle from start point—good if slope is nearly
  constant across the interval.
- **RK4:** break the interval into subsegments, sample slopes in the interior,
  reconstruct a better approximation of the curve, and average the areas.

## When to Use Which
- **Euler:** debugging, simple heuristics, stiff damping if `dt` is tiny.
- **RK4:** default choice for smooth dynamics—balance between accuracy and cost.
- **Adaptive RK (Dormand–Prince etc.):** adjust `dt` to handle varying stiffness.

This repository’s `dm_integrate_euler` and `dm_integrate_rk4` follow the above
forms; upcoming Dormand–Prince support will build on the same intuition with
automatic step-size control.
