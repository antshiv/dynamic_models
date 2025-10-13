# Linear Model Reference

This document records the state-space models derived from the nonlinear
examples. Each model is expressed as `ẋ = A x + B u`, `y = C x + D u`
following the conventions in `docs/state_space_primer.md`.

## Point Mass (1D)

- **State vector:** `x = [position, velocity]^T`
- **Input:** `u = F` (net thrust along the axis)
- **Outputs:** `y = [position, velocity]^T`
- **Equilibrium:** Any constant thrust `F*` that balances weight. Linear model
  does not depend on the specific `F*` because the dynamics are already linear.

Matrices:

```
A = [[0, 1],
     [0, 0]]

B = [[0],
     [1/m]]

C = I₂
D = 0
```

Implementation: `dm_point_mass_linearize()` in
`src/linear/point_mass_linear.c` populates these matrices given the mass `m`
and returns an error when `m ≤ 0` or the output pointer is null.

Unit coverage: `tests/linear/test_point_mass_linear.c` checks the generated
matrices and validates the error paths.

---

## Spring Pendulum (Hanging Equilibrium)
- **State vector:** `x = [x, y, v_x, v_y]^T`
- **Input:** `u = [F_x, F_y]^T` (small external Cartesian forces)
- **Outputs:** `y = x`
- **Equilibrium:** bob located at `x = 0`, `y = -r_eq`, `v = 0` where
  `r_eq = L₀ + (m g)/k`.

Matrices:

```
A = [[0, 0, 1, 0],
     [0, 0, 0, 1],
     [a_x, 0, -c/m, 0],
     [0, a_y, 0, -c/m]]

where  a_x = -(k/m) * (1 - L₀ / r_eq)
       a_y = -(k/m)

B = [[0,   0],
     [0,   0],
     [1/m, 0],
     [0, 1/m]]

C = I₄
D = 0
```

Implementation: `dm_spring_pendulum_linearize()` in
`src/linear/spring_pendulum_linear.c`. Validates input parameters and reports
the equilibrium state in `x_eq`.

Unit coverage: `tests/linear/test_spring_pendulum_linear.c`.

---

Future sections will capture the spring pendulum, inverted pendulum, and
quadcopter hover linearizations once those derivations are complete.
