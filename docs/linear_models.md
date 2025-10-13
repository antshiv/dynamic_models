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

Future sections will capture the spring pendulum, inverted pendulum, and
quadcopter hover linearizations once those derivations are complete.
