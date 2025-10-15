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

## Mass–Spring–Damper (1D)

- **State vector:** `x = [position, velocity]^T`
- **Input:** `u = F` (external force applied to the mass)
- **Outputs:** `y = x`
- **Equilibrium:** zero displacement and velocity with zero input.

Matrices:

```
A = [[0, 1],
     [-k/m, -c/m]]

B = [[0],
     [1/m]]

C = I₂
D = 0
```

Implementation: `dm_mass_spring_linearize()` in
`src/linear/mass_spring_linear.c` validates the parameters (`m > 0`,
`k ≥ 0`, `c ≥ 0`) and returns the equilibrium state in `x_eq`.

Unit coverage: `tests/linear/test_mass_spring_linear.c`.

---

## Simple Pendulum (Downward Equilibrium)

- **State vector:** `x = [θ, ω]^T`
- **Input:** `u = τ` (torque about the pivot)
- **Outputs:** `y = x`
- **Equilibrium:** bob hanging straight down (`θ = 0`, `ω = 0`, `τ = 0`).

Matrices (small-angle approximation):

```
A = [[0, 1],
     [-g/L, -c/(mL²)]]

B = [[0],
     [1/(mL²)]]

C = I₂
D = 0
```

Implementation: `dm_pendulum_linearize()` in
`src/linear/pendulum_linear.c`. The function ensures positive mass/length,
non-negative damping, and returns the equilibrium state via `x_eq`.

Unit coverage: `tests/linear/test_pendulum_linear.c`.

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

## Inverted Pendulum on a Cart (Upright Equilibrium)
- **State vector:** `x = [x, θ, ẋ, θ̇]^T`
- **Input:** `u = F` (cart force)
- **Outputs:** `y = x`
- **Equilibrium:** cart centered, pole upright (`x = 0`, `θ = 0`, `ẋ = θ̇ = 0`,
  `F = 0`).

Let `M_total = M + m` and `den = l (4/3 - m / M_total)`. Define
`coeff = (m·l) / M_total`. The linearized matrices are:

```
A = [[0, 0, 1, 0],
     [0, 0, 0, 1],
     [a_xθ, 0, a_xẋ, a_xθ̇],
     [a_θθ, 0, a_θẋ, a_θθ̇]]

where
  a_θθ   = g / den
  a_θẋ  = b_c / (den · M_total)
  a_θθ̇  = -b_p / (den · m · l)
  b_θ    = -1 / (den · M_total)
  a_xθ   = -coeff · a_θθ
  a_xẋ  = -(b_c / M_total) - coeff · a_θẋ
  a_xθ̇  = -coeff · a_θθ̇
  b_x    = (1 / M_total) - coeff · b_θ

B = [[0],
     [0],
     [b_x],
     [b_θ]]

C = I₄,
D = 0,
u_eq = 0.
```

Implementation: `dm_inverted_pendulum_linearize()` in
`src/linear/inverted_pendulum_linear.c`.

Unit coverage: `tests/linear/test_inverted_pendulum_linear.c`.

---

## Double Pendulum (Downward/Hanging Equilibrium)
- **State vector:** `x = [θ₁, θ₂, ω₁, ω₂]^T`
- **Inputs:** joint torques `u = [τ₁, τ₂]^T`
- **Outputs:** `y = x`
- **Equilibrium:** both links hanging straight down (`θ = 0`, `ω = 0`, `τ = 0`).

Define the mass matrix and its inverse:

```
M = [[(m₁ + m₂) L₁²,  m₂ L₁ L₂],
     [ m₂ L₁ L₂,      m₂ L₂²]]

M⁻¹ = (1/det) [[m₂ L₂²,      -m₂ L₁ L₂],
               [-m₂ L₁ L₂,  (m₁ + m₂) L₁²]]
```

The stiffness vector is `G = diag(((m₁ + m₂) g L₁), (m₂ g L₂))` and damping is
`D = diag(c₁, c₂)`. The state-space matrices are:

```
A = [[0, 0, 1, 0],
     [0, 0, 0, 1],
     [-M⁻¹ G   | -M⁻¹ D]]

B = [[0, 0],
     [0, 0],
     [M⁻¹]]

C = I₄,  D = 0
```

Implementation: `dm_double_pendulum_linearize()` in
`src/linear/double_pendulum_linear.c`.

Unit coverage: `tests/linear/test_double_pendulum_linear.c`.

---

Future sections will capture the quadcopter hover linearizations once those
derivations are complete.
