# Dynamics Analysis Cheat Sheet

Use this checklist when turning a mechanical system into equations of motion.
It highlights which physical laws apply, how to account for forces, and how to
spot common patterns (spring, pendulum, cart-pole, etc.).

---

## 1. Identify the System & Frames
1. **Choose coordinates:** Cartesian (x, y, z) or generalized (angles, lengths).
2. **Define frames:** inertial frame (often NED) and body/attached frames.
3. **State variables:** pick the minimum set that capture position/orientation
   and their first derivatives (velocity/angular rate).

## 2. Inventory the Forces & Torques
| Force Type | Law / Expression | Notes |
| --- | --- | --- |
| Inertia | `F = m·a`, `τ = I·α` | Newton’s 2nd law; always include. |
| Gravity | `W = m·g` | Acts downward (direction depends on frame). |
| Spring | `F_s = -k·x` or `-k(r - L₀)·r̂` | Hooke’s law; vector form points along the spring. |
| Damping | `F_d = -c·v` | Opposes velocity; removes energy. |
| Thrust | `T = k_t·ω²` | Rotor/motor models (quadcopter roadmap). |
| Normal/Constraint | Reaction forces/torques | Often computed via constraints (e.g., hinge, pivot). |

### Additional Force Families (for future models)
| Family | Example Laws | Notes / Use Cases |
| --- | --- | --- |
| Aerodynamic / Hydrodynamic | Lift `L = ½ρV²SC_L(α)`, Drag `D = ½ρV²SC_D(α)` | Acts on wings or props; depends on dynamic pressure and coefficients. |
|  | Moments `M = ½ρV²S c̄ C_m` | Pitch/roll/yaw moments for control surfaces. |
|  | Propeller slipstream models | Combine thrust and induced drag with blade-element or momentum theory. |
| Friction / Contact | Coulomb `F = μN`, rolling/viscous `F = -c_r·v` | Ground vehicles, joints with friction, collision impulses. |
| Buoyancy & Fluid Effects | `F_b = ρ_fluid V_displaced g`, added mass | Underwater/airship dynamics; hydrodynamic damping. |
| Rotating Frames | Coriolis `2m(ω×v)`, centrifugal `m ω×(ω×r)` | Appears when working in body frames (rigid-body attitude). |
| Electromagnetic / Actuators | `F = q v × B`, motor torque constants | Electric actuation, magnetic bearings. |
| Environment Disturbances | Wind shear, thrust vectoring, control surface deflection models | Needed for aerodynamic robustness tests. |

Add body forces/torques to the free-body diagram (FBD) before writing equations.

## 3. Apply Newton–Euler or Lagrange
- **Newton–Euler (force balance):**
  ```
  ΣF = m·a
  Στ = I·α
  ```
  Use when forces/torques are easy to enumerate.

- **Lagrange (energy-based):**
  ```
  L = T - U
  d/dt(∂L/∂q̇) - ∂L/∂q = Q
  ```
  Convenient for systems with constraints or generalized coordinates (pendula).

## 4. Typical Equation Patterns
| System | Coordinates | Governing Equation(s) | Notes |
| --- | --- | --- | --- |
| Spring–mass | `x` | `m·ẍ + c·ẋ + k·x = F_ext` | Add constant offset for gravity: equilibrium `x_eq = mg/k`. |
| Spring pendulum | `r`, `θ` or `x,y` | `m·r̈ = -k(r-L₀)·r̂ - c·v - m·g·ĵ` | Convert to Cartesian for implementation. |
| Double pendulum | `θ₁, θ₂` | Coupled Lagrange equations | Nonlinear; see `examples/double_pendulum`. |
| Inverted pendulum | `x, θ` | Cart force balance + torque equation | Linearize around `θ≈0` for control. |
| Point mass hover | `z` | `m·z̈ = ΣT_i - m·g` | Baseline for thrust/throttle mapping. |

## 5. Build the Equations Step-by-Step
1. Draw FBD, mark forces.
2. Write force balance per direction (or Lagrange equations).
3. Substitute constitutive laws (Hooke, damping, thrust).
4. Solve for accelerations (`ẍ`, `θ̈`, etc.).
5. Collect into state-space form (first-order equations) to prepare for
   simulation or linearization.

## 6. Example Blueprints

### Spring–Mass–Damper (1D)
- **Forces:** spring `-k·x`, damper `-c·ẋ`, gravity `-m·g`, external `F`.
- **Equilibrium:** `k·x_eq = m·g`.
- **Equation:** `m·ẍ + c·ẋ + k·(x - x_eq) = F`.
- **State vector:** `[x - x_eq, ẋ]^T`.

### Spring Pendulum
- **Forces:** spring along tether, damping along velocity, gravity.
- **Equation (vector):** `m·r̈ = -k(r-L₀)·r̂ - c·ṙ - m·g·ĵ`.
- **State vector:** `[x, y, vx, vy]^T`; compute energy via `½m‖v‖² + ½k(r-L₀)² + mgy`.

### Double Pendulum
- **Forces:** gravity on both links; hinge reactions handled implicitly.
- **Approach:** Use Lagrange with generalized coords `(θ₁, θ₂)`; equations appear
  in `examples/double_pendulum/main.c`. Watch for chaotic behavior.

### Inverted Pendulum on Cart
- **Forces:** cart thrust, cart damping, pole gravity, hinge damping.
- **Equations:** two coupled equations yielding `ẍ` and `θ̈` (see
  `examples/inverted_pendulum/main.c`).
- **Linearization:** around `θ ≈ 0`, create state `[x, θ, ẋ, θ̇]^T`.

## 7. Checklist Before Coding
- [ ] Frames defined and consistent across equations.
- [ ] All forces included (gravity, damping, spring, control inputs).
- [ ] Units consistent (SI base).
- [ ] State vector chosen; first-order system ready (`ẋ = f(x,u)`).
- [ ] Energies computed for sanity checks (conservation or decay).
- [ ] Linearization equilibrium identified (when needed).

## 8. Reference Documents
- `docs/forces_and_energies.md` – quick lookup for force laws.
- `docs/state_space_primer.md` – state-space form and linearization.
- `docs/linear_models.md` – derived `(A, B, C, D)` matrices.
- `docs/examples.md` – validation outputs for each example.

Use this guide as a blueprint for future models (quadcopter hover, attitude,
etc.) so the derivation steps stay consistent.***
