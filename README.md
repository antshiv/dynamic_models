# Dynamic Models Library

This repository hosts physics-based and data-driven modules for aerial vehicle dynamics:
- Multi-rotor rigid-body modeling (quad, hex, heli)
- Motor and propulsion dynamics
- Battery and power-train modelling (planned)
- System-identification and neural state-space utilities (planned)

## Current Layout
- `include/` – Public headers for drone, motor, battery, utilities.
- `src/` – Implementation for each subsystem (currently stubs to be filled).
- `data/` – Example datasets for identification and validation.
- `tests/` – Unit and integration tests.
- `examples/` – Sample simulations that exercise the models.

## Hover Computation Graph
The first concrete deliverable is a hover-capable plant that feeds downstream control libraries.

```
motor command ωᵢ
      │
      ▼
┌────────────────────────────┐
│ Motor/ESC dynamics         │  electrical → mechanical
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ Rotor aerodynamics         │  thrust/torque = f(ωᵢ, params)
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ Force/torque aggregator    │  sums thrust, maps torques
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ Newton–Euler rigid body    │  m·ξ̈ = -mgD + R·F
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ Integrator (RK4/Euler)     │  updates state {x,y,z,ψ,θ,φ}
└────────────────────────────┘
      │
      ▼
  next state
```

- **Configurable inputs**: mass, inertia tensor, rotor geometry, thrust/drag maps, gravity.
- **High-level defaults**: simple quadratic thrust model (`T = kₜ ω²`, `Q = k_q ω²`). Higher-fidelity modules (momentum theory, blade element) simply replace that block.
- **Output**: time evolution of position, velocity, attitude, angular rates—ready for controller-in-the-loop testing.

## Terminology & Notation
- **RK4** – Classical fourth-order Runge–Kutta integrator. It approximates the continuous-time differential equations by sampling intermediate slopes (`k1…k4`) over a timestep and combining them for higher accuracy than simple Euler integration.
- **`T = k_t ω²`** – Simplified rotor thrust law. `T` is thrust (N), `ω` is rotor angular speed (rad/s), `k_t` is a thrust coefficient that lumps propeller geometry, air density, and motor characteristics.
- **`Q = k_q ω²`** – Simplified reaction torque law. `Q` is torque about the rotor axis, `k_q` a drag coefficient related to blade profile and motor losses.
- **`ξ = (x,y,z)`** – Position of the vehicle center of mass in the inertial frame (we use NED).
- **`η = (ψ, θ, φ)`** – Yaw, pitch, roll Euler angles describing the attitude.
- **`R(η)`** – Rotation matrix that maps body-frame forces into the inertial frame; derived from the Euler-angle sequence chosen in the PDF reference.
- **`m·ξ̈ = -mgD + R·F`** – Newton–Euler translational equation: total thrust counteracts weight (`mg`) and produces acceleration.
- **Hover condition** – Sum of rotor thrusts equals weight: `Σ T_i = mg`. For a symmetric quad each rotor provides ~`mg/4` in steady hover, modulated by attitude corrections.
- **Rotor weighting** – Changes in individual thrusts (e.g., rear vs. front) generate body torques responsible for pitch/roll/yaw responses as shown in Eq. (2.67) of *Quadcopter Modelling*.

These placeholders will be replaced by higher-fidelity functions or lookup tables (e.g., blade element theory) as we incorporate richer physics, but the interface to the dynamics core remains the same.

## High-Level Derivation Path
1. **Adopt frames & conventions** – Use the PDF’s NED inertial frame and body-fixed axes to stay consistent across math, code, and controllers.
2. **Summarize rotor forces/torques** – Start with the quadratic thrust/drag approximations above; later substitute functions derived from momentum or blade-element theory.
3. **Apply Newton–Euler** – Follow Sect. 2.2 to derive the translational (`mẍ, mÿ, mz̈`) and rotational (`ψ̈, θ̈, φ̈`) equations with the chosen frame sequence.
4. **Solve for hover** – Set accelerations to zero to confirm `Σ T_i = mg` and extract per-rotor thrust requirements (≈`mg/4` for a symmetric quad).
5. **Integrate** – Use RK4 (or Euler for quick checks) to propagate the differential equations in time, driven by rotor commands.
6. **Iterate fidelity** – Refine `k_t`, `k_q`, and motor dynamics via theory or test data without changing the surrounding structure.

## Roadmap
1. State-space toolkit: linearize the spring pendulum, inverted pendulum, and double pendulum around their operating points and expose `(A,B,C,D)` helpers (point mass already done).
2. Quaternion-aware rigid-body propagation: integrate the attitudeMathLibrary quaternions into the RK4 integrator with normalization each step to avoid gimbal lock.
3. Motor/ESC dynamics: implement first-order lag (`τ·ω̇ + ω = ω_cmd`) and extend with measured propeller thrust/drag maps.
4. JSON-based vehicle configuration loader to drive the hover computation graph and examples with real hardware parameters.
5. CLI automation: generate state-space/Laplace models, step/Bode plots, and regression comparisons directly from configuration files.
6. Extend to lateral dynamics, wind disturbances, and alternative airframes once the above layers are stable.

## Building
1. Clone the repository:
   ```bash
   git clone https://github.com/antshiv/dynamic_models.git
   cd dynamic_models
   git submodule update --init --recursive  # pulls attitudeMathLibrary
   ```

## Examples
- `point_mass_example` – Uses the RK4 solver to integrate a 1D point mass under constant acceleration (see `examples/point_mass/main.c`). Build with `cmake --build . --target point_mass_example` and run the resulting binary to inspect the position/velocity timeline. This mirrors the hover axis before adding rotor thrust: replace the constant acceleration with `(ΣT - mg)/m` to transition into the full Newton–Euler model.
- `spring_pendulum_example` – Simulates a mass on a spring in a gravity field to demonstrate coupled translational dynamics (see `examples/spring_pendulum/main.c`). Build with `cmake --build . --target spring_pendulum_example` and run the binary to observe damped oscillations sampled every 0.5 s.
- `double_pendulum_example` – Captures chaotic two-link dynamics with RK4 integration and energy monitoring (see `examples/double_pendulum/main.c`). Build with `cmake --build . --target double_pendulum_example` and run the binary to inspect angle/rate/energy traces (optional CSV output supported by passing a file path argument to any example).
- `inverted_pendulum_example` – Models a cart-pole around the upright equilibrium with damping and energy tracking (see `examples/inverted_pendulum/main.c`). Build with `cmake --build . --target inverted_pendulum_example` and run the binary to watch the unstable mode evolve (CSV export supported via optional file path argument).
All examples accept an optional command-line argument with a CSV file path; when provided, the simulation streams the full time series for downstream plotting.

See `docs/free_body_diagrams.md` for ASCII sketches of the point mass, spring pendulum, and forthcoming inverted pendulum free-body diagrams, along with inertia references for multirotors. `docs/forces_and_energies.md` collects the key force/energy identities (thrust, weight, Hooke’s law, damping, torque) so you can translate between FBDs and algebra quickly. `docs/dynamics_analysis_guide.md` outlines the step-by-step blueprint (forces, laws, state choice) for modeling new systems, while `docs/intuitive_analysis_flow.md` shows how to bounce between forces, torques, state-space, and response intuition. `docs/state_space_primer.md` explains the `ẋ = A x + B u`, `y = C x + D u` form we use when linearizing the nonlinear models, and `docs/linear_models.md` enumerates the derived matrices (currently the point mass, with pendulum models to follow). A deeper narrative on integrators lives in `docs/numerical_methods.md`. Solver validation snippets for the examples are collected in `docs/examples.md`. Refer to `docs/tests_overview.md` when you want a quick description of each unit test executable and its expected console output.

## Reference Tables

### Example Dynamics Modules
| Scenario | Purpose | Numerical Methods | Current Status |
| --- | --- | --- | --- |
| Point mass (1D) | Minimal translational plant for solver bring-up | Euler, RK4 | Implemented (`examples/point_mass`) |
| Spring pendulum | Elastic pendulum showing coupled translation/rotation | Euler, RK4, adaptive RK | Implemented (`examples/spring_pendulum`) |
| Inverted pendulum | Balancing benchmark for control loops | RK4, implicit Euler, model-predictive integrators | Implemented (`examples/inverted_pendulum`) |
| Planar double pendulum | Energy exchange and chaos benchmark | RK4, adaptive RK | Implemented (`examples/double_pendulum`) |

### Numerical Integration Methods
| Method | Order | Stability Traits | Typical Use | Notes |
| --- | --- | --- | --- | --- |
| Forward Euler | 1st | Conditionally stable; fast but diffusive | Quick prototyping, sanity checks | Already exposed via `dm_integrate_euler` |
| Classical RK4 | 4th | Good accuracy with fixed step sizes | Core solver for rigid-body propagation | Implemented as `dm_integrate_rk4` |
| Adaptive RK (Dormand–Prince) | 5/4 pair | Step-size control improves efficiency | Planned upgrade for stiff scenarios | To be slotted into `utilities` once error control scaffold exists |
| Implicit Euler / trapezoidal | 1st/2nd implicit | Unconditionally stable for stiff systems | Contact dynamics, inverted pendulum | Will require linear solver backend |

### First-Principles Modeling Paths
| Approach | Governing Equations | Strengths | Use Cases |
| --- | --- | --- | --- |
| Newtonian (force/torque balance) | `ΣF = m·a`, `Στ = I·α` | Direct mapping from FBD to dynamics; intuitive | Multirotor hover, motor thrust aggregation |
| Lagrangian (energy based) | `d/dt(∂L/∂q̇) - ∂L/∂q = Q` | Handles constraints elegantly; symbolic friendly | Pendulum variants, flex modes, underactuated rigs |
| Hamiltonian | `q̇ = ∂H/∂p`, `ṗ = -∂H/∂q + Q` | Explicit energy preservation; good for control canonical forms | Future optimal control and estimation pipelines |

### FBD Composition Chain (Multirotor Example)
| Block | FBD Summary | Purpose in Chain | Notes |
| --- | --- | --- | --- |
| Rotor disk | Thrust and drag forces along rotor axis | Converts motor torque/current to lift & reaction torque | Parameterized by `k_t`, `k_q`, or blade-element tables |
| Motor/ESC | Electrical input vs. shaft torque | Captures lag and saturation in thrust commands | Links controller outputs to rotor FBD |
| Arm + frame | Couples individual rotor forces into net body forces/torques | Builds body wrench from per-rotor thrust vectors | Geometry matrix maps rotor frame to body frame |
| Rigid body | Summed forces/torques acting on CoM | Produces translational and rotational acceleration | Uses Newton–Euler or Lagrangian forms above |
| Integrator | State propagation (position, velocity, attitude, rates) | Evolves vehicle state in time | Backed by chosen numerical method (Euler, RK4, etc.) |
| Disturbance models | Wind, payload shifts, bias torques | Stress-tests robustness | Optional injection layer for later roadmap items |

These tables act as a quick index: pick the FBD building blocks, choose the governing equations, and select an integration method to assemble models ranging from a 1D hover axis to a full quadcopter. The roadmap items will gradually populate the planned rows with concrete code and documentation.
