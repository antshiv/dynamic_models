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
1. JSON-based vehicle configuration loader that populates the hover graph.
2. Newton–Euler core implementation with unit tests and hover equilibrium checks.
3. Simulation harness for step/sine inputs with CSV output for analysis (Python/SciPy or Octave).
4. Motor/propulsion upgrades: replace constants with measured or blade-element models.
5. Extend to lateral dynamics, wind disturbances, and alternative airframes.

## Building
1. Clone the repository:
   ```bash
   git clone https://github.com/antshiv/dynamic_models.git
   cd dynamic_models
