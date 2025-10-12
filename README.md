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
