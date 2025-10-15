# Test Suite Guide

This README lives alongside the test sources so contributors can quickly see
what each executable exercises, which inputs it uses, and what output to expect
after running `ctest` or an individual binary.

## Running the Tests
```bash
cmake --build build          # builds all targets, including tests
cd build
ctest --output-on-failure    # runs the entire suite with console logs
```
To inspect a single test interactively:
```bash
./tests/<executable_name>
```

The table below mirrors the current executables in `build/tests/`.

| Executable | What it Covers | Inputs | Expected Output |
| --- | --- | --- | --- |
| `test_numerical_solvers` | Validates explicit Euler/RK4 on a 1D constant-acceleration system, enforces energy decay on the damped spring pendulum, benchmarks adaptive RK45 vs. fixed-step RK4 for the double pendulum, and cross-checks RK45 accuracy against a fine-step RK4 baseline for the inverted pendulum. A preamble explains each scenario, governing formulas, and the FBD elements involved. | Point mass: `a = 2 m/s²`, `dt = 0.01 s`, `steps = 100`. Spring pendulum: matches example parameters. Double pendulum: 5 s horizon with 0.05 s macro step. Inverted pendulum: 5 s horizon comparing RK4 (`dt = 0.001`) and RK45 (`dt = 0.05`). | Prints simulated vs. analytic trajectories, energy balance for each scenario, reports adaptive improvements on the double pendulum, and shows RK4/RK45 state differences below sub-millimetre/radian tolerances for the inverted pendulum. |
| `test_physics_model` | Validates the rigid-body vehicle dynamics: gravitational free fall, hover equilibrium, roll/pitch torques from off-axis thrust, and rotor reaction yaw torque. | 1-rotor craft, `m = 1 kg`, `g = 9.81 m/s²`, inertia diag `(0.02, 0.03, 0.04)`, scenario-specific thrust/torque coefficients. | Confirms acceleration equals gravity with zero thrust, cancels under hover thrust, and that angular accelerations match analytic `τ/I` for roll, pitch, and yaw cases. |
| `test_neural_state_space` | Placeholder for neural state-space identification utilities. | – | Outputs `[TODO] neural state-space tests pending implementation.` |
| `test_battery_model` | Placeholder for battery plant dynamics. | – | Outputs `[TODO] battery model tests pending implementation.` |
| `test_battery_management` | Placeholder for battery management logic. | – | Outputs `[TODO] battery management tests pending implementation.` |
| `test_motor_dynamics` | Placeholder for motor/ESC dynamics. | – | Outputs `[TODO] motor dynamics tests pending implementation.` |
| `test_point_mass_linear` | Confirms the analytic state-space matrices for the 1D point mass and checks error handling. | Mass `m = 1.2 kg`. | Logs the matrices, asserts `A = [[0,1],[0,0]]`, `B = [[0],[1/m]]`, `C = I`, `D = 0`, and verifies error codes on invalid inputs. |
| `test_mass_spring_linear` | Verifies the 1D mass–spring–damper linearization. | `m = 2.0 kg`, `k = 40 N/m`, `c = 1.2 N·s/m`. | Reports `A = [[0,1],[-k/m,-c/m]]`, `B = [[0],[1/m]]`, identity outputs, and asserts error handling on invalid parameters. |
| `test_spring_pendulum_linear` | Verifies the spring pendulum linear model about the hanging equilibrium and validates the equilibrium state. | `m = 0.5 kg`, `k = 15 N/m`, `c = 0.3 N·s/m`, `L₀ = 0.5 m`, `g = 9.81 m/s²`. | Prints equilibrium radius and matrix entries, checks analytic coefficients (`-(k/m)(1 - L₀ / r_eq)` and `-k/m`), and exercises invalid-parameter error paths. |
| `test_pendulum_linear` | Checks the small-angle planar pendulum linearization. | `m = 0.8 kg`, `L = 0.7 m`, `g = 9.81 m/s²`, `c = 0.05 N·m·s`. | Confirms `A = [[0,1],[-g/L,-c/(mL²)]]`, torque gain `1/(mL²)`, identity outputs, and invalid-parameter handling. |
| `test_inverted_pendulum_linear` | Validates the linearization of the upright cart-pole system including damping effects. | `M = 1.0 kg`, `m = 0.1 kg`, `l = 0.5 m`, `g = 9.81 m/s²`, `b_c = 0.05`, `b_p = 0.002`. | Confirms analytic expressions for `A`/`B`, ensures identity output matrices, equilibrium `(x,θ,ẋ,θ̇) = 0`, and checks error handling for invalid parameters. |
| `test_double_pendulum_linear` | Checks the downward double pendulum linearization with joint damping and torque inputs. | `m₁ = 1.0 kg`, `m₂ = 0.8 kg`, `L₁ = 0.6 m`, `L₂ = 0.4 m`, `g = 9.81 m/s²`, `c₁ = 0.05`, `c₂ = 0.02`. | Verifies the `A` block (`-M⁻¹G`, `-M⁻¹D`), the two-input `B = M⁻¹`, and identity outputs; exercises invalid parameter cases. |

The `attitudeMathLibrary` submodule ships its own test binaries (located under
`build/attitudeMathLibrary/`). Their focus and output format are described in
`docs/tests_overview.md`.

## Next Steps
- Replace each placeholder executable with targeted assertions once the
  corresponding subsystem is implemented.
- Extend the adaptive RK coverage to future models (inverted pendulum,
  quadrotor attitude) and promote the energy checks to regression plots once
  the CLI tooling lands.
