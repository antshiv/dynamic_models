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
| `test_physics_model` | Placeholder until the rigid-body physics implementation lands. | – | Outputs `[TODO] physics model verification pending implementation.` |
| `test_neural_state_space` | Placeholder for neural state-space identification utilities. | – | Outputs `[TODO] neural state-space tests pending implementation.` |
| `test_battery_model` | Placeholder for battery plant dynamics. | – | Outputs `[TODO] battery model tests pending implementation.` |
| `test_battery_management` | Placeholder for battery management logic. | – | Outputs `[TODO] battery management tests pending implementation.` |
| `test_motor_dynamics` | Placeholder for motor/ESC dynamics. | – | Outputs `[TODO] motor dynamics tests pending implementation.` |
| `test_point_mass_linear` | Confirms the analytic state-space matrices for the 1D point mass and checks error handling. | Mass `m = 1.2 kg`. | Logs the matrices, asserts `A = [[0,1],[0,0]]`, `B = [[0],[1/m]]`, `C = I`, `D = 0`, and verifies error codes on invalid inputs. |

The `attitudeMathLibrary` submodule ships its own test binaries (located under
`build/attitudeMathLibrary/`). Their focus and output format are described in
`docs/tests_overview.md`.

## Next Steps
- Replace each placeholder executable with targeted assertions once the
  corresponding subsystem is implemented.
- Extend the adaptive RK coverage to future models (inverted pendulum,
  quadrotor attitude) and promote the energy checks to regression plots once
  the CLI tooling lands.
