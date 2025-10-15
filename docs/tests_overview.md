# Test Executable Overview

Running `ctest` (or individual binaries under `build/tests/` and
`build/attitudeMathLibrary/`) produces a set of helper programs. This page
collects what each one evaluates, the inputs it uses, and how to interpret the
printed numbers.

## Core Library Tests (`build/tests`)

| Executable | Description | Key Inputs | Output / Expectation |
| --- | --- | --- | --- |
| `test_numerical_solvers` | Verifies the Euler and RK4 integrators on a 1D constant-acceleration plant, checks spring-pendulum energy decay, compares fixed-step vs. adaptive RK on the double pendulum, and cross-validates RK45 against a fine-step RK4 baseline for the inverted pendulum. The executable prints a preamble summarizing each scenario, the governing equations, and the associated FBD context. | Point mass: `a = 2 m/s²`, `dt = 0.01 s`, `steps = 100`. Spring pendulum: same parameters as the example. Double pendulum: 5 s with 0.05 s macro step. Inverted pendulum: 5 s horizon with RK4 reference `dt = 0.001`. | Logs analytic vs. simulated position/velocity, energy balance, the adaptive-vs-fixed energy drift, and RK4 vs. RK45 state differences (sub-millimetre/radian tolerances). |
| `test_physics_model` | Checks the rigid-body plant: verifies gravity-only free fall, hover trim cancellation, moment-arm torques about roll and pitch, and yaw reaction torque. | 1-rotor vehicle, `m = 1 kg`, `g = 9.81 m/s²`, inertia diag `(0.02, 0.03, 0.04)`, thrust/torque coefficients chosen per scenario. | Asserts acceleration matches `g` with zero thrust, zeroes under hover thrust, and that angular accelerations equal analytically computed `τ/I` for roll, pitch, and yaw. |
| `test_neural_state_space` | Placeholder for neural state-space identification tests. | None yet. | Outputs `[TODO]` marker. |
| `test_battery_model` | Placeholder for future battery plant verification. | None yet. | Outputs `[TODO]` marker. |
| `test_battery_management` | Placeholder for BMS logic tests. | None yet. | Outputs `[TODO]` marker. |
| `test_motor_dynamics` | Placeholder for motor/ESC dynamic response checks. | None yet. | Outputs `[TODO]` marker. |
| `test_point_mass_linear` | Verifies the generated `A`, `B`, `C`, `D` matrices for the point-mass state-space model and exercises error handling. | Mass parameter (`m = 1.2 kg`). | Prints the matrices and asserts they match the analytic derivation; confirms error codes for invalid inputs. |
| `test_mass_spring_linear` | Checks the classic 1D mass–spring–damper linearization and confirms damping/force gains. | `m = 2.0 kg`, `k = 40 N/m`, `c = 1.2 N·s/m`. | Logs the `A/B` entries (`-k/m`, `-c/m`, `1/m`) and exercises invalid-parameter error handling. |
| `test_spring_pendulum_linear` | Checks the spring pendulum linearization about the hanging equilibrium and validates the reported equilibrium state. | `m = 0.5 kg`, `k = 15 N/m`, `c = 0.3 N·s/m`, `L₀ = 0.5 m`, `g = 9.81 m/s²`. | Compares `A`, `B`, `C`, `D` entries against analytic formulas and exercises error handling paths. |
| `test_pendulum_linear` | Validates the small-angle planar pendulum model and torque input gain. | `m = 0.8 kg`, `L = 0.7 m`, `g = 9.81 m/s²`, `c = 0.05 N·m·s`. | Prints the `A/B` entries (`-g/L`, `-c/(mL²)`, `1/(mL²)`) and checks error cases for invalid geometry. |
| `test_inverted_pendulum_linear` | Verifies the cart-pole linear model around the upright equilibrium (unstable) including damping terms. | `M = 1.0 kg`, `m = 0.1 kg`, `l = 0.5 m`, `g = 9.81 m/s²`, `b_c = 0.05`, `b_p = 0.002`. | Checks the analytic coefficients for the `A`/`B` matrices, ensures `C = I`, `D = 0`, and validates error handling. |
| `test_double_pendulum_linear` | Validates the hanging double pendulum linearization with joint damping and torque inputs. | `m₁ = 1.0 kg`, `m₂ = 0.8 kg`, `L₁ = 0.6 m`, `L₂ = 0.4 m`, `g = 9.81 m/s²`, `c₁ = 0.05`, `c₂ = 0.02`. | Confirms the `A/B` blocks derived from `M⁻¹G`, `M⁻¹D`, and ensures identity outputs plus error handling. |

As the subsystem implementations arrive, each placeholder will be replaced by
real assertions (and the `[TODO]` prints removed).

## Attitude Math Library Tests (`build/attitudeMathLibrary`)

These come from the external `attitudeMathLibrary` submodule and exercise
quaternion/Euler conversion utilities:

| Executable | Description | Output interpretation |
| --- | --- | --- |
| `test_attitude` | Converts Euler angles → DCM → quaternion → back. | Prints the original angles, quaternion components, and recovered angles. Matches indicate the conversions are coherent. |
| `test_attitude_degrees` | Same as above but using degree-based APIs. | Shows original and recovered Euler angles in degrees. |
| `test_dcm_orthogonal` | Confirms a rotation matrix stays orthonormal. | Prints `PASS` when the orthogonality check passes (tolerance < 1e-6). |
| `test_euler` | Round-trip conversion at a single canonical orientation. | Displays original vs. recovered Euler angles. |
| `test_euler_random` | Samples 1000 random orientations, reconstructs the DCM, and enforces a 1e-9 Frobenius-norm match after round-tripping. | Reports max angle drift (informational) plus the DCM error; fails if the matrix mismatch exceeds the tolerance. |
| `test_quaternion` / `test_quaternion_inverse_axis` / `test_quaternion_slerp` | Validate quaternion algebra and spherical linear interpolation. | Log the input quaternions and results, ensuring norms and axis inversions behave. |
| `test_rotation` | Sweeps a quaternion rotation over time. | Prints time, attitude, and quaternion components for inspection. |

## Using the Tests
1. Build: `cmake --build build`
2. Run all tests: `cd build && ctest --output-on-failure`
3. Run a specific binary: `./tests/test_numerical_solvers` (for example) to
   inspect its console output directly.

Combine this page with `docs/free_body_diagrams.md` to map the test quantities
back to their physical interpretations.

For a quick-reference summary stored alongside the sources, see
`tests/README.md`.
