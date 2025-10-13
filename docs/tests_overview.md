# Test Executable Overview

Running `ctest` (or individual binaries under `build/tests/` and
`build/attitudeMathLibrary/`) produces a set of helper programs. This page
collects what each one evaluates, the inputs it uses, and how to interpret the
printed numbers.

## Core Library Tests (`build/tests`)

| Executable | Description | Key Inputs | Output / Expectation |
| --- | --- | --- | --- |
| `test_numerical_solvers` | Verifies the Euler and RK4 integrators on a 1D constant-acceleration plant, checks spring-pendulum energy decay, compares fixed-step vs. adaptive RK on the double pendulum, and cross-validates RK45 against a fine-step RK4 baseline for the inverted pendulum. The executable prints a preamble summarizing each scenario, the governing equations, and the associated FBD context. | Point mass: `a = 2 m/s²`, `dt = 0.01 s`, `steps = 100`. Spring pendulum: same parameters as the example. Double pendulum: 5 s with 0.05 s macro step. Inverted pendulum: 5 s horizon with RK4 reference `dt = 0.001`. | Logs analytic vs. simulated position/velocity, energy balance, the adaptive-vs-fixed energy drift, and RK4 vs. RK45 state differences (sub-millimetre/radian tolerances). |
| `test_physics_model` | Placeholder until the rigid-body physics implementation lands. | None yet. | Emits a `[TODO]` message to remind developers the suite is waiting on the physics module. |
| `test_neural_state_space` | Placeholder for neural state-space identification tests. | None yet. | Outputs `[TODO]` marker. |
| `test_battery_model` | Placeholder for future battery plant verification. | None yet. | Outputs `[TODO]` marker. |
| `test_battery_management` | Placeholder for BMS logic tests. | None yet. | Outputs `[TODO]` marker. |
| `test_motor_dynamics` | Placeholder for motor/ESC dynamic response checks. | None yet. | Outputs `[TODO]` marker. |

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
| `test_euler_random` | Samples 1000 random orientations and checks round-trip error. | Lists per-sample angle errors. Failures (currently observed) flag known tolerance issues slated for upstream fixes. |
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
