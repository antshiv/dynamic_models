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
| `test_numerical_solvers` | Validates the explicit Euler and RK4 integrators on a 1D constant-acceleration system. Also checks that RK4 refuses to run if scratch buffers are missing. | Acceleration `a = 2 m/s²`, step size `dt = 0.01 s`, iteration count `steps = 100`. | Prints both simulated and closed-form position/velocity values and asserts they match within `1e-12` (Euler) / `1e-9` (RK4). Logs a message confirming the "missing scratch" guard leaves the state unchanged. |
| `test_physics_model` | Placeholder until the rigid-body physics implementation lands. | – | Outputs `[TODO] physics model verification pending implementation.` |
| `test_neural_state_space` | Placeholder for neural state-space identification utilities. | – | Outputs `[TODO] neural state-space tests pending implementation.` |
| `test_battery_model` | Placeholder for battery plant dynamics. | – | Outputs `[TODO] battery model tests pending implementation.` |
| `test_battery_management` | Placeholder for battery management logic. | – | Outputs `[TODO] battery management tests pending implementation.` |
| `test_motor_dynamics` | Placeholder for motor/ESC dynamics. | – | Outputs `[TODO] motor dynamics tests pending implementation.` |

The `attitudeMathLibrary` submodule ships its own test binaries (located under
`build/attitudeMathLibrary/`). Their focus and output format are described in
`docs/tests_overview.md`.

## Next Steps
- Replace each placeholder executable with targeted assertions once the
  corresponding subsystem is implemented.
- Expand the numerical solver test to cover additional trajectories (e.g., the
  spring pendulum energy profile) as the utilities layer grows.
