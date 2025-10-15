# Example Validation Notes

This document captures the numerical checks that accompany the example
executables. Use it alongside `docs/numerical_methods.md` and
`docs/free_body_diagrams.md` when inspecting solver accuracy. Each example
accepts an optional command-line argument with a CSV file path; when provided,
the full time series is recorded for plotting or CLI-driven reports.

## Point Mass Constant Acceleration
- **Scenario:** 1 kg mass subject to constant thrust producing `a = 2 m/s²`.
- **Integrator:** `dm_integrate_rk4` (example) and both Euler/RK4 in
  `test_numerical_solvers`.
- **Analytic expectation:** `x(t) = 0.5·a·t²`, `v(t) = a·t`,
  `KE = 0.5·m·v²`, Work by thrust = `F·x`.
- **Validation:**  
  - Example output lists time, position, velocity, kinetic energy, and work at
    0.1 s increments (`examples/point_mass/main.c`).  
  - Unit test compares Euler and RK4 trajectories against closed-form values and
    reports the kinetic energy vs. work balance. Euler exhibits a small energy
    error (~2×10⁻² J) due to first-order drift, while RK4 matches to <10⁻⁹ J.

## Damped Spring Pendulum
- **Scenario:** 0.5 kg mass on a spring (`k = 15 N/m`, rest length 0.5 m) with
  light damping (`c = 0.3 N·s/m`) in gravity.
- **Integrator:** `dm_integrate_rk4`.
- **Energies tracked:** Kinetic `½m‖v‖²`, spring potential
  `½k(r - L₀)²`, gravitational `mgy`.
- **Validation:**  
  - Example output (every 0.5 s) enumerates position, velocity, and each energy
    component plus the total.  
  - Unit test `test_spring_pendulum_energy_decay` ensures the total mechanical
    energy decays monotonically (allowing ≤10⁻⁶ J numerical noise) and logs the
    initial/final energy gap. This confirms damping removes energy as expected.

## Planar Double Pendulum
- **Scenario:** Two 1 kg links (`l₁ = l₂ = 1 m`) with gravity, starting from
  `θ₁ = θ₂ = π/2` and zero velocities.
- **Integrator:** `dm_integrate_rk4` in the example; adaptive Dormand–Prince
  (`dm_integrate_rk45_adaptive`) in the test harness.
- **Energies:** Kinetic energy combines link velocities while potential uses
  `mgy` for each mass relative to the pivot height.
- **Validation:**  
  - Example prints angles, angular rates, and energy components every 0.2 s,
    and emits a CSV when a file path argument is supplied.  
  - Unit test `test_double_pendulum_adaptive_energy` compares fixed-step RK4 and
    adaptive RK45 over 5 s; the adaptive method reduces energy drift by over an
    order of magnitude (<10⁻² J absolute error).

## Inverted Pendulum on a Cart
- **Scenario:** 1 kg cart with 0.1 kg pole (`l = 0.5 m`), small damping, and a
  5° initial tip from the upright position.
- **Integrator:** `dm_integrate_rk4` for the example; adaptive RK45 in the test.
- **Quantities tracked:** Cart position/velocity, pole angle/rate, kinetic and
  potential energy.
- **Validation:**  
  - Example logs state and energy every 0.2 s and supports CSV export.  
  - Unit test `test_inverted_pendulum_adaptive` integrates 5 s with RK45 and
    confirms energy remains bounded (max step increase ≤ 10⁻⁴ J) thanks to
    damping.

## Quadrotor Hover (Drone)
- **Scenario:** 1.6 kg quadrotor with 0.23 m arm length, thrust axis aligned
  with body `-Z`, and a symmetric inertia tensor (`diag(0.03, 0.03, 0.05)`).
  Rotor thrust/torque gains are tuned so `ω ≈ 511.5 rad/s` balances weight in
  hover.
- **Integrator:** `dm_integrate_rk4`.
- **Quantities tracked:** altitude (`-z` in NED), vertical velocity, and Euler
  roll/pitch angles reconstructed from the quaternion.
- **Validation:**  
  - Example prints the state every 0.1 s. The first second demonstrates steady
    hover. Between 1.0–1.1 s, front rotors receive a 1% boost and rear rotors a
    1% reduction, producing a gentle pitch response while thrust symmetry keeps
    roll ≈ 0°.  
  - Because no feedback controller or drag is modeled yet, the vehicle slowly
    descends after the pulse—highlighting the need for control loops even in
    otherwise balanced thrust configurations.

## Next Steps
- Add CSV output support to the quadrotor example once controller hooks are
  integrated, enabling trajectory plots alongside the existing point-mass and
  pendulum cases.
