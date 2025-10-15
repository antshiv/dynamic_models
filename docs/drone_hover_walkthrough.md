# Drone Hover Walkthrough

This note explains, step by step, how the quadrotor hover example and its unit
tests tie together. The goal is to show how the basic physics you learn early—
“force equals mass times acceleration” and “torque equals inertia times angular
acceleration”—show up directly in the code.

- **Source:** `examples/drone/example_physics_model.c`
- **Physics core:** `src/drone/physics_model.c`
- **Regression tests:** `tests/drone/test_physics_model.c`

We assume you already know what a quaternion is and that you’re comfortable with
basic calculus (derivatives) and vectors.

---

## 1. Building the Drone Model

1. **Mass and inertia tensor.** In `example_physics_model.c:55`, the quad is
   given a total mass of 1.6 kg and an inertia matrix
   `diag(0.03, 0.03, 0.05)` in kg·m². This is the “how hard is it to move or
   rotate” part of Newton’s second law.
2. **Rotor locations and thrust law.** Lines `63-82` define each rotor’s
   position, spin axis, and coefficients `k_t`, `k_q` for thrust and reaction
   torque. The simple model relies on the quadratic law `F = k_t · ω²`.
3. **State representation.** A `dm_state_t` contains position, velocity,
   quaternion, and angular velocity. When we integrate, we advance this whole
   package.
4. **Physics evaluator.** `src/drone/physics_model.c:1` is the engine. Given the
   current state and rotor speeds it:
   - sums thrust forces in the body frame,
   - converts them to the inertial frame using the quaternion-derived rotation
     matrix,
   - adds gravity, and
   - computes body torques (moment arm `r × F` plus reaction torque), feeding the
     rigid-body equations `τ = I·α + ω×(I·ω)`.

If all four rotors use the same speed, the thrust forces are symmetric. Their
sum equals weight (hover), and their torques cancel, so translational
acceleration equals gravity minus thrust, and angular acceleration is zero.

---

## 2. Hover With Symmetric Thrust

1. In the example, we compute a single `hover_omega` (`example_physics_model.c:99`)
   such that each rotor’s thrust is `m·g / 4`. That means four rotors → total
   thrust = `m·g`.  
2. We fill the `inputs.rotor_omega[]` array with that speed, call
   `dm_vehicle_evaluate`, and integrate with RK4 (`example_physics_model.c:123`).
   Because torques cancel, the angular acceleration returned in `state_dot` is
   all zeros, so the quaternion derivative is zero and orientation stays fixed.
3. The printed lines at `example_physics_model.c:131` show altitude, vertical
   velocity, roll, and pitch. For the first second, everything remains zero (or
   near floating-point zero). That is how hover should look for a perfectly
   balanced vehicle.

---

## 3. Pitch Disturbance (Front vs. Back Rotors)

1. To demonstrate pitch, the example tweaks the front rotors up 1 % and the rear
   rotors down 1 % during the interval `1.0 s < t ≤ 1.1 s`
   (`example_physics_model.c:137`).
2. That imbalance means the net thrust force now points slightly forward rather
   than straight up. The moment arm produces a body torque about the pitch axis.
3. Inside `dm_vehicle_evaluate`, the torque appears in `total_torque_body`. When
   we compute angular acceleration (`α = I⁻¹ · (τ - ω×(I·ω))`) we get a positive
   pitch rate. This feeds the quaternion derivative `q̇ = ½ Ω(q) ω`
   (`physics_model.c:117`).
4. When you look at the output around `t = 1.1 s`, pitch starts drifting from
   zero while roll remains near zero. That matches intuition: only front/back
   thrust changed.

---

## 4. Roll Disturbance (Left vs. Right Rotors)

1. Later, between `1.4 s < t ≤ 1.5 s`, the example boosts the right rotors and
   eases the left rotors (`example_physics_model.c:142`). That creates a torque
   about the roll axis.
2. The same rigid-body math now produces a non-zero roll acceleration. The
   quaternion derivative moves the attitude about the body X-axis.
3. In the console output you’ll see roll begin to drift after 1.4 s while pitch
   stays near its earlier value.

Note that without a controller, both pitch and roll keep wandering—there’s
nothing to bring the vehicle level again. That is expected and reinforces why a
feedback loop (from the control library) is needed.

---

## 5. Unit Tests for Torques

`tests/drone/test_physics_model.c:1` provides small, isolated checks:

1. **Roll torque (`test_offset_rotor_generates_roll_torque`).** Move a single
   rotor to `y = 0.1 m` and spin it. The expected roll acceleration is
   `(y × thrust) / I_x`. The test verifies the code returns that value.
2. **Pitch torque (`test_offset_rotor_generates_pitch_torque`).** Same idea with
   a rotor at `x = 0.12 m`: expected pitch acceleration is `(x × thrust) / I_y`.
3. **Yaw torque (`test_reaction_torque_about_yaw`).** Uses the rotor’s drag
   coefficient to ensure `ω² k_q` shows up as the yaw acceleration.

These tests keep the math honest; if a future edit breaks the cross product or
inertia handling, the assertions fail immediately.

---

## 6. Quaternion Update Intuition

`physics_model.c:108-119` computes:

```
q̇ = ½ * [  0   -ωₓ -ω_y -ω_z
            ωₓ   0    ω_z -ω_y
            ω_y -ω_z   0    ωₓ
            ω_z  ω_y -ωₓ   0  ] * q
```

If `ω = 0`, then `q̇ = 0` and the orientation stays put. As soon as a torque
creates a non-zero `ω`, the quaternion starts moving. Because rotor torques feed
directly into `ω̇`, and integrating `ω̇` gives `ω`, the quaternion update is the
bridge from physical torque to “what does the drone look like in space.”

---

## 7. Summary

- Equal rotor speeds → thrust sums to weight, torques cancel → hover.
- Change thrust front/back → torque about pitch → non-zero pitch rate →
  quaternion tilts forward.
- Change thrust left/right → torque about roll → non-zero roll rate →
  quaternion tilts sideways.
- The hover example and the accompanying tests are two sides of the same coin:
  one shows the effect over time, the other verifies the math in a single step.

When the control library comes online, it will sit above this layer and adjust
rotor speeds to drive the rates back to zero—closing the loop you see open in
`example_physics_model.c`.
