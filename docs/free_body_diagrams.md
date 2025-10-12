# Free-Body Diagrams and Composition Notes

This page sketches the free-body diagrams (FBDs) for the core example systems
and outlines how they stack together to form a multirotor model. ASCII art keeps
the diagrams lightweight while still identifying forces and axes.

## Point Mass (1D Gravity Axis)
```
  z ↑
    │   T = k·u  (applied thrust)
    ●───┐  ↑
    │   │  │
    ▼   │  │
  mg    │  │
  (down)┘  │
```
- **Equations:** `m·ẍ = T - mg`
- **Use in repo:** Validates that the RK4 integrator matches analytic motion
  under constant acceleration.
- **Extension:** Replace `T` with the sum of rotor thrusts in a quad to recover
  the hover axis.

## Spring Pendulum (Mass on Elastic Tether)
```
      y ↑
        │
        ● (mass m)
       ↙│↘
    kΔL │  (spring force along tether)
        │
        └───╱╲─── anchor
    damping c⋅v
    gravity mg ↓
```
- **Equations:** `m·r̈ = -k(r - L₀)·r̂ - c·v - m·g·ĵ`
- **Use in repo:** Demonstrates coupled translation/rotation and energy exchange
  between potential (spring) and kinetic terms.
- **Extension:** Swap the spring with a rotor thrust vector to mimic tethered
  drones or boom-mounted sensors.

## Inverted Pendulum on Cart (Planned)
```
            ↑ Fy (constraint)
            ● (mass m)
            │
            │   τ = reaction about pivot
            │
     ┌──────┴──────┐
     │    cart     │ → Fx (actuator)
     └─────────────┘
```
- **Equations:** Lagrangian formulation couples cart translation with pendulum
  rotation. Used to benchmark controllers and implicit integration when
  stiffness grows.
- **Status:** Documentation placeholder; implementation planned alongside the
  controller-in-the-loop demos.

## Multirotor Composition Chain
```
  Motor/ESC  →  Rotor Disk  →  Force/Torque Aggregator  →  Rigid Body  →  Integrator
     │              │                   │                     │                 │
 electrical     thrust/drag         ΣF, Στ maps           Newton–Euler     Euler / RK4 /
   inputs         FBDs             to body wrench          I·ω̇, m·a        implicit (future)
```
- **Rotor disk FBD:** Lift `T_i` along rotor axis, drag `Q_i` about the shaft.
- **Frame aggregation:** Geometry matrix `B` maps per-rotor forces into body
  torques (`τ = B·T`).
- **Rigid body:** Uses translational and rotational equations with the inertia
  tensor described below.

## Rotational Inertia Quick Reference
When extending to attitude dynamics, each rigid body segment contributes a
moment of inertia about the principal axes. Common shapes:
- **Solid cylinder / disk (radius r, mass m):**
  - About central axis: `I_z = 0.5·m·r²`
  - About diametral axis: `I_x = I_y = 0.25·m·r² + (1/12)·m·h²`
- **Rod about center (length L):** `I = (1/12)·m·L²`
- **Rod about end:** `I = (1/3)·m·L²`
- **Rectangular prism (a × b × c):** `I_x = (1/12)·m·(b² + c²)` and cyclic permutations.

These inertias populate the diagonal (or full) inertia tensor `I`. The body-rate
dynamics use:
```
I·ω̇ + ω × (I·ω) = τ_body
```
where `τ_body` is the net torque assembled from the rotor FBDs. Matching torque
capability to the inertia determines the motor sizing required to achieve
desired roll/pitch/yaw accelerations.

---

**Next documentation steps**
1. Replace ASCII diagrams with SVGs once the project adopts a documentation
   site generator.
2. Add measured inertia examples for specific airframes to bridge the gap
   between textbook formulas and CAD-exported inertia tensors.
