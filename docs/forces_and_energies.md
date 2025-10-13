# Force & Energy Equivalences

This cheat sheet lists the core force and energy relationships that show up in
the example models (point mass, spring pendulum, double pendulum) and extend
into multirotor hover/attitude modeling. Use it to translate between different
representations—work/energy, constitutive laws, and Newtonian balance—when
reasoning about the dynamics.

## Newtonian Force Balance
| Symbol | Expression | Interpretation | Example Usage |
| --- | --- | --- | --- |
| `F = m·a` | Force equals mass times acceleration. | Translational dynamics in any inertial frame. | Point mass; hover axis (`ΣT - mg = m·ẋ̈`) |
| `τ = I·α` | Torque equals inertia times angular acceleration. | Rotational analogue of Newton’s 2nd law. | Planned multirotor roll/pitch/yaw models |
| `ΣF = 0` (equilibrium) | Net force zero at steady state. | Hover trim or static deflection. | `mg = ΣT_i` for quad hover |

## Gravitational Weight & Potential
| Quantity | Expression | Notes |
| --- | --- | --- |
| Weight (force) | `W = m·g` (direction depends on frame) | Acts downward in the examples (`+z` down). |
| Gravitational potential | `U_g = m·g·h` | Height `h` measured relative to a datum; used in pendulum energies. |

## Thrust & Propulsion
| Quantity | Expression | Notes |
| --- | --- | --- |
| Ideal thrust | `T = k_t·ω²` | Simplified rotor law; relates RPM (`ω`) to thrust. |
| Reaction torque | `Q = k_q·ω²` | Couples into yaw dynamics. |
| Work by constant thrust | `W = F·Δx` | Used in the point-mass energy check; equates to kinetic energy gain. |

## Elastic & Damping Forces
| Type | Expression | Notes | Example |
| --- | --- | --- | --- |
| Spring (Hooke’s law) | `F_s = -k·x` | `x` is displacement from rest length. | 1D spring, spring pendulum |
| Vector spring | `F_s = -k·(r - L₀)·r̂` | Points along the tether; used for the pendulum’s elastic force. | Spring pendulum |
| Damping | `F_d = -c·v` | Opposes velocity; removes energy. | Spring pendulum |

## Pendulum-Specific Relations
| Quantity | Expression | Context |
| --- | --- | --- |
| Angle-to-position | `x = l·sinθ`, `y = -l·cosθ` | Converts pendulum angle to Cartesian coordinates. |
| Kinetic energy (single link) | `T = ½·m·l²·ω²` | Angular kinetic energy. |
| Double-link kinetic energy | `T = ½·m₁·l₁²·ω₁² + ½·m₂·(v₁² + v₂² + 2·l₁·l₂·ω₁·ω₂·cos(θ₁-θ₂))` | Captures coupling between links. |

## Energy Equivalences
| Equality | Meaning | Example |
| --- | --- | --- |
| `ΔKE = Work` | Net work equals change in kinetic energy. | Point mass runs constant thrust vs. KE. |
| `E_total = KE + U_s + U_g` | Sum of kinetic, spring, gravitational energies. | Spring pendulum, double pendulum. |
| Damped energy decay | `dE/dt < 0` | Non-negative damping removes energy. | Spring pendulum regression test. |
| Conservative energy | `E_total = constant` | No damping or external work. | Double pendulum adaptive solver benchmark. |

## Mapping to Repository Components
- **Point mass:** uses `F = m·a`, thrust work, and gravitational weight.
- **Spring pendulum:** adds Hooke’s law, damping, and energy decomposition.
- **Double pendulum:** introduces coupled angular equations and shared energy.
- **Rotorcraft roadmap:** will combine thrust (`k_t·ω²`), torque balances,
  and inertia tensors (`τ = I·α`) to model roll/pitch/yaw.

---

## Practical Checklist
1. **Identify forces in the FBD** (weight, thrust, spring, damping). Translate
   each to its algebraic form using the tables above.
2. **Write the Newtonian or Lagrangian equations** (`ΣF = m·a`, energy balance,
   or generalized coordinates). Note how forces contribute to acceleration or
   potential energy.
3. **Check energy/work consistency** to validate the simulation (e.g., work by
   thrust should match kinetic gain for the point mass).
4. **For multirotors**, extend with torque equations: rotor thrusts map to body
   torques using moment arms and the inertia tensor.

By keeping these equivalences handy, you can transition smoothly from simple
point-mass reasoning to multi-axis quadrotor dynamics without losing track of
the underlying physics.
