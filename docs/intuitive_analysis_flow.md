# Intuitive Dynamics Flow

This note captures the “back-and-forth” reasoning loop you can use when working
with mechanical systems. It complements `docs/dynamics_analysis_guide.md`
by describing how to move between forces, torques, state-space parameters,
and qualitative response without getting lost in algebra.

---

## 1. Start with the Free-Body Diagram
1. Identify **forces** and **moments**:
   - Translational: gravity, thrust, spring, damping, friction, etc.
   - Rotational: torques from lever arms (`τ = r × F`) or reaction torques.
2. Map how they act on your system (forces → accelerations via Newton’s laws).

### Example: Cart-Pole (Inverted Pendulum)
- Force at each wheel (cart thrust) → lever arm of pole → torque about pivot.
- Weight at the pole’s center of mass → destabilizing torque.
- Damping at the hinge → energy removal.

By tracing the arrows in the FBD, you already know which terms will appear in
the equations and how they interact.

---

## 2. Convert to State-Space or Energy Form
- **State-space**: store position, velocity, angle, angular rate so everything
  becomes first-order: `ẋ = f(x, u)`.
- **Energy check**: kinetic (`½m‖v‖²`, `½I‖ω‖²`) + potential (spring, gravity)
  verifies whether energy conserves or decays (damping).

This dual view makes it easy to:
- Predict acceleration directions from forces/torques.
- Understand whether energy should stay constant (conservative) or decrease.

---

## 3. Linearize for Small Motions
- Choose equilibrium (`x*`, `u*`); compute `A = ∂f/∂x`, `B = ∂f/∂u`, etc.
- Perturbations `δx`, `δu` around the equilibrium follow `δẋ = A·δx + B·δu`.

**Why this helps intuition:**
- Eigenvalues/eigenvectors of `A` explain natural modes (“this pole tips within
  0.5 s if I do nothing”).
- Controllability tells you which inputs influence which states.
- Observability reveals whether your chosen sensors can “see” the modes.

---

## 4. Jump Between Representations

| Representation | What it tells you | How to use it |
| --- | --- | --- |
| **Force/Torque balance** | Immediate intuition (“if I push here, what happens?”) | Quick sanity checks; FBD-level reasoning. |
| **Energy view** | Conservation or damping, potential wells | Spot oscillations vs. overdamped behavior. |
| **State-space (A,B,C,D)** | Linear dynamics, mode coupling, controller design | Plug into LQR, MPC, observers. |
| **Transfer/Laplace** | Frequency response, step/bode plots | Predict overshoot, bandwidth, phase margin. |

When you change one parameter (e.g., increase damping), trace the impact through
each view:
- Damping coefficient up →
  - Force equation gets larger `-c·v`.
  - Energy decays faster.
  - State-space poles move left (less overshoot).
  - Step response becomes slower, less oscillatory.

This mental loop makes the response intuitive before you simulate it.

---

## 5. Qualitative Response Indicators

| Parameter Change | Expected Effect |
| --- | --- |
| Higher spring constant `k` | Higher natural frequency; faster oscillations; more restoring torque. |
| Higher damping `c` | Less overshoot/ringing; longer rise time if overdamped. |
| Higher mass `m` | Slower acceleration; more torque/force required for same motion. |
| Longer lever arm `r` | Larger torque for the same force; easier to rotate. |
| Higher inertia `I` | Slower angular response; more torque needed. |

Keep these heuristics in mind to anticipate step responses or tuning changes.

---

## 6. Putting It All Together
1. **Draw FBD → Forces/Torques**: determine directions and magnitudes.
2. **Translate to equations**: Newton–Euler or Lagrange.
3. **Check energy**: ensures no missing forces.
4. **Linearize**: get `(A, B, C, D)`; inspect modes.
5. **Predict response**: use qualitative cues (damping, natural frequency).
6. **Validate**: run simulations (time series, frequency response) to confirm.

Repeating this loop builds intuition so that when you see a system (spring,
pendulum, rotorcraft), you can quickly reason about its behavior without
guesswork. The rest of the repository—examples, tests, linear models—feeds back
into the loop by providing references for each step.
