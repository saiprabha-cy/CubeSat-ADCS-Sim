# Project Report: CubeSat 6-DOF Attitude Dynamics & Control Simulator

**Author:** Saiprabha C Y · **Tools:** MATLAB, Simulink

---

## 1. Objective

Design, implement, and validate — from first principles, not from an
existing toolbox — the core attitude control sequence every CubeSat
executes after deployment: detumbling, point-and-hold, and disturbance
rejection. Cross-validate the result in two independent implementations
(MATLAB/`ode45` and native Simulink block diagrams) to demonstrate both
scripting and Model-Based Design competency.

## 2. Scope

| In scope | Out of scope (documented, not hidden) |
|---|---|
| Rigid-body attitude dynamics (Euler's equations) | Full IGRF magnetic field model |
| Quaternion kinematics, double-cover-safe control | J2/eccentric orbit perturbations |
| B-dot magnetic detumbling | Aerodynamic / solar-radiation-pressure disturbance torques |
| Reaction-wheel PD/PID pointing control, with saturation | Magnetic desaturation maneuvers |
| Gravity-gradient disturbance rejection | 4-wheel pyramid actuator configuration |
| Simulink cross-validation of the pointing loop | Full mission (detumble+point+disturbance) rebuilt in Simulink |

## 3. Methodology

Each stage was built and *independently validated* before the next stage
was added, with earlier validated files never modified — later stages
built as new, parallel files instead (see `docs/02_design_decisions.md`).
Every gain-tuning decision changed exactly one variable per test run, to
avoid misattributing an observed effect to the wrong cause (documented
with specific examples in `docs/02_design_decisions.md`).

## 4. Results

### 4.1 Dynamics validation
Torque-free rigid-body motion conserved kinetic energy and angular
momentum magnitude to <0.01% drift, and reproduced the analytically
predicted symmetric-top precession behavior for this spacecraft's inertia
properties — an independent physical check, not just numerical stability.

### 4.2 Detumble
B-dot magnetic detumbling reduced an aggressive 62°/s initial tumble by
**96.1%**, crossing the 0.05 rad/s "ready for fine pointing" threshold
after ~3.6 orbits. Empirically confirmed that detumble performance is
driven primarily by elapsed orbital time (field-direction diversity), not
elapsed rotation time — a real, documented property of B-dot control.

### 4.3 Pointing control
Reaction-wheel PD control executed a 45° slew to **exactly 0.0000°**
final error, with peak wheel momentum at 24.5% of saturation capacity.
This result was reached only after diagnosing and fixing a genuine
sign-convention bug between the control law's and actuator model's
torque conventions — see §5.

### 4.4 Disturbance rejection
Real gravity-gradient torque (not a synthetic placeholder) was derived
and implemented, producing a steady-state pointing error under pure PD
control that matched an independent hand calculation to within an order
of magnitude — confirming the simulated response is physically grounded.
Testing the full PID controller against this same disturbance revealed
that the disturbance is **periodic** (tied to orbital motion) rather than
constant, correcting an initial modeling assumption and explaining
otherwise-counterintuitive gain-tuning results (§5).

### 4.5 Simulink cross-validation
An independently-built Simulink block-diagram model of the pointing loop
qualitatively reproduced the MATLAB result (convergence from 45° to a
small stable error, wheel momentum well under saturation), with a
final-precision discrepancy (0.491° vs 0.0000°) that remains open — the
initial solver-tolerance hypothesis was directly tested and ruled out
(both platforms confirmed at `RelTol=1e-8`), and the true cause is
documented as an honest unresolved item rather than a false resolution —
see `docs/03_results_analysis.md` §6.

### 4.6 Auto-generated embedded C
The validated pointing-loop model was reconfigured for Embedded Coder
(fixed-step discrete solver, Root Inport/Outport interface) and compiled
to standalone C via Simulink Coder. A hand-written C test harness — no
MATLAB or Simulink involved — drove the generated code through the
identical 45° slew scenario, and the output matched the Simulink baseline
to **6 significant figures** (`err_deg`: 0.491000 vs. 0.491010, max
absolute difference across all signals `< 5×10⁻⁵`). Two further real
issues were found and fixed along the way: a Root Inport left undriven
during interactive testing silently defaults to zero rather than erroring,
and continuous-time integration is incompatible with standard embedded
codegen, requiring (and numerically verifying) a switch to discrete-time
integration. Full detail in `codegen/`.

## 5. Notable engineering findings

This project's debugging record is treated as a first-class deliverable,
not an afterthought:

1. **Reaction wheel sign-convention bug** — the actuator model was
   silently inverting every control command due to a torque-convention
   mismatch between the control law and actuator interfaces. Diagnosed
   from the specific symptom pattern (instant saturation + growing error)
   rather than by blind retuning.
2. **Periodic vs. constant disturbance modeling error** — an initial
   assumption that gravity-gradient torque is roughly constant was
   falsified by gain-tuning data showing the opposite of the predicted
   Ki-dependence, leading to a corrected and more accurate understanding
   of the disturbance's physical nature.
3. **Simulink multi-input port-order bug** — an independently-arising,
   platform-specific instance of essentially the same bug *class* as
   finding #1, diagnosed using the same reasoning after ruling out every
   block's internal code individually.
4. **Code-generation-stage issues** — a Root Inport left undriven
   silently defaults to zero rather than erroring (producing a
   plausible-looking but meaningless constant output), and continuous-time
   integration is fundamentally incompatible with standard embedded code
   generation. Both required real engineering decisions, not just settings
   toggles, and both were numerically verified rather than assumed correct.

Full diagnostic trails for all three are in `docs/02_design_decisions.md`
and `docs/03_results_analysis.md`.

## 6. Conclusion

All core ADCS objectives — detumble, point-and-hold, disturbance rejection
— are validated end-to-end in MATLAB, with a working (and honestly
assessed) Simulink cross-validation. The project demonstrates not just
working code, but a documented engineering process: hypothesis-driven
debugging, single-variable experimental isolation during tuning, and
explicit documentation of every simplifying assumption and its
justification.

## 7. Future extensions

- Add aerodynamic and solar-radiation-pressure disturbance torques.
- Implement magnetic desaturation for the reaction wheels.
- Extend to a 4-wheel pyramid actuator configuration.
- Resolve and re-verify the Simulink solver-tolerance discrepancy (§4.5).
- Extend the Simulink model to the full detumble→point→disturbance-reject sequence.

## Repository

Full source, all validation scripts, and detailed documentation:
`docs/01_theory_notes.md`, `docs/02_design_decisions.md`,
`docs/03_results_analysis.md`, `README.md`.