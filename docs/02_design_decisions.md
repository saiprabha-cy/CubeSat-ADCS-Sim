# Design Decisions

Rationale behind architectural choices, simplifications, and — most
importantly — the debugging methodology used throughout, since the
decisions made *while things were broken* are as much a part of this
project's engineering record as the final working state.

---

## Architecture

**Quaternions over Euler angles.** No gimbal lock, simpler composition,
matches real flight-software convention. Trade-off: less physically
intuitive to eyeball, so `quat_to_euler` exists purely as a debug/plotting
convenience — never fed back into any control law.

**Static-method classes for multi-function utility files
(`quaternion_ops.m`, `plotting_helpers.m`).** MATLAB only allows a file's
*first* function to be called externally — any additional functions in a
plain `.m` file are invisible outside it. A `classdef` with `Static`
methods keeps a toolkit in one logically grouped file while every method
stays fully callable (`quaternion_ops.quat_rotate(q, v)`), and is a minor,
free signal of MATLAB OOP fluency.

**Wheel-free and wheel-augmented dynamics kept as separate files**
(`rigid_body_dynamics.m` vs. `rigid_body_dynamics_with_wheels.m`, same
pattern for the state-derivative and control-law files). Once a version is
validated against an independent physical check (energy/momentum
conservation for the dynamics; the detumble result for the wheel-free
control-loop chain), it is never modified again — later-stage features are
built as new, parallel files rather than edits, so nothing that already
passed validation can silently break.

**Control law and actuator saturation kept as separate pipeline stages**
(e.g. `pid_attitude_control.m` computes a *desired* torque with no
knowledge of hardware limits; `reaction_wheel_model.m` applies saturation
with no knowledge of what control law produced the command). Mirrors real
GNC software structure and made the actuator sign-bug (below) isolatable
to a single file once found.

## Documented simplifications (not hidden assumptions)

| Simplification | Where | Why acceptable for this project's scope |
|---|---|---|
| Circular orbit, no J2, no eccentricity | `orbit_propagator.m` | Adequate for minutes-to-hours-scale attitude control; would matter for multi-day mission planning |
| Box-model uniform-mass inertia tensor | `inertia_properties.m` | Correct starting point; real CubeSats have concentrated masses (battery, PCBs) needing parallel-axis corrections |
| Centered tilted-dipole Earth field, not full IGRF | `magnetic_field_model.m` | Standard first-pass model in CubeSat ADCS literature |
| Gravity-gradient is the only disturbance implemented | `disturbance_torques.m` | Dominant disturbance for a compact LEO CubeSat; aero/SRP noted as future extensions, not silently ignored |
| 3 independent orthogonal wheels, not a 4-wheel pyramid | `reaction_wheel_model.m` | Simpler actuator model; a pyramid config would need a wheel-to-body mapping matrix |
| No magnetic desaturation maneuver | `reaction_wheel_model.m` | Wheel momentum stayed well under saturation in all tested scenarios; documented as the next real feature to add |

None of these were chosen post-hoc to make results look better — each is
stated in its source file's docstring before any result was generated.

## Gain-tuning methodology: isolate one variable at a time

Every tuning decision in this project changed exactly one parameter per
run, holding everything else fixed, specifically to avoid the trap of
attributing an improvement to the wrong cause:

- **B-dot detumble:** initial run (`k=5e4`, `t=1200s`) showed slow, weak
  convergence. Rather than changing gain and duration together, duration
  was extended first with gain fixed (`k=8e3`, `t=6500s → 17000s → 21000s`),
  which revealed the true driver: B-dot needs orbital field-direction
  diversity, not just more rotation time — reduction went 30% → 64% → 92% →
  96% purely from duration, confirming the hypothesis before declaring success.
- **PID disturbance rejection:** `Ki` was tested at `2e-5` (2000s), then
  `8000s` at the same value, then `5e-6` at `8000s`, then reverted to
  `2e-5`. This directly falsified an initial "Ki too aggressive → limit
  cycle" hypothesis (lowering `Ki` made the residual *worse*, the opposite
  prediction), leading to the correct periodic-disturbance explanation
  instead of a wrong conclusion reached by changing too much at once.

## Debugged findings (the actual engineering content)

### 1. Reaction wheel sign-convention bug (`.m` implementation)

**Symptom:** `main_pointing.m`'s first run showed wheels saturating
instantly (173% of nominal capacity) while attitude error *grew* (45° →
169°) instead of shrinking.

**Root cause:** `pid_attitude_control.m` returns the *desired torque on
the body*. `reaction_wheel_model.m` was treating that same value as the
*motor torque to apply directly to the wheel*, then computing
`tau_body = -tau_actual ≈ -tau_cmd` — the exact negative of what the
controller wanted. By Newton's third law (`τ_body = -τ_wheel`), achieving
a desired body torque requires commanding the wheel to the *negative* of
that value — the negation was missing.

**Diagnosis method:** the specific combination of symptoms (instant
saturation + growing, not shrinking, error) is the signature of inverted
feedback, distinguishable from a simple mistuned-gain problem, which would
show slow/no convergence but not active divergence.

**Fix:** one line (`tau_wheel_cmd = -tau_cmd_body`) added before
saturation, fully localized to `reaction_wheel_model.m` — no changes
needed anywhere else in the pipeline, confirming the modular
control/actuator separation paid off.

### 2. Periodic vs. constant disturbance (control theory finding)

**Initial assumption:** gravity-gradient torque is roughly constant, so a
PID integral term should drive steady-state error to true zero given
enough time.

**Contradicting evidence:** two long-duration runs (`Ki=2e-5` →
`0.0007°` residual; `Ki=5e-6` → `0.0018°` residual) showed the *opposite*
Ki-dependence a "Ki too aggressive → limit cycle" explanation would
predict — lower gain made the oscillation worse, not better.

**Correct explanation:** `q_target` is fixed in the inertial frame, so as
the satellite orbits, the nadir direction sweeps through the body frame
once per orbit — gravity-gradient torque is therefore periodic, not DC.
A PID integral term nulls a constant disturbance exactly but only
*attenuates* a periodic one; higher gain gives better attenuation up to
stability limits, which is exactly the observed direction.

**Resolution:** `Ki=2e-5` retained as the better-performing (not
"correct" in an absolute sense — genuinely better) gain; the residual
bounded oscillation is documented as expected behavior for a standard PID
against a periodic disturbance, not a tuning failure.

### 3. Simulink MATLAB Function block port-order bug

**Symptom:** the independently-built Simulink pointing loop reproduced
the *same* saturation-with-growing-error failure mode as bug #1, despite
every individual block's code being independently verified correct.

**Root cause:** `Rigid Body Dynamics`'s three input ports (`omega`,
`h_wheel`, `h_wheel_dot`) are ordered by argument position, not by wire
label — `h_wheel` (the integrated momentum *state*) and `h_wheel_dot`
(the raw, un-integrated feedthrough *rate*) were crossed. Both signals
originate from the same crowded region of the diagram, making this the
single easiest wire-crossing mistake in the whole model.

**Diagnosis method:** once every block's *code* was confirmed correct
(ruling out logic errors), the remaining failure mode had to be pure
wiring — the fix was to delete and deliberately rebuild each of the three
input wires individually, verifying each source before connecting,
rather than re-inspecting a visually busy diagram repeatedly.

**Significance:** this bug class (multi-input port ordering in
Simulink MATLAB Function blocks) is distinct from — but produces
symptoms nearly identical to — the `.m`-side sign-convention bug,
demonstrating the same underlying discipline (control law / actuator /
dynamics separation) transfers across implementation platforms even when
the specific failure mechanism differs.