# Theory Notes

Derivations underlying every stage of this project, in the order they were
implemented: rigid-body dynamics → quaternion kinematics → detumble control
→ reaction wheel actuation → pointing control → gravity-gradient disturbance.

---

## 1. Euler's Rotational Equations of Motion

Angular momentum in the body frame: `L = I·ω`, where `I` is the (body-frame-
constant) inertia tensor and `ω` is angular velocity. Newton's law
`τ = dL/dt` holds in the inertial frame, so the transport theorem is needed
to differentiate `L` while working in the rotating body frame:

```
(dL/dt)_inertial = (dL/dt)_body + ω × L
```

Substituting `L = I·ω` and rearranging:

```
ω̇ = I⁻¹ · [τ − ω × (I·ω)]
```

The `ω × (I·ω)` term is gyroscopic coupling — rotation about one axis
induces torque-like effects on the other two purely from the body's own
spin. This is what makes attitude dynamics a genuinely coupled nonlinear
system, not three independent single-axis problems.

For a symmetric-top CubeSat (`Ixx = Iyy ≠ Izz`, true for this project's box
inertia model since `a = b`), this predicts exact conservation of the
spin-axis component `ωz` under torque-free motion, with `ωx`/`ωy` executing
circular precession — confirmed by `main_torque_free_check.m`.

## 2. Quaternion Kinematics

Quaternions avoid the gimbal-lock singularity and numerical composition
issues of Euler angles, which is why every real flight software stack uses
them. Representation: `q = [q0, q1, q2, q3]`, unit norm constraint
`q0²+q1²+q2²+q3² = 1`.

Propagation equation:
```
q̇ = ½ · Ω(ω) · q
```
with `Ω(ω)` the 4×4 skew-symmetric matrix built from `ω`. Numerical
integration drifts `q` off the unit sphere over time — renormalization
after every integration step is required (`quaternion_ops.quat_normalize`).

Attitude error between current `q` and target `q_target`:
```
q_err = q_target⁻¹ ⊗ q
```
The vector part of `q_err` is proportional to small-angle attitude error
and is fed directly into the control law — no Euler-angle conversion needed.

**Double-cover correction:** `q` and `-q` represent the same physical
attitude. Without correcting for this, a controller can occasionally
command the "long way around" a rotation. Fix: multiply the error vector
by `sign(qe0)` before using it in any control law.

## 3. B-dot Detumbling Law

A magnetorquer's dipole moment `m` interacting with the local field `B`
produces torque `τ = m × B`. This torque can only act perpendicular to
`B` — a magnetorquer alone can never achieve full 3-axis control, only
detumble/partial control.

For a body with angular velocity `ω`, the field measured in body frame
changes at rate (dominant term, for a field quasi-static in the inertial
frame):
```
Ḃ_body ≈ -ω × B_body
```
Control law:
```
m = -k · Ḃ_body
```
No absolute attitude knowledge required — only the *change* in the locally
sensed field — which is why B-dot is the standard first-stage controller
immediately after deployment.

**Implementation note:** computed analytically as `-ω × B_body`, not via
finite-differencing across timesteps — `ode45` evaluates the derivative
function at non-sequential internal stage points, so a "previous timestep"
finite difference is not well-defined mid-integration. The analytic form
is exact under the quasi-static-field assumption, not an approximation of
convenience.

## 4. Reaction Wheel Momentum Exchange

By Newton's third law, torque applied to spin up a wheel produces an equal
and opposite reaction torque on the body: `τ_body = -τ_wheel`. To command a
desired body torque `τ_cmd_body`, the wheel motor must be driven to
`-τ_cmd_body` — this sign relationship was the source of the reaction-wheel
bug documented in `02_design_decisions.md` and `03_results_analysis.md`.

Total system angular momentum (body + wheel) obeys:
```
I·ω̇ = τ_ext − ω × (I·ω + h_wheel) − ḣ_wheel
```
derived from `Ḣ_total,body + ω × H_total = τ_ext` via the transport
theorem, where `H_total = I·ω + h_wheel`. Wheel momentum `h_wheel` has a
hard saturation limit — once reached, the wheel can no longer absorb
further momentum in that direction without a magnetic desaturation
maneuver (not implemented in this project; documented as a future extension).

## 5. Quaternion PD/PID Attitude Control

```
τ_cmd = -Kp · sign(qe0) · qe_vec - Ki · e_int - Kd · ω
```
`e_int` accumulates `qe_vec` over time (only in the full-PID variant), with
anti-windup: integration is frozen on any axis already at its clamp limit
and still being pushed further in that direction — the same
saturation-aware pattern used in the wheel's momentum-limit logic.

## 6. Gravity-Gradient Disturbance Torque

A non-spherical mass distribution experiences torque in a gravity field
because different parts of the body sit at slightly different distances/
directions from Earth's center:
```
τ_gg = (3·μ / R³) · (o3 × (I·o3))
```
where `o3` is the nadir-pointing unit vector expressed in body frame and
`R` is orbital radius. For a spherically symmetric body (`I = c·Identity`),
`o3 × (I·o3) = 0` — zero torque, as physically expected; this is a built-in
correctness check on the formula.

This is the dominant disturbance torque for a compact LEO CubeSat — larger
than aerodynamic or solar radiation pressure torque at typical CubeSat
altitudes, which is why it's the one implemented here (see
`03_results_analysis.md` for the magnitude estimate and why it's small
relative to actuator authority for this specific spacecraft).

**Important subtlety discovered during testing:** if the target attitude
is fixed in the *inertial* frame, `τ_gg` is **periodic** with orbital
motion (the nadir direction sweeps through the body frame each orbit), not
constant/DC. A PID integral term nulls a constant disturbance exactly but
can only *attenuate*, not fully eliminate, a periodic one — see
`03_results_analysis.md` for the empirical evidence that led to this
correction.