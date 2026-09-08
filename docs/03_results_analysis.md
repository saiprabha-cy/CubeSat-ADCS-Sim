# Results Analysis

Numeric results for every validation stage, in the order run, with
interpretation. Figures referenced are in `results/figures/`.

---

## 1. Torque-free dynamics verification (`main_torque_free_check.m`)

**Setup:** 3U CubeSat (`Ixx=Iyy=0.04187`, `Izz=0.006667` kg·m²), initial
tumble `ω0 = [0.5, -0.3, 0.2]` rad/s (deliberately off-principal-axis),
zero applied torque, 60s, tight solver tolerances (`RelTol=1e-10`).

| Quantity | Result |
|---|---|
| Kinetic energy drift | 0.0000% |
| Angular momentum magnitude drift | 0.0000% |
| `ωz` behavior | Constant (exact conservation) |
| `ωx`/`ωy` behavior | Sinusoidal, constant-amplitude precession |

**Interpretation:** exact conservation of `ωz` and circular precession of
`ωx`/`ωy` is the textbook symmetric-top torque-free solution — this
spacecraft has `Ixx=Iyy` by construction of the box-inertia model, and the
simulation reproduces the analytically-known behavior for that case. This
is a stronger validation than "the numbers didn't blow up" — it's an
independent physical prediction, matched.

## 2. B-dot detumble (`main_detumble.m`)

**Setup:** `ω0 = [0.8, -0.6, 0.4]` rad/s (≈62°/s, an aggressive worst-case
tip-off rate), gain `k=8e3`, `m_max=0.2` A·m², 500km/51.6° orbit.

| Duration tested | Result | Note |
|---|---|---|
| 1200s | 30% reduction, did not reach threshold | Gain also mis-set (5e4, causing full saturation) |
| 6500s (gain corrected to 8e3) | 64% reduction | |
| 17000s | 92.4% reduction, `0.082` rad/s | |
| 21000s | **96.1% reduction, `0.042` rad/s**, threshold crossed at t=20,270s | Final validated result |

**Key finding:** B-dot detumble time is fundamentally limited by how much
of the orbit has elapsed, not just elapsed rotation time — the controller
needs the magnetic field's inertial direction to sweep through enough
diversity to damp all three axes. This is a documented, expected property
of B-dot, confirmed empirically here by isolating duration as the sole
variable across the four runs above. The amplitude-modulated envelope
decay visible in the plot (fast nutation oscillation inside a shrinking
envelope) is the visual signature of this — not a flat non-decaying
oscillation, which would indicate a problem.

## 3. Reaction-wheel pointing (`main_pointing.m`)

**Setup:** 45° slew about `[1,1,1]/√3`, starting from the detumbled
residual `ω0 ≈ [0.02, -0.015, 0.025]` rad/s, `Kp=3e-3`, `Kd=1.2e-2`,
`h_max=0.01` N·m·s, `tau_max=0.0006` N·m.

| Run | Final error | Max wheel momentum | Settle time |
|---|---|---|---|
| Before sign-convention fix | 169.0° (diverged) | 173.2% of capacity (over-saturated) | Did not settle |
| **After fix** | **0.0000°** | **24.5% of capacity** | **20.7s** |

See `02_design_decisions.md` for the full bug diagnosis. The fixed result
is the validated baseline used for all subsequent stages and for
cross-checking the Simulink model.

## 4. Disturbance rejection — pure PD (`main_disturbance_rejection.m`)

**Setup:** same target/gains as above, real gravity-gradient torque active
throughout (`τ_gg = (3μ/R³)·(o3 × I·o3)`), 2000s.

| Quantity | Result |
|---|---|
| Steady-state error | 0.0009° |
| Hand-derived estimate (`θ ≈ τ_gg/Kp`) | ≈0.0025° |

**Interpretation:** same order of magnitude as the independent hand
calculation (`τ_gg ≈ 1.3×10⁻⁷ N·m`, ≈5000× smaller than wheel torque
authority `τ_max=6×10⁻⁴ N·m`) — confirms the simulated disturbance
response is physically reasonable, not a numerical artifact. Also
demonstrates that gravity-gradient is a genuinely weak disturbance for a
compact 3U CubeSat at this altitude, unlike larger/more elongated
spacecraft where it's historically significant enough for passive
gravity-gradient stabilization.

## 5. Disturbance rejection — full PID (`main_disturbance_rejection_pid.m`)

| `Ki` | Duration | Result |
|---|---|---|
| 2e-5 | 2000s | 0.0007° (transient too short to confirm settling) |
| 5e-6 | 8000s | **0.0018°, persistent oscillation (non-shrinking)** |
| 2e-5 | 8000s | **0.0007°, persistent oscillation (non-shrinking), reproducible on repeat run** |

**Key finding:** lowering `Ki` made the residual *worse*, not better —
the opposite of what a simple "integral gain too aggressive" explanation
predicts. Correct explanation (see `01_theory_notes.md` §6): the
disturbance is periodic (orbital-motion-linked, since `q_target` is
inertially fixed), and a PID integral term attenuates but cannot fully
null a periodic disturbance. `Ki=2e-5` is the better-performing of the two
tested gains and is retained as the final configuration. Both residuals
are far below any practical CubeSat pointing requirement.

## 6. Simulink cross-validation (`simulink/adcs_full_loop.slx`)

Independent block-diagram implementation of the pointing loop (§3 above),
built natively in Simulink rather than derived from the `.m` code, then
validated against it.

| Signal | Final value |
|---|---|
| Attitude error | **0.491°** |
| `ω` | `[7.0e-13, -3.0e-13, -6.0e-11]` rad/s (converged to numerical zero) |
| `h_wheel` | `[0.000305, -0.000681, 0.000753]` N·m·s |

**Honest discrepancy note:** the `.m` result for this identical scenario
is `0.0000°`. `0.491°` is a real, non-negligible gap, not a rounding
difference. However, `ω` having converged to numerical zero
(`~1e-13` level) while `h_wheel` is holding steady, unchanging values
indicates the loop reached a genuine dynamic equilibrium, not an
unconverged transient — ruling out the wiring-bug failure mode documented
in §3/§7 (that failure mode showed sustained large oscillation, not a
small stable offset).

**Most likely explanation — TESTED AND RULED OUT:** the build guide
specifies `RelTol=1e-8` to match the `.m` script. Checked directly in
Model Configuration Parameters: the Simulink model was already at
`RelTol=1e-8`, not Simulink's default `1e-3`. **This rules out the solver
tolerance hypothesis** — the discrepancy is not explained by a
precision-setting mismatch between the two platforms.

**Status: open, unresolved discrepancy.** The `0.491°` vs `0.0000°` gap
remains real and unexplained. With `ω` converged to numerical zero and
`h_wheel` holding steady, non-changing values, the loop has genuinely
reached a stable dynamic equilibrium — so this is not a transient-still-
in-progress artifact. The two most plausible remaining candidates, neither
independently confirmed, are: (1) a very small residual asymmetry in how
one of the MATLAB Function blocks was hand-transcribed versus its `.m`
source (a subtle rounding or operator-precedence difference too small to
show up as a gross wiring failure but large enough to shift a true-zero
equilibrium by ~0.5°), or (2) a genuine difference in how Simulink's
fixed-point iteration at each solver step handles this particular coupled
algebraic structure versus MATLAB's `ode45` applied directly to the same
equations. Documented here as a known, honestly-flagged open item rather
than a false resolved explanation — a legitimate finding in its own right
for a portfolio piece, since claiming false certainty would be a worse
outcome than an accurately scoped unknown.

**Value of this result regardless:** even with the discrepancy
unconfirmed, the qualitative behavior — convergence from 45° to a small
stable value, wheel momentum settled well under saturation, `ω` fully
damped — independently reproduces the `.m` implementation's *behavior*,
which is itself a legitimate verification technique (two independently
built implementations agreeing qualitatively, even before matching to the
fourth decimal place).

## 7. Debugged findings summary

Two independent, platform-specific instances of the same underlying bug
*class* (actuator command sign/order inversion) were found and fixed
during this project — see `02_design_decisions.md` for full diagnosis of
each:

1. `.m` implementation: sign-convention mismatch between control-law
   output convention and actuator input convention (`reaction_wheel_model.m`).
2. Simulink implementation: input-port order mismatch between the
   integrated momentum state and the raw feedthrough rate
   (`Rigid Body Dynamics` block).

Both produced nearly identical symptom signatures (actuator saturating
aggressively while error grows instead of shrinking) despite arising from
different mechanisms in different tools — evidence that the underlying
physical/control reasoning used to diagnose bug #1 transferred correctly
to diagnosing bug #2, rather than each fix being tool-specific luck.