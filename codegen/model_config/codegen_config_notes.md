# Code Generation Configuration Notes

Every setting changed from the base `adcs_full_loop.slx` model, in the
order applied, with the reasoning behind each — so this is reproducible
from scratch, not just a list of clicks.

---

## 1. Model I/O: Constant/Scope → Root Inport/Outport

**Changed:** removed the `q_target` Constant block, replaced with a Root
Inport (`q_target_in`, port dimension `4`). Added three Root Outports
(`err_deg_out`, `omega_out`, `h_wheel_out`) tapped from the same signals
already feeding the existing Scopes (Scopes left in place — harmless for
codegen, useful for interactive sanity-checking).

**Why:** Simulink Coder only creates genuine external I/O in generated C
for Root Inport/Outport blocks. A Constant block gets baked into the
generated code as a fixed literal value with no way for external C code
to drive different scenarios; a Scope produces no usable output data
structure at all. This is a real architectural requirement for code
generation, not a style preference.

## 2. Solver: Variable-step `ode45` → Fixed-step `ode4`

**Changed:** `Ctrl+E` → Solver → Type: `Fixed-step`, Solver: `ode4
(Runge-Kutta)`, Fixed-step size: `0.01` (100 Hz).

**Why:** generated embedded C runs a `model_step()` function once per
fixed hardware-timer tick — there's no way to "adaptively" choose a step
size on deployed hardware the way a variable-step solver does on a
desktop. 100 Hz was chosen as a realistic starting point for a CubeSat
attitude-control loop rate; verified (not assumed) to be adequate by
comparing the final state against the original variable-step run before
proceeding — see §5.

## 3. Integrator blocks: Continuous → Discrete-Time Integrator

**Changed:** all three `1/s` Continuous Integrator blocks (`Omega
Integrator`, `Q Integrator`, `H_wheel Integrator`) replaced with
**Discrete-Time Integrator** blocks, Sample time `0.01` (matching the
fixed-step size exactly), same Initial Conditions as before
(`[0.02;-0.015;0.025]`, `[1;0;0;0]`, `[0;0;0]` respectively), integration
method left at default (Forward Euler).

**Why:** attempting to generate code with continuous Integrator blocks
under this configuration fails outright:
```
Error using tlc_c
Block 'adcs_full_loop_codegen/H_wheel Integrator' uses continuous time,
which is not supported with the current configuration.
```
Embedded Coder can technically embed a full ODE solver into generated C
(there's a "Support continuous time" option), but that's not what a real
embedded target does — actual flight/ECU control loops use discrete-time
state updates computed once per tick, not a runtime numerical solver.
Swapping to Discrete-Time Integrator blocks is the textbook-correct
deployment choice, not a workaround forced by an error message.

## 4. Root Inport default value: Data Import/Export workaround

**Problem found:** a Root Inport with nothing externally driving it
during interactive simulation defaults to an all-zero signal. For
`q_target_in = [0,0,0,0]`, the quaternion error math produces `qe0 = 0`
regardless of the actual attitude, so `err_deg = 2·acosd(0) = 180°` — a
constant, plausible-looking but entirely meaningless value. This silently
broke the model without an obvious error.

**Fix:** `Ctrl+E` → Data Import/Export → Input field set to a MATLAB
workspace variable `q_target_in`, itself set via:
```matlab
q_target_in = [0    0.9239 0.2209 0.2209 0.2209;
               800  0.9239 0.2209 0.2209 0.2209];
```
(time-matrix format: first column = time, remaining columns = port
value — two identical rows at t=0 and t=800 hold the value constant for
the whole run.) This is purely an interactive-testing convenience — for
actual code generation, the harness drives this port directly in C
instead (see §6).

## 5. Verification checkpoint: fixed-step + discrete integrators vs. baseline

Before generating any code, both changes above (§2, §3) were verified
together against the last confirmed variable-step/continuous baseline
(`err_deg=0.491°`, matching across two independent prior runs):

| Config | `err_deg_out` | `omega_out` | `h_wheel_out` |
|---|---|---|---|
| Variable-step `ode45`, continuous Integrators | 0.491 | `~1e-13` | `[3.045e-4, -6.812e-4, 7.528e-4]` |
| Fixed-step `ode4` + discrete Integrators | 0.491 | `~1e-15` | `[3.046e-4, -6.814e-4, 7.529e-4]` |

Match to 4+ significant figures — Forward Euler at 100 Hz introduces no
practically meaningful drift for this system. This is a real verified
result, not an assumption carried forward from the ADCS project.

## 6. Code generation target settings

`Ctrl+E` → Code Generation:
- **System target file:** `ert.tlc`
- **Language:** `C`
- **Create code generation report:** checked
- **Support: floating-point numbers:** checked (default; required, model uses double precision throughout)

Everything else left at default — no speculative settings changed beyond
what was needed to get a correct build.

## 7. Build toolchain

- **Compiler:** MinGW-w64, installed via MATLAB Add-On Explorer
  ("MATLAB Support for MinGW-w64 C/C++ Compiler"), confirmed via
  `mex -setup C`.
- **Build command:** `rtwbuild('adcs_full_loop_codegen')`
- **Output:** `adcs_full_loop_codegen_ert_rtw/`, containing
  `adcs_full_loop_codegen.c/.h`, `adcs_full_loop_codegen_types.h`,
  `rtwtypes.h`, and Embedded Coder's own `ert_main.c` (not used — see
  `harness/main_test_harness.c` for the actual standalone driver).