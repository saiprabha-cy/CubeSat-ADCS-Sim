# Generated Code Walkthrough

What Embedded Coder actually produced, and how it's structured — for
anyone (including future-you) reading the generated `.c`/`.h` files
without having watched the generation happen.

---

## Files generated

| File | Contents |
|---|---|
| `adcs_full_loop_codegen.h` | Public interface: I/O structs, entry-point function declarations |
| `adcs_full_loop_codegen.c` | The actual control-loop logic — every block's math, in one file |
| `adcs_full_loop_codegen_types.h` | Custom type definitions used by the model |
| `adcs_full_loop_codegen_private.h` | Internal declarations not part of the public interface |
| `rtwtypes.h` | Standard Simulink Coder type definitions (`real_T`, etc. — portable typedefs, not raw `double`, so the same code can retarget to different word sizes) |
| `ert_main.c` | Embedded Coder's own default test driver — **not used**; see `harness/main_test_harness.c` instead |

## The three-function interface

Every Simulink Coder model, regardless of complexity, boils down to
exactly three callable functions:

```c
extern void adcs_full_loop_codegen_initialize(void);  // call once, at startup
extern void adcs_full_loop_codegen_step(void);          // call once per control-loop tick
extern void adcs_full_loop_codegen_terminate(void);      // call once, at shutdown
```

`_initialize()` sets every block's initial state — this is where each
Discrete-Time Integrator's Initial Condition value actually gets written
into memory, equivalent to a struct-zeroing/setup routine you'd hand-write
in bare-metal C.

`_step()` is the entire control loop, one tick's worth: read input →
quaternion error → PD control → reaction wheel saturation → rigid body
dynamics → kinematics → normalize → write output. On real hardware, this
function would be called from a timer interrupt service routine firing
at the configured rate (100 Hz here) — the harness's `for` loop calling it
80,000 times is a *simulation* of that ISR pattern, not how it would
actually run deployed.

## The I/O structs

```c
typedef struct {
  real_T q_target_in[4];
} ExtU_adcs_full_loop_codegen_T;   /* External inputs */

typedef struct {
  real_T err_deg_out;
  real_T omega_out[3];
  real_T h_wheel_out[3];
} ExtY_adcs_full_loop_codegen_T;   /* External outputs */

extern ExtU_adcs_full_loop_codegen_T adcs_full_loop_codegen_U;  /* the actual instance */
extern ExtY_adcs_full_loop_codegen_T adcs_full_loop_codegen_Y;
```

`U` (inputs) and `Y` (outputs) are global struct instances — write into
`U` before calling `_step()`, read from `Y` after. This is exactly the
memory-mapped-register-style interface pattern from your STM32 bare-metal
work: a fixed struct, no dynamic allocation, no hidden state beyond what's
declared.

## The state struct

```c
typedef struct {
  real_T QIntegrator_DSTATE[4];
  real_T OmegaIntegrator_DSTATE[3];
  real_T H_wheelIntegrator_DSTATE[3];
} DW_adcs_full_loop_codegen_T;
```

`DW` = "DWork" (Simulink's term for block-owned persistent state). This
is exactly the three Discrete-Time Integrator states — `q`, `ω`, and
`h_wheel` — nothing more, nothing hidden. Every other block in the model
(PD Control, Reaction Wheel, Rigid Body Dynamics, Kinematics, Quaternion
Error/Normalize) is purely combinational — computed fresh each `_step()`
call from current inputs and these three states, with no memory of its
own. This is a direct, readable mapping back to the block diagram: three
integrators, three state variables, matching exactly.

## What got optimized away

The generated header notes:
```
Block '<Root>/Display' : Unused code path elimination
Block '<Root>/Error Scope' : Unused code path elimination
...
```
Embedded Coder correctly recognized that Scope and Display blocks have no
effect on the computed output values — they're purely for interactive
visualization inside Simulink — and eliminated all code for them. This is
worth understanding rather than being surprised by: it confirms the
generated code contains *only* the actual control-loop computation, with
zero debug/visualization overhead, which is exactly what you want in
deployed embedded code.

## Traceability

The header includes a system hierarchy comment mapping each subsystem
(`<S1>` through `<S7>`) back to its original block name — e.g. `<S3>` =
`PD Control`. Combined with inline comments throughout `adcs_full_loop_codegen.c`
like `/* '<Root>/PD Control' */`, every line of generated C can be traced
directly back to the block that produced it. This is the actual mechanism
that makes "verify the model, then trust the generated code" a legitimate
engineering practice rather than blind faith in a code generator — every
line has a known origin.