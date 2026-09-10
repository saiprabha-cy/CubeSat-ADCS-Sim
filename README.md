# CubeSat 6-DOF Attitude Dynamics & Control Simulator

A from-scratch MATLAB/Simulink implementation of a CubeSat ADCS (Attitude
Determination and Control Subsystem): rigid-body attitude dynamics,
quaternion kinematics, B-dot magnetic detumbling, reaction-wheel pointing
control, and disturbance rejection against real gravity-gradient torque —
independently cross-validated in both `.m`/`ode45` and native Simulink
block-diagram form, then extended with auto-generated, independently
verified embedded C via Simulink Coder + Embedded Coder.

Built as a portfolio project targeting embedded/GNC roles at small-satellite
and launch-vehicle startups (e.g. Agnikul, Skyroot, Pixxel, Dhruva Space).

## Why this project

Nearly every real CubeSat mission executes the exact sequence implemented
here — tumbling after deployment, detumbling via magnetorquers, then
slewing to and holding a target attitude with reaction wheels, all while
rejecting environmental disturbance torques. This project builds and
validates each stage from first principles (Euler's equations, quaternion
kinematics, B-dot control theory) rather than wrapping an existing toolbox,
and documents every debugging decision along the way.

## Results summary

| Stage | Script | Result |
|---|---|---|
| Torque-free dynamics validation | `sim/main_torque_free_check.m` | Kinetic energy & angular momentum magnitude conserved to <0.01% drift over 60s |
| B-dot detumble | `sim/main_detumble.m` | 96.1% reduction, 1.077 → 0.042 rad/s, crossed 0.05 rad/s threshold at t=20,270s (~3.6 orbits, 500km/51.6°) |
| Reaction-wheel pointing | `sim/main_pointing.m` | 45° slew about [1,1,1] axis, settled to 0.0000° error at t=20.7s, peak wheel momentum 24.5% of saturation |
| Disturbance rejection (PD) | `sim/main_disturbance_rejection.m` | Real gravity-gradient torque, 0.0009° steady-state offset (matches hand-derived estimate) |
| Disturbance rejection (PID) | `sim/main_disturbance_rejection_pid.m` | 0.0007° stable bounded residual — see note on periodic vs. DC disturbances below |
| Simulink cross-validation | `simulink/adcs_full_loop.slx` | Independent block-diagram implementation of the pointing loop, reproduces the `.m` result |
| Auto-generated embedded C | `codegen/` | Embedded Coder-generated C, run standalone (no MATLAB/Simulink) via a hand-written test harness — matches Simulink output to 6 significant figures |

## Key engineering findings (not just "it worked")

**1. Reaction wheel sign-convention bug.** The control law's output
convention (desired *body* torque) and the actuator model's input
convention (motor torque applied to the *wheel*) were mismatched — the
actuator was silently inverting every command. Diagnosed from the symptom
pattern (instant wheel saturation + growing, not shrinking, error) rather
than by retuning gains blindly. See `src/actuators/reaction_wheel_model.m`
docstring for the full writeup.

**2. Gravity-gradient disturbance is periodic, not constant.** Initial
assumption was that a persistent disturbance torque would leave a
constant steady-state error under P+D control, fully nullable by adding
an integral term. Testing revealed the opposite Ki-dependence than that
model predicts (*more* integral gain gave a *smaller* residual, less gave
a larger one) — the correct explanation is that gravity-gradient torque is
periodic with orbital motion (since the target attitude is fixed in the
inertial frame while the nadir direction sweeps through the body frame
each orbit), so a standard PID can attenuate but not fully null it. See
`docs/03_results_analysis.md` for the full diagnostic trail.

**3. Simulink port-order bug.** A multi-input MATLAB Function block
(`Rigid Body Dynamics`, 3 inputs) had its `h_wheel` (integrated momentum
state) and `h_wheel_dot` (raw feedthrough rate) inputs crossed — visually
subtle since both signals originate from the same crowded region of the
diagram, but produced a persistent limit-cycle failure mode nearly
identical to finding #1. Fixed by rewiring each port individually and
verifying source-by-source rather than re-inspecting the whole diagram at once.

**4. Root Inport zero-default + continuous-time incompatibility (code
generation stage).** Preparing the model for embedded C generation
surfaced two further issues: a Root Inport with nothing externally
driving it silently defaults to zero, producing a plausible-looking but
meaningless constant output; and continuous-time Integrator blocks are
incompatible with standard embedded code generation, requiring a
switch to discrete-time integration — verified numerically (not
assumed) to introduce no meaningful drift before trusting the result.
See `codegen/model_config/codegen_config_notes.md` for the full trail.

## Folder structure

```
cubesat-adcs-sim/
├── docs/                    theory notes, design decisions, results analysis
├── src/
│   ├── dynamics/            Euler's equations, quaternion kinematics, inertia model
│   ├── actuators/           magnetorquer + reaction wheel hardware models
│   ├── environment/         orbit propagator, magnetic field, gravity-gradient
│   ├── control/             B-dot, PD, and full PID attitude control laws
│   └── utils/               quaternion operations, plotting helpers
├── simulink/                block-diagram cross-validation + build guide
├── codegen/                 auto-generated embedded C, standalone test harness, verification
├── sim/                     run scripts (one per validation stage)
├── results/figures/         generated plots
└── tests/                   quaternion unit tests
```

Each `_with_wheels` / `_full` / `_pid` file variant is kept deliberately
separate from its predecessor rather than modifying it in place — earlier
versions stay validated and untouched while later stages build on top.

## Running this project

Requires MATLAB (Simulink for `simulink/adcs_full_loop.slx`). No toolboxes
beyond base MATLAB are required for the `.m` scripts.

```matlab
cd sim
unit_tests_quaternion              % run tests/ first — quaternion math sanity checks
main_torque_free_check             % validates dynamics implementation
main_detumble                      % B-dot detumbling
main_pointing                      % reaction-wheel point-and-hold
main_disturbance_rejection         % gravity-gradient, pure PD
main_disturbance_rejection_pid     % gravity-gradient, full PID
```

Each script prints a numeric summary to console and saves a plot to
`results/figures/`.

For the Simulink cross-validation and auto-generated C, see
`simulink/SIMULINK_BUILD_GUIDE.md` and `codegen/README.md` respectively —
both are self-contained build/reproduction guides.

## Simplifications (documented, not hidden)

- Circular orbit only, no J2 perturbation, no eccentricity (`orbit_propagator.m`)
- Box-model inertia tensor, uniform mass distribution assumed (`inertia_properties.m`)
- Centered tilted-dipole Earth field model, not full IGRF (`magnetic_field_model.m`)
- Gravity-gradient is the only disturbance torque implemented; aerodynamic
  and solar radiation pressure torques are documented as future extensions
- Reaction wheels modeled as 3 independent orthogonal axes, not a 4-wheel
  pyramid configuration

None of these were chosen to make results look better — each is the
standard first-pass simplification in spacecraft dynamics literature, and
each is called out explicitly in its source file's docstring rather than
silently assumed.

## Author

SAIPRABHA C Y 
Related work: [embedded-systems](https://github.com/saiprabha-cy/embedded-systems),
[communication-systems](https://github.com/saiprabha-cy/communication-systems)