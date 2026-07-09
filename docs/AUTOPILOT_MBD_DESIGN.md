# Autopilot design notes

Design brief for the autopilot in `autopilot.slx`. Reads left-to-right:
what the model does, why the gains are what they are, and how the
generated C ends up flying aircraft inside the CLEARANCE ATC sim.

## What it is

A two-tier cascade autopilot for a transport-category airliner in
level cruise.

- **Inner loops** — three PID controllers, each holding one axis
  against a commanded setpoint:
  - `PID_phi`: `phi_cmd - phi -> delta_a` (bank hold)
  - `PID_theta`: `theta_cmd - theta -> delta_e` (pitch hold)
  - `PID_V`: `V_cmd - V -> delta_t` (airspeed hold)
- **Outer loops** — heading and altitude commands are converted to
  bank and pitch setpoints. These live in the CLEARANCE-side wrapper
  (`AutopilotWrapper.cpp`) rather than the model, because the wrapper
  already has to compute heading errors with wrap-around handling and
  the outer gains are trivial P.

Everything on the Simulink side is authored around this split. The
model has six root inports (`phi_cmd`, `theta_cmd`, `V_cmd`, `phi`,
`theta`, `V`) and three root outports (`delta_a_out`, `delta_e_out`,
`delta_t_out`). Nothing else.

## Plant assumption

The model does not model an aircraft. It models three controllers
against a virtual plant that the *integrator downstream* provides.

Consider the roll axis. The Simulink model outputs `delta_a` (aileron
angle in rad). CLEARANCE takes that value and treats it as a *roll
rate* command:

```
bank_angle += delta_a * K_roll_rate * dt
```

So the plant seen by `PID_phi` is a pure integrator: `phi(s) = k / s`.
Same story on pitch. This is not a real aircraft — a real aircraft
has roll damping and inertia — but it is what CLEARANCE ships, and
the controller is tuned to match.

## Inner-loop gains

The model's PIDs are tuned for a pure-integrator plant. That drives
three choices:

- **P-only or PD.** A P controller on a pure integrator is a first-
  order closed loop with time constant 1/K. Adding I on top would
  wind up when the aircraft is on target and the outer loop stops
  commanding, because the integrator never sees error but keeps its
  accumulated value. So `Ki = 0` on the bank and pitch loops.
- **Small D.** A little derivative damps the closed-loop response
  and keeps overshoot low without adding phase lag. `Kd = 0.05` on
  both attitude axes.
- **Airspeed loop keeps I.** The throttle-to-speed plant *is*
  first-order in reality (drag rise + engine lag), so PI + a
  windup guard is fine. `Kp_V = 0.05, Ki_V = 0.02, Kd_V = 0`.

Gains live in `model/plant_params.m` and load via the model's
`PreLoadFcn`, so opening `autopilot.slx` from anywhere pulls them
into the workspace.

## Saturation

Every PID output feeds a saturation block modelling control-surface
travel:

- Aileron: ±25°
- Elevator: ±25°
- Throttle: 0..1

The saturation limits are read from `plant_params.m` so
`plant_params` is the single source of truth for control envelope.
The wrapper on the CLEARANCE side reads model outputs *after*
saturation, so the aircraft never sees an unsaturated command.

## Code generation

Configured for **reusable-function packaging** (see
`tools/configure_reusable_function.m`). This is the important detail
for the CLEARANCE integration.

Default Embedded Coder puts model state (block IO, continuous states,
external inputs, external outputs) in file-scope globals. That's fine
for a rig with one autopilot but breaks the moment you want two — the
second aircraft trashes the first's integrator. Reusable-function
packaging moves the state into an `RT_MODEL_autopilot_T` struct that
the caller allocates. Every entry point takes a pointer:

```c
void autopilot_initialize(RT_MODEL_autopilot_T *rtM);
void autopilot_step      (RT_MODEL_autopilot_T *rtM);
void autopilot_terminate (RT_MODEL_autopilot_T *rtM);
```

CLEARANCE allocates one per aircraft. A fleet of any size runs
concurrently without any shared state.

Codegen target is `ert.tlc` (Embedded Real-Time) with fixed-step
solver at 0.02 s (50 Hz), ode4. Continuous-time support is on
because the PID filter blocks and actuator lag are continuous.
Makefile generation is off — CLEARANCE compiles the `.c` itself as
part of its normal build.

Regenerating C from the model:

```matlab
run('tools/configure_reusable_function.m')   % once per fresh workspace
rtwbuild('autopilot')
```

Output lands in `autopilot_ert_rtw/`. The five files CLEARANCE needs
are `autopilot.c`, `autopilot.h`, `autopilot_types.h`,
`autopilot_private.h`, `rtwtypes.h` plus two Simulink runtime
headers (`rtw_continuous.h`, `rtw_solver.h`) that the .h references.

## Verification

The regression check in `tools/compare_sim.m` loads
`ci_artifacts/simOut_with_pid_defaults.mat` and asserts the final
`delta_e` from a fresh sim matches the reference within tolerance.
Fails CI on any drift.

The traceability report (`traceability_report.html`) maps every
tagged requirement in `req_map.csv` to a specific block in the
model, with a `HasProbe` column that flags any requirement that
isn't backed by a captured signal. Simulink Requirements Toolbox
regenerates it via `slreq.generateReport`.

The integration test is the sim itself — spawn traffic in CLEARANCE,
watch it fly, issue heading and altitude commands, verify the
aircraft tracks. That test is manual by design: closed-loop
integration with the CLEARANCE plant model is the deliverable, and
if the autopilot behaves badly the operator sees it immediately.

## Integration with CLEARANCE

The plugin module in CLEARANCE is `ClearanceAutopilotMBD`. Files:

- `AutopilotWrapper.h/.cpp` — thin C++ wrapper around the extern-C
  entry points. Runs the outer loops in the wrapper (heading → target
  bank, altitude → target pitch), owns one `RT_MODEL_autopilot_T`
  per instance.
- `AutopilotGeneratedUnit.cpp` — compilation shim. A single TU
  includes `autopilot.c` under `extern "C"` so Unreal Build Tool
  compiles the generated code as part of the module without needing a
  per-file compilation rule.
- `ClearanceAutopilotMBD.Build.cs` — detects the presence of
  `ThirdParty/AutopilotGenerated/include/` and `src/`, flips
  `CLEARANCE_AUTOPILOT_MBD_HAVE_CODEGEN=1`, and adds the include
  paths.

Every aircraft (`UClearanceAircraftBehaviour`) holds one
`FAutopilotWrapper` and calls `Step()` from its tick. When the
aircraft is captured (heading, altitude, and speed all inside
tolerance), the wrapper call is skipped and rates are frozen — this
kills a small limit cycle we saw during hold from the model's D-term
filter re-exciting the loop from floating-point noise.

Two console commands toggle it live:

```
clearance.autopilot.engage    <callsign>
clearance.autopilot.disengage <callsign>
```

Autopilot is default-on for every aircraft.

## What's not in here

Deliberate omissions:

- **No 6-DOF plant.** The model is three controllers. If someone
  wants to run it against a real plant they can build a rig around
  it — the interface is `phi_cmd, theta_cmd, V_cmd` in and `delta_a,
  delta_e, delta_t` out.
- **No yaw damper.** Rudder is unmodelled. The lateral dynamics are
  bank-to-turn only, which is what CLEARANCE's aircraft use.
- **No LQR or gain scheduling.** Cascade PID is enough for the
  operating envelope (transport cruise). Adding either would be a
  branch, not a rewrite.
- **No sensor noise or actuator faults.** Sensors are perfect,
  actuators are the saturation blocks. Realistic sensor models
  belong in the CLEARANCE-side plant, not the controller model.

## Files

```
autopilot.slx                       -- source of truth
model/plant_params.m                -- gains, limits, solver settings
tools/
  add_control_outports.m            -- promotes delta_e/delta_a to root outports
  expose_command_inports.m          -- promotes phi_cmd/theta_cmd to root inports
  configure_reusable_function.m     -- switches code interface to reusable
  run_model_tests_and_build.m       -- CI entry
  compare_sim.m                     -- regression baseline check
ci_artifacts/                       -- reference sim outputs + plot
req_map.csv                         -- requirement -> block ID map
traceability_report.csv/html        -- rendered traceability
```
