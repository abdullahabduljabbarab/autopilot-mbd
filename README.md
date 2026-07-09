# autopilot-mbd

Model-Based Design of a **cascade autopilot** for aircraft attitude and
airspeed control in Simulink. Three inner-loop PID controllers (bank →
aileron, pitch → elevator, airspeed → throttle) with saturation and
probe-based verification, plus outer loops (heading → target bank,
altitude → target pitch) that turn ATC-level commands into inner-loop
setpoints. Auto-code-generated to portable C via Embedded Coder using
**reusable-function packaging**, so every consumer gets its own
per-instance state — a fleet of aircraft can run the same generated
model concurrently without shared globals.

Integrated live into the [CLEARANCE](https://github.com/) ATC simulator:
every aircraft in the sim flies under the Simulink-generated
autopilot, each carrying its own PID history, filter states, and
integrator memory. See [Integration with CLEARANCE](#integration-with-clearance)
below.

![Autopilot subsystem — inner-loop PID controllers](docs/img/controller_internals.png)

*Inside `AutopilotSubsystem`: three PID controllers with output
saturation. `phi_cmd`/`theta_cmd`/`V_cmd` inputs drive the bank, pitch,
and airspeed hold loops; outputs are `delta_a`/`delta_e`/`delta_t`
(aileron / elevator / throttle) surface commands.*

---

## Controller architecture

**Two-tier cascade.** The Simulink model runs the **inner** loops
(fast attitude and airspeed hold); the CLEARANCE-side wrapper runs
the **outer** loops (slow ATC command tracking) that generate the
`phi_cmd` / `theta_cmd` setpoints:

```
ATC command (heading / altitude / speed)
        │
        ▼
    ┌───────────────────────────────────────────┐
    │ OUTER loop  (in CLEARANCE C++ wrapper)    │
    │   heading err  →  target bank  (phi_cmd)  │
    │   altitude err →  target pitch (theta_cmd)│
    └───────────────────────────────────────────┘
        │  phi_cmd, theta_cmd, V_cmd, phi, theta, V
        ▼
    ┌───────────────────────────────────────────┐
    │ INNER loop  (Simulink → generated C)      │
    │   PID_phi   :  phi_cmd - phi   → delta_a  │
    │   PID_theta :  theta_cmd - theta → delta_e│
    │   PID_V     :  V_cmd - V       → delta_t  │
    └───────────────────────────────────────────┘
        │  delta_a, delta_e, delta_t
        ▼
    Aircraft dynamics (in CLEARANCE)
```

Each PID has a saturation block modelling control-surface travel
limits (±25° aileron, ±25° elevator, 0..1 throttle). Probe outports on
pre- and post-saturation signals feed the traceability report and the
regression check.

Inner-loop gains (in `model/plant_params.m`) are tuned for a
pure-integrator plant — the CLEARANCE aircraft dynamics integrate
control-surface deflection directly, so P-only with a small D term
gives clean tracking without limit-cycling. Integral action is
disabled on `PID_phi` and `PID_theta` (Ki = 0) because the outer loop
zeroes steady-state error at the commanded heading / altitude.

![Top-level model](docs/img/model_top_level.png)

*Top-level: root inports (`phi_cmd`, `theta_cmd`, `V_cmd`, `phi`,
`theta`, `V`) feed `AutopilotSubsystem`; root outports (`delta_a_out`,
`delta_e_out`, `delta_t_out`) surface the control commands. Actuator
lag models (`1/(0.05s+1)`) live on the top level for verification;
CLEARANCE substitutes its own aircraft dynamics for integration.*

---

## Reusable-function code generation

The model is configured (`tools/configure_reusable_function.m`) with
**Code Interface Packaging = Reusable function**. Generated entry
points take a per-instance model pointer:

```c
void autopilot_initialize(RT_MODEL_autopilot_T *rtM);
void autopilot_step      (RT_MODEL_autopilot_T *rtM);
void autopilot_terminate (RT_MODEL_autopilot_T *rtM);
```

Every field of the run-time state (`blockIO`, `contStates`, `inputs`,
`outputs`) lives inside the `RT_MODEL_autopilot_T` struct pointed to
by `rtM`. Consumers allocate one per aircraft. There are **no
file-scope globals** and no shared state between instances — a fleet
of any size runs concurrently on the same generated `.c`.

---

## Repository layout

```
autopilot_repo/
├── autopilot.slx              <-- Simulink model (source of truth)
├── autopilot.slxc             <-- Simulink cache
├── model/
│   └── plant_params.m         <-- linearised trim params + PID gains
├── tools/
│   ├── run_model_tests_and_build.m       <-- CI entry point
│   ├── compare_sim.m                     <-- tolerance-based regression
│   ├── add_control_outports.m            <-- expose delta_e / delta_a as root outports
│   ├── expose_command_inports.m          <-- promote step blocks to root inports
│   └── configure_reusable_function.m     <-- switch code interface to reusable
├── ci_artifacts/              <-- smoke sim outputs (uploaded by CI)
├── req_map.csv                <-- requirement → block mapping
├── traceability_report.csv    <-- Simulink Requirements Toolbox export
├── traceability_report.html
├── docs/
│   ├── AUTOPILOT_MBD_DESIGN.md
│   ├── VERIFICATION_REPORT.md
│   ├── VerificationTest_HDG.md
│   └── img/                   <-- README figures
└── .github/workflows/ci.yml   <-- MATLAB CI pipeline
```

## Getting started

Open `autopilot.slx` in Simulink R2023b or later with:

- Simulink
- Simulink Test
- Embedded Coder
- MATLAB Coder
- Simulink Coder

Run smoke sim + regression check locally:

```matlab
addpath(genpath(pwd))
run('model/plant_params.m')
sim('autopilot')
tools/compare_sim
```

Generate C for integration:

```matlab
run('tools/configure_reusable_function.m')  % once, sets the code interface
rtwbuild('autopilot')                        % produces autopilot.c + autopilot.h
```

Output lands in `autopilot_ert_rtw/`. The CI job also copies `.c` /
`.h` to `codegen_out/` and uploads as a workflow artefact.

## Verification

Every requirement in `req_map.csv` traces to a specific block.
`traceability_report.html` renders the coverage matrix.

![Traceability report excerpt](docs/img/traceability.png)

*Requirement → block mapping generated by the Simulink Requirements
Toolbox. `HasProbe = 1` means a verification probe is attached to
that block for signal capture.*

`tools/compare_sim.m` loads `ci_artifacts/simOut_with_pid_defaults.mat`
and fails CI on any drift outside tolerance.

![Elevator step response](docs/img/step_response.png)

*Reference elevator command (top) and aileron command (bottom) over
the smoke sim. The elevator shows PID_theta's derivative kick at
t=1 s followed by the integrator ramp toward saturation. The
aileron saturates on the phi command step at t=0.*

## Integration with CLEARANCE

The CLEARANCE ATC simulator carries a `ClearanceAutopilotMBD` UE
plugin module. Its architecture:

- **Per-aircraft `FAutopilotWrapper`** — each aircraft's
  `UClearanceAircraftBehaviour` owns one. The wrapper allocates its
  own `RT_MODEL_autopilot_T` on first use.
- **`AutopilotGeneratedUnit.cpp` shim** — a single translation unit
  includes `autopilot.c` from `ThirdParty/AutopilotGenerated/src/` so
  Unreal Build Tool compiles it into the module without needing a
  per-file compilation rule.
- **`Build.cs` auto-detection** — presence of the generated `include/`
  and `src/` directories flips `CLEARANCE_AUTOPILOT_MBD_HAVE_CODEGEN=1`
  and the wrapper switches from a P-only stub to the real generated
  model transparently.
- **Cascade wiring** — the wrapper computes `phi_cmd` (heading → bank)
  and `theta_cmd` (altitude → pitch) outer loops in C++, pushes them
  plus the aircraft's live state into the model's `ExtU` struct,
  steps once, reads back `delta_a` / `delta_e` / `delta_t` from
  `ExtY`, and translates them into aircraft rate changes.

Console commands in-sim:

```
clearance.autopilot.engage <callsign>
clearance.autopilot.disengage <callsign>
```

Every aircraft is autopilot-engaged by default. Disengage puts a
specific aircraft back on CLEARANCE's built-in rate-limited slew for
A/B comparison.

## Continuous integration

`.github/workflows/ci.yml` runs on manual dispatch (MATLAB licensing
on GitHub-hosted runners is a separate concern — see the CI notes in
CLEARANCE). Steps:

1. Set up MATLAB (`matlab-actions/setup-matlab@v2`).
2. Run `tools/run_model_tests_and_build.m` — smoke sim + Test Manager.
3. `rtwbuild('autopilot')` — Embedded Coder generates C.
4. Upload `codegen_out/`, `ci_artifacts/`, and traceability reports as
   workflow artefacts.

## License

MIT — see [`LICENSE`](LICENSE).
