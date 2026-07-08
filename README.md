# autopilot-mbd

Model-Based Design of an aircraft autopilot in Simulink. Cascaded PID
controllers (heading → bank → aileron, altitude → pitch → elevator,
airspeed → throttle) auto-code-generated to portable C via Embedded
Coder, verified against reference sim outputs, and integrated into the
[CLEARANCE](https://github.com/) air traffic control simulator through a
UE C++ bridge module.

The MATLAB / Simulink model is the source of truth. Everything under
`codegen_out/` is generated. Verification traces every requirement to
a test — no untraced controller.

---

## Repository layout

```
autopilot_repo/
├── autopilot.slx              <-- Simulink model (source of truth)
├── autopilot.slxc             <-- Simulink cache
├── model/
│   └── plant_params.m         <-- aircraft state-space + trim params
├── tools/
│   ├── run_model_tests_and_build.m   <-- CI entry point
│   ├── create_autopilot_model.m
│   ├── populate_autopilot_subsystem.m
│   ├── fix_autopilot_wiring.m
│   └── add_alias_inports.m
├── ci_artifacts/              <-- smoke sim outputs (uploaded by CI)
│   ├── simOut_with_pid_defaults.mat
│   ├── delta_e_logs.mat
│   ├── delta_e_log2.csv
│   └── delta_e_plot.png
├── req_map.csv                <-- requirement → test mapping
├── traceability_report.csv    <-- Simulink Requirements Toolbox export
├── traceability_report.html
├── docs/                      <-- design + verification specifications
│   ├── AUTOPILOT_MBD_DESIGN.md
│   ├── VERIFICATION_REPORT.md
│   ├── VerificationTest_HDG.md
│   ├── Reproducible Codegen (CI).md
│   └── ...
└── .github/workflows/ci.yml   <-- MATLAB CI pipeline
```

## Getting started

Open `autopilot.slx` in Simulink R2023b or later with the following
toolboxes installed:

- Simulink
- Simulink Test
- Embedded Coder
- MATLAB Coder
- Simulink Coder

Run tests + generate code locally:

```matlab
addpath(genpath(pwd))
cd tools
run_model_tests_and_build
```

Outputs land under `ci_artifacts/`. Generated C source is written to
`slprj/ert/autopilot/` — the CI job copies the `.c` / `.h` files out to
`codegen_out/` and uploads them as a workflow artefact.

## Controller architecture

Three cascaded loops running at 50 Hz:

1. **Heading hold** — heading error → bank command → aileron.
2. **Altitude hold** — altitude error → pitch command → elevator.
3. **Airspeed hold** — airspeed error → throttle command.

Plant model is a linearised trim about a nominal cruise point (see
`model/plant_params.m`). Full state-space matrices, gain tables, and
tuning rationale are in [`docs/AUTOPILOT_MBD_DESIGN.md`](docs/AUTOPILOT_MBD_DESIGN.md).

## Verification

Every requirement in `req_map.csv` is tagged in the corresponding
Simulink Test case. `traceability_report.html` renders the coverage
matrix. Individual verification narratives (e.g. the heading-hold
step response) live under `docs/`.

## Integration with CLEARANCE

The CLEARANCE ATC simulator carries a `ClearanceAutopilotMBD` UE plugin
module that consumes the generated C code. On every green build of this
repo the workflow uploads two artefacts:

- `autopilot-generated-c` — the `.c` / `.h` output.
- `autopilot-sim-artefacts` — sim outputs + traceability CSVs.

Integrators download the generated-code artefact, drop it into
CLEARANCE's `Plugins/ClearanceSim/ThirdParty/AutopilotGenerated/`, and
the UE side picks it up on next rebuild. Drop-in instructions ship
inside CLEARANCE's plugin tree.

## Continuous integration

`.github/workflows/ci.yml` runs on every push to `main` and every PR:

1. Set up MATLAB (via `matlab-actions/setup-matlab@v2`).
2. Run `tools/run_model_tests_and_build.m` — smoke sim + Test Manager.
3. `rtwbuild(model)` — Embedded Coder generates C.
4. Upload `codegen_out/`, `ci_artifacts/`, and traceability reports as
   workflow artefacts.

The pipeline fails on any script error, missing test case, or codegen
failure. Green = the model is buildable, verifiable, and portable.

## License

MIT — see [`LICENSE`](LICENSE).
