### Production Log — Autopilot Model Integration Project

Summary
- Repository root: C:\Users\jerem\Desktop\Simulink\autopilot_repo
- Purpose: Run Simulink smoke tests, collect artifacts, prepare code generation, and integrate generated runtime into an Unreal Engine (UE) project via a lightweight C++ adapter. All non-toolchain tasks completed; final codegen/compile step requires MATLAB toolchain registration and a restart.

Model Files and Location
- Primary model: tools/autopilot.slx
- Working copy loaded from: C:\Users\jerem\Desktop\Simulink\autopilot_repo\tools\autopilot.slx

Simulation & Workspace Actions
- Issue discovered: Model required PID gain workspace variables (Kp_V, Ki_V, Kd_V, Kp_phi, Ki_phi, Kd_phi, Kp_theta, Ki_theta, Kd_theta) that were missing; simulation would not run until defined.
- Action taken: Created reasonable default PID gains in the base workspace and used assignin('base', ...) so noninteractive runs (matlab -batch) could evaluate block parameters.
  - Values used (defaults, adjust later):  
    Kp_V=1.0, Ki_V=0.1, Kd_V=0.01  
    Kp_phi=2.0, Ki_phi=0.1, Kd_phi=0.05  
    Kp_theta=2.0, Ki_theta=0.1, Kd_theta=0.05
- Simulation executed (smoke sim) with those defaults. Simulation ran successfully after fixes (no remaining ErrorMessage in Simulink.SimulationOutput).

Saved Simulation Artifacts
- Saved simulation output (Simulink.SimulationOutput):
  - tools/ci_artifacts/simOut_with_pid_defaults.mat
- Extracted signals and exports:
  - tools/ci_artifacts/delta_e_logs.mat  (variables: delta_e_log2, delta_e_log_ws, tout)
  - tools/ci_artifacts/delta_e_log2.csv (two-column CSV: time, delta_e_log2)
  - tools/ci_artifacts/delta_e_plot.png (PNG plot of delta_e vs time)
- Observed key values:
  - final delta_e_log2 = 0.580025 (printed from simOut)
  - simOut contains logged fields: delta_e_log2, delta_e_log_ws, phi_cmd_log, phi_internal_log, several probe_* signals, tout, yout.

CI Script and Behavior
- Script created: tools/run_model_tests_and_build.m
  - Steps implemented:
    1. Load model (load_system)
    2. Run smoke simulation (sim) and save simOut to artifacts
    3. Run Test Manager tests if Simulink Test present (safe guarded)
    4. Attempt model coverage if Simulink Coverage present (safe guarded)
    5. Traceability report generation from req_map.csv (if present): produces traceability_report.csv and HTML
    6. Attempt model build/codegen (slbuild) inside try/catch; on failure warns and continues
    7. Save workspace snapshot (best-effort)
    8. Zip artifacts to tools/ci_artifacts.zip
- Behavior observed when run in current environment:
  - Smoke sim initially failed until PID variables were defined.
  - slbuild failed due to missing/invalid codegen toolchain (MATLAB registry mismatch).

Toolchain / Codegen Status
- mex -setup and mex -setup cpp were run and reported selection of "Microsoft Visual C++ 2026" for MEX compilation.
- slbuild / rtwbuild attempts fail because MATLAB's codegen toolchain registry does not list the default toolchain (error: Unable to determine the default toolchain).
- Root cause: MEX configured but codegen toolchain registry not refreshed / requires MATLAB restart / additional registration steps.
- Interim workaround applied:
  - Avoided compile/link by generating only simulation artifacts locally.
  - Recommended final steps: run mex -setup -v, restart MATLAB, then slbuild('autopilot') or rtwbuild('autopilot') to produce generated sources and optional compiled library.

Unreal Engine Integration Scaffolding
- UE adapter stubs added:
  - Source/AutopilotBridge/Public/AutopilotWrapper.h
  - Source/AutopilotBridge/Private/AutopilotWrapper.cpp
  - Stubs declare expected C API from generated code: autopilot_initialize, autopilot_step, autopilot_terminate; include FAutopilotWrapper class with Initialize/Tick/Shutdown methods and placeholders for mapping logic.
- Build configuration guidance provided:
  - Build.cs snippet created to reference ThirdParty/AutopilotGenerated (include, src, lib). It handles both linking a prebuilt library (autopilot.lib) or compiling generated .c files directly into the UE module.
- Third-party layout created:
  - ThirdParty/AutopilotGenerated/
    - include/   (target for generated headers)
    - src/       (target for generated C source files)
    - lib/       (optional: compiled .lib/.dll)
    - README.md  (instructions for integrating generated code)

Traceability, Tests, and Reporting
- Traceability: run_model_tests_and_build.m contains logic to read req_map.csv (if present) and produce traceability_report.csv and HTML. req_map.csv was not present, so this step was skipped.
- Regression test script added:
  - tools/compare_sim.m — compares final delta_e to expected value (example expected=0.58, tol=1e-2) and returns pass/fail exit code for CI.
- Coverage and Test Manager steps are guarded when toolboxes are not installed; script prints clear messages when skipped.

Repository & CI
- Files added and committed (examples, not exhaustive):
  - tools/run_model_tests_and_build.m
  - tools/compare_sim.m
  - tools/ci_artifacts/* (simulation outputs and exports)
  - Source/AutopilotBridge/AutopilotWrapper.h/.cpp
  - .github/workflows/matlab-ci.yml (GitHub Actions workflow targeting a self-hosted Windows runner with MATLAB)
  - .gitignore (MATLAB, slprj, codegen outputs, Unreal artifacts)
  - README.md summarizing repo layout and integration notes
  - ThirdParty/AutopilotGenerated/README.md and directories
- CI workflow behavior:
  - Workflow runs matlab -batch to execute tools/run_model_tests_and_build.m on a self-hosted Windows runner and uploads tools/ci_artifacts.zip and contents as artifact.
  - CI currently defers slbuild compile step to runner with a registered toolchain (recommended to run after mex -setup and restart).

Files and Paths of Note
- Model: tools/autopilot.slx
- CI script: tools/run_model_tests_and_build.m
- Regression test: tools/compare_sim.m
- Artifacts (local): tools/ci_artifacts/
  - simOut_with_pid_defaults.mat
  - delta_e_logs.mat
  - delta_e_log2.csv
  - delta_e_plot.png
  - ci_artifacts.zip
- UE adapter stubs: Source/AutopilotBridge/
- Integration target area for generated code: ThirdParty/AutopilotGenerated/

Decisions, Assumptions, and Rationale
- Default PID gains applied in workspace are temporary development defaults to enable smoke simulation and artifact generation. These must be reviewed and tuned by control engineers; they are not assumed final production gains.
- Code generation/compilation is postponed until a robust toolchain registration is completed and MATLAB restarted. This avoids losing the interactive session and the current chat context.
- Generated code will be integrated into UE either as:
  - compiled static library (autopilot.lib) that UE links against, or
  - compiled directly as C sources within the UE module build (both options supported via Build.cs placeholders).
- SIL/PIL testing and numeric parity checks are recommended before deployment; CI should include parity checks once codegen is available on the runner.

Outstanding Items (next steps and owners)
1. Owner: Developer (you)
   - Review and replace default PID values with validated controller gains; record final values in a parameter file or script.
   - If model edits are needed (control law, logging, block changes), make them and save a versioned model (autopilot_vX.slx).
2. Owner: Developer
   - Register a codegen-capable toolchain:
     - Run: mex -setup -v; mex -setup cpp -v
     - Restart MATLAB (required to refresh toolchain registry)
     - Run: slbuild('autopilot')  (or rtwbuild('autopilot') if you only want sources)
3. Owner: Developer / Build Engineer
   - After codegen: copy generated include/.c/.lib files into ThirdParty/AutopilotGenerated and update Build.cs to compile/link them.
   - Implement mapping code in AutopilotWrapper.cpp using actual generated I/O names and structures.
4. Owner: QA / CI
   - Add SIL/PIL tests and expected-reference datasets to CI. Extend run_model_tests_and_build.m to run parity checks and fail the CI job if outputs diverge above tolerance.
5. Owner: Documentation
   - Update README with final build and integration steps, model version, and verification criteria.

Appendix — Commands Used During Work (examples)
- Run smoke sim and save output (MATLAB):
  - Define defaults and assign to base workspace via assignin('base',...)
  - simOut = sim(modelFull,'SaveOutput','on','SaveFormat','StructureWithTime');
  - save('tools/ci_artifacts/simOut_with_pid_defaults.mat','simOut');
- Create CSV and PNG (MATLAB):
  - writematrix([simOut.tout, simOut.delta_e_log2], 'tools/ci_artifacts/delta_e_log2.csv');
  - h=figure('Visible','off'); plot(simOut.tout, simOut.delta_e_log2); saveas(h,'tools/ci_artifacts/delta_e_plot.png'); close(h);
- Register MEX compiler (MATLAB interactive):
  - mex -setup -v
  - mex -setup cpp -v
- Generate code (after restart):
  - load_system('autopilot'); slbuild('autopilot')  % requires registered toolchain
  - or rtwbuild('autopilot')                       % generate sources only

If you want, I will now:
- produce a versioned model save script that snapshots tools/autopilot.slx to tools/autopilot_v1.slx and records the current config parameters; or
- produce a detailed change manifest template for model edits (block-level checklist) to include in the repo.
