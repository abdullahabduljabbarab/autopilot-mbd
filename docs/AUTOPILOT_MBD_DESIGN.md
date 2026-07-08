This is a revised, elevated design brief for autopilot-mbd that incorporates the high-priority changes, improved test realism, codegen/CI suggestions, and clearer assumptions. It preserves your original structure but replaces ambiguous items with concrete, defensible choices and actionable files/params.

### Scope Summary

Model-based design of a cascaded autopilot in MATLAB/Simulink. Deliverables: parameterised Simulink model, plant parameter script, verification test suite, reproducible Embedded C codegen script, and CLEARANCE C++ integration notes. Uses published aerodynamic derivatives (citation), anti-windup, actuator/sensor dynamics, and CI-friendly codegen.

### Table Of Contents

- Prereqs & Toolchain
- Traceable Plant Source & Units
- Requirements (unchanged IDs, clarified metrics)
- Plant Model (longitudinal + lateral) with citations and clarified states
- Controller Architecture (anti-windup, actuator dynamics, sensor models)
- Parameter Script (plant_params.m) — variables expanded
- Simulink Model Construction (key implementation rules)
- Controller Tuning Methods (PID Tuner, root-locus, LQR) — guidance
- Verification Tests (tightened definitions, measurement windows, tolerances)
- Code Generation & Reproducibility (regen_code.m, CI notes)
- CLEARANCE Integration (facade, sampling, equivalence test)
- Repo Layout, README, and Deliverables
- Extensions and Portfolio Positioning

### Prereqs & Toolchain

- MATLAB R2024a or newer, Simulink.
- Control System Toolbox, Simulink Control Design, Aerospace Blockset (recommended), Embedded Coder + MATLAB Coder (for codegen).
- Recommend running tests on the same MATLAB release used to generate committed artifacts; record version string in generated/MATLAB_VERSION.txt.

### Traceable Plant Source & Units

- Use published aerodynamic derivatives and cite a specific source in docs/PLANT_MODEL_DERIVATION.pdf. Example accepted choices:
  - Stevens & Lewis — Aircraft Control and Simulation, Appendix A (F-16) — good for relevant control practice.
  - Nelson — Flight Stability and Automatic Control, Appendix B.
  - Cook — Flight Dynamics Principles, relevant airliner tables.
- Units (explicit):
  - Angles: radians internally; degrees only for human-readable tables.
  - Time: seconds.
  - Length: meters (altitude, h).
  - Speed: m/s (note 1 kt ≈ 0.514444 m/s).
  - Forces/moments: SI units as per cited source.

State/measurement conventions (explicit):
- Longitudinal x_long = [u, w, q, theta] where u,w are perturbations (m/s). Trim airspeed V0 in m/s. Altitude h is an integrated vertical position (m).
- Lateral simplified state x_lat = [p, phi, psi] with psi heading (rad).

Add a short assumptions section: small-perturbation linearisation, trim at level flight 125 m/s, valid for approach/transport envelope only.

### Requirements (Section 2) — Clarifications

- Keep REQ-AP-001 ... REQ-AP-013 as originally defined.
- Measurement conventions:
  - Steady-state (SS) error = mean error over final 10 s of simulation unless otherwise specified.
  - Settling time (2%) measured from command time until error magnitude remains within 2% of final command for 10 continuous seconds.
- Numerical tolerances for codegen equivalence softened: default tolerance = 1e-4 for double vs single/float mismatch; allow channel-specific tolerances; report max abs and RMS.

### Plant Model (Section 3) — Concrete Values and Notes

Cite the chosen reference in docs and replace "notional" with cited derivatives. For the brief we keep your numeric example but mark it as "example (from [cite])" — replace before public repo.

Longitudinal matrices (explicit A_long, B_long) as in original. Add conversion note:
- Altitude h is not a state of x_long; derive h by integrating body-axis vertical speed to inertial vertical velocity:
  - approximate: h_dot = - (w + V0 * sin(theta)) ≈ -w for small theta; implement a small Integrator block with sign convention documented. Provide explicit integrator block in Simulink instructions (see below).
- Provide explicit formula for measuring true airspeed:
  - V = V0 + u (since u is forward-axis perturbation about trim V0).

Lateral model:
- Use roll-subsidence simplification but explicitly show state ordering and A_lat used in the Simulink State-Space block:
  - x_lat = [p; phi; psi], A_lat = [Lp 0 0; 1 0 0; 0 V0 0], B_lat = [Lda; 0; 0].
- Document Dutch-roll omitted and recommend adding yaw damper as extension (washout + rudder).

Actuator dynamics:
- Add optional first-order actuator lag and rate limits for elevator and aileron:
  - τ_act (sec) and rate_limit (rad/s), implemented using a Transfer Fcn (1/(τ s + 1)) plus Rate Limiter blocks.
- Put actuator parameters in plant_params.m: TAU_ELEV, TAU_AIL, RATE_ELEV, RATE_AIL.

Sensor dynamics:
- Add optional sensor low-pass filters (time constant TS_SENSOR) and configurable noise (std dev) to make tests robust. Parameters in plant_params.m.

### Controller Architecture (Section 4) — Elevations

Cascaded structure unchanged, but:

- Anti-windup:
  - Use built-in PID Controller anti-windup (back-calculation) or PI Controller with Integral Anti-Windup. Add BackCalc gain Kb variables in plant_params.m.
  - For altitude and airspeed integrators, enable anti-windup. Document the chosen method and parameter (Kb_inv time constant).
- Saturation limits are variables in plant_params.m:
  - PHI_LIMIT_RAD = deg2rad(25); THETA_LIMIT_RAD = deg2rad(10); DELTA_T_MIN = 0; DELTA_T_MAX = 1.
- Sampling / inner-loop rate:
  - Target inner-loop sample rate for codegen: 10 Hz (fixed-step 0.1 s) — make this a variable (CODEGEN_STEP).
- Controller gains: keep initial set, but store in plant_params.m with comments. Add recommended anti-windup Kb values:
  - ALT_INT_KB = 1/(5) (example) meaning anti-windup time constant 5 s; tune as needed.

Actuator anti-windup / saturation interaction: document that PID blocks should be configured with Output Limiting and Anti-windup enabled and that external saturation blocks must reference the same limit variables to keep consistent behavior for Embedded Coder signal names.

### Parameter Script: model/plant_params.m (elevated)

Create a single authoritative parameter file that includes:
- All aerodynamic derivatives (with citation comment).
- All controller gains and anti-windup parameters.
- All saturation limits and actuator/sensor time constants.
- Simulation parameters (SIM_STOP_TIME, CODEGEN_STEP, MAX_STEP_VARSTEP).
- Numeric tolerances for verification (TOL_CODEGEN_EQ = 1e-4; TOL_CHANNEL = struct per channel).
- A printed summary with MATLAB version and timestamp (for reproducibility).

Example header (replace with full file in repo):
```matlab
% plant_params.m - authoritative parameter file (values must be referenced by name in Simulink)
% Source: Stevens & Lewis, Appendix A (reference details here)
Xu = -0.021; Xw = 0.122; ... % etc
V0 = 125; g = 9.81;
% Saturation limits
PHI_LIMIT_RAD = 25*pi/180;
THETA_LIMIT_RAD = 10*pi/180;
DELTA_T_MIN = 0; DELTA_T_MAX = 1;
% Actuator dynamics
TAU_ELEV = 0.05; TAU_AIL = 0.05;
RATE_ELEV = 30*pi/180; RATE_AIL = 30*pi/180;
% Controller gains
K_hdg = 1.5; K_phi = 3.0; K_p = 0.5; ...
% Anti-windup
ALT_INT_BACKCALC = 0.2; % back-calculation coeff (Kb)
% Simulation settings
SIM_STOP_TIME = 120;
CODEGEN_STEP = 0.1; % fixed-step for codegen
MAX_STEP_VARSTEP = 0.05;
% Verification tolerances
TOL_CODEGEN_EQ = 1e-4;
```

Set as PreLoadFcn: run(fullfile(fileparts(mfilename('fullpath')),'plant_params.m')); This uses module-local path.

### Simulink Model Construction (Section 5) — Key Implementation Rules

- Model file: model/autopilot.slx. Solver defaults: variable-step ode45 for normal simulation with MaxStep = MAX_STEP_VARSTEP. For codegen, switch to fixed-step (CODEGEN_STEP) and ert.tlc target.
- All gains, saturation limits, actuator/sensor time constants, and sample times must reference names from plant_params.m (no hard-coded numbers inside blocks).
- Autopilot subsystem interface must match CLEARANCE facade ordering. Name every signal explicitly; use descriptive signal names (e.g., psi_err, phi_cmd) — Embedded Coder uses these names.
- Implement altitude integration explicitly and document sign conventions:
  - h_dot = - (w + V0 * sin(theta)) approximated as h_dot = -w for small angles, or use full transform if you add nonlinear 6-DOF extension.
- Place actuator dynamics between controller outputs and plant inputs; include Rate Limiter and first-order lag Transfer Fcn.
- Use PID Controller blocks with anti-windup enabled (Back-calculation) for altitude and airspeed loops or use PI Controller with built-in anti-windup.
- Signal Editor scenarios in model/scenarios — one .mat per test.

### Controller Tuning (Section 6) — Guidance

- Start with PID Tuner for inner loops (pitch/pitch-rate and roll/roll-rate) using Simulink Control Design. Use linearisation at trim.
- For defensible interview approach, include a root-locus / rltool example and an LQR alternative: show how you moved from cascaded PID to state-feedback and compare closed-loop bandwidth and margins.
- Document the final tuned gains and post-tuning margins in docs/VERIFICATION_REPORT.md.

### Verification Tests (Section 7) — Elevations & Procedures

General rules:
- Each test is a MATLAB function in verification/test_*.m that:
  - Loads plant_params.m, opens autopilot.slx, sets solver to match test (variable-step vs fixed-step as needed).
  - Loads a named Signal Editor scenario .mat to drive psi_cmd, h_cmd, V_cmd.
  - Runs sim with STOP_TIME = SIM_STOP_TIME or test-specific stop time.
  - Computes metrics over explicit windows (steady-state = mean last 10 s; settling time uses the 2% band over final value with 10 s persistence).
  - Produces PNG plots (time-series) and returns struct with name, status, detail, stats (max, rms, steady-state).
- Tests now report numeric pass/fail plus a small JSON/struct summary saved to verification/results_<timestamp>.mat for CI.

Specific adjustments:
- TEST-AP-HDG-STEP:
  - SS error: mean error over last 10 s ≤ deg2rad(1).
  - Settling time ≤ 30 s measured as above.
  - Test duration 60 s (simulate t=0..60).
- TEST-AP-BANK-LIMIT:
  - Use worst-case large heading step; pass if max |phi| ≤ PHI_LIMIT_RAD.
- TEST-AP-ALT-STEP:
  - Input 1000 ft = 304.8 m; SS error ≤ 20 ft ≈ 6.096 m.
  - Settling ≤ 60 s.
- TEST-AP-PITCH-LIMIT: worst-case altitude step; check max |theta| ≤ THETA_LIMIT_RAD.
- TEST-AP-IAS-STEP:
  - 125 to 140 m/s test; SS error ≤ 2 kt ≈ 1.0289 m/s; settling ≤ 45 s.
- TEST-AP-BODE:
  - Linearise inner pitch loop at defined trim (documented in test) and evaluate margin(sys) or margin(L) to obtain PM and GM. Pass: PM ≥ 45°, GM ≥ 6 dB.
  - Save bode plot to verification/bode.png and margins to results struct.
- TEST-AP-DIST-GUST:
  - Use a vertical gust step of 3 m/s (≈ 10 ft/s) injected into w at t=5 s.
  - Measure time to return to |err| ≤ 6 m (20 ft) after disturbance; must be ≤ 15 s.
- TEST-AP-CODEGEN-EQ:
  - Equivalence test uses sim outputs (double) vs generated C model run in the MATLAB executable harness (or S-Function produced by codegen) sampled at the same 10 Hz.
  - Tolerances: default TOL_CODEGEN_EQ (1e-4) absolute; also compute RMS error and per-channel maxima. Fail only if max abs > tol or RMS large.
  - Save sim_ref.mat and comparison PNGs.
- TEST-AP-CLEARANCE-INT:
  - Integration test described; encourage using CSV of reference trajectory and a CLEARANCE automation test that compares outputs within configured tolerances (use slightly looser tolerance than codegen eq, e.g., 1e-3).

Make test runner robust: verification/run_all_tests.m collects results, writes human-friendly summary plus machine-readable JSON (or MAT) output. Use consistent time-stamped filenames for artifacts.

### Code Generation & Reproducibility (Section 8)

Provide scripts and conventions:

- generated/regen_code.m:
  - Loads plant_params.m, opens model, sets model config params programmatically (SystemTargetFile='ert.tlc', SolverType='Fixed-step', FixedStep=CODEGEN_STEP), sets BuildConfiguration to 'Faster Runs' or specific options, and calls slbuild('model/autopilot.slx').
  - On success, copy selected generated C files (autopilot.c, autopilot.h, rtwtypes.h) into generated/ and write generated/METADATA.json with MATLAB version, model MD5, timestamp, and build options.
  - Example workflow:
```matlab
% regen_code.m - run from repo root
run(fullfile('model','plant_params.m'));
model = 'model/autopilot';
load_system(model);
set_param(model,'SystemTargetFile','ert.tlc','SolverType','Fixed-step','FixedStep',num2str(CODEGEN_STEP));
slbuild(model);
% copy outputs and write metadata...
```
- CI: provide a GitHub Actions (or similar) job template that runs regen_code.m inside a hosted MATLAB job (or uses MathWorks CI) to ensure deterministic codegen. Store diffs (git diff) in artifacts so reviewers can see generated changes.

Numeric equivalence guidance:
- Use identical sample rate between Simulink reference and C harness (CODEGEN_STEP).
- If using single-precision in generated code, set Simulink model to use single or cast comparison appropriately. Report both absolute and relative errors.

### CLEARANCE Integration (Section 9)

- Vendor generated C files into CLEARANCE/ThirdParty/AutopilotMBD.
- Provide a C++ facade FClearanceAutopilot with methods:
  - void SetCommand(float PsiCmdRad, float HCmdM, float VCmdMps);
  - void UpdateMeasurements(float Phi, float Theta, float Psi, float H, float V, float p, float q);
  - void Step(); // calls autopilot_step()
  - float GetAileron(), GetElevator(), GetThrottle();
- Ensure timing: Step() invoked at CODEGEN_STEP frequency (10 Hz) from UClearanceAircraftBehaviour::Tick using an accumulator to handle Unreal tick rate.
- Provide a small integration test harness: convert sim_ref.mat to CSV, drive facade, and compare outputs. Store the CSV and comparison results in integration/tests/.

### Repo Layout (elevated)

Add/modify files:

- model/
  - autopilot.slx
  - plant_params.m
  - scenarios/*.mat
- verification/
  - run_all_tests.m
  - test_*.m (eight test functions)
  - results/ (auto-created per run with timestamped MAT/PNG)
- generated/
  - autopilot.c, autopilot.h, rtwtypes.h (committed or produced by CI)
  - regen_code.m
  - METADATA.json
  - MATLAB_VERSION.txt
- integration/
  - CLEARANCE-INT-NOTES.md
  - fc_autopilot_facade.cpp/.h (example skeleton)
- docs/
  - DESIGN.md (this doc)
  - REQUIREMENTS.md
  - V_AND_V_PLAN.md
  - PLANT_MODEL_DERIVATION.pdf (with citation)
- README.md (updated quick start and commands)
- .github/workflows/matlab_regen.yml (optional CI template)
- LICENSE, CONTRIBUTING.md

### Deliverables (explicit)

- Fully parameterised Simulink model and plant_params.m.
- regen_code.m and simple CI instructions.
- Eight verification tests with plots and machine-readable results.
- Integration facade skeleton for CLEARANCE and an equivalence CSV.
- Documentation with cited plant source and derivation.

### Extensions & Portfolio Positioning

- Nonlinear 6-DOF plant, gain-scheduling, yaw-damper, Dryden turbulence, sensor noise + Kalman filter, actuator faults, HIL. Each extension should be a separate branch/PR with its own verification additions.
- Suggested CV bullet (concise): "Developed a model-based autopilot using MATLAB/Simulink with traceable aerodynamic derivatives, cascaded control with anti-windup and actuator dynamics, verified against 8 formal requirements, and auto-generated to embedded C for integration into a C++ simulator — CI-enabled reproducible workflow."