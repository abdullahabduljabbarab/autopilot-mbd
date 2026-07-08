Reproducible Codegen (CI)
The file generated/regen_code.m programmatically sets model config params and runs slbuild.
Commit generated/METADATA.json and MATLAB_VERSION.txt (or produce them in CI) to track provenance.
Example GitHub Actions job: run MATLAB, run regen_code.m, capture generated artifacts and diffs.
Project Layout
model/: autopilot.slx, plant_params.m, scenarios/
verification/: tests, run_all_tests.m, results/
generated/: generated C files + regen_code.m + metadata
docs/: DESIGN.md, V_AND_V_PLAN.md, PLANT_MODEL_DERIVATION.pdf
integration/: CLEARANCE facade skeleton and tests
Notes
All block numeric values must reference variables in model/plant_params.m.
Replace placeholder aerodynamic derivative citation in plant_params.m before publishing.


---

### E — DESIGN.md (elevated design brief)
Save as docs/DESIGN.md (concise elevated brief)
```markdown
# autopilot-mbd — Elevated Design Brief

Scope Summary
-------------
Model-based design of a cascaded autopilot in MATLAB/Simulink. Deliverables: parameterised Simulink model, plant parameter script, verification suite, reproducible Embedded C codegen, and CLEARANCE C++ integration notes. Includes traceable aerodynamic derivatives, anti-windup, actuator/sensor dynamics, and CI-friendly codegen.

Prereqs & Toolchain
-------------------
MATLAB R2024a+, Simulink, Control System Toolbox, Simulink Control Design, Embedded Coder + MATLAB Coder. Recommend Aerospace Blockset.

Traceable Plant Source & Units
------------------------------
Use a cited textbook (e.g., Stevens & Lewis Appendix A). Units: radians (angles), meters (length), m/s (speed), seconds (time). Document source in docs/PLANT_MODEL_DERIVATION.pdf.

Requirements
------------
REQ-AP-001 ... REQ-AP-013 as defined. Measurement rules: steady-state = mean over last 10 s; settling time uses 2% band with 10 s persistence. Codegen equivalence tolerance default: 1e-4.

Plant Model
-----------
Two decoupled linear models:
- Longitudinal x_long = [u; w; q; theta], A_long and B_long as specified. Altitude h derived via integrator (h_dot ≈ -w for small theta). V = V0 + u.
- Lateral roll-subsidence x_lat = [p; phi; psi], A_lat = [Lp 0 0;1 0 0;0 V0 0], B_lat = [Lda;0;0].

Add optional actuator first-order lag + rate limits and optional sensor LPF + noise.

Controller Architecture
-----------------------
Cascaded outer/inner PID chains for heading, altitude, and airspeed. Mandatory anti-windup (back-calculation) for integrators. Saturations and limits defined in plant_params.m. Inner-loop codegen step: CODEGEN_STEP (default 0.1 s).

Parameter Script
----------------
`model/plant_params.m` is the single authoritative file with all derivatives, gains, saturations, actuator/sensor dynamics, simulation settings, and verification tolerances.

Simulink Model Construction
---------------------------
- model/autopilot.slx, variable-step for interactive sim (ode45), fixed-step for codegen.
- All numeric values reference plant_params.m.
- Autopilot subsystem interface must match CLEARANCE facade.
- Name every signal for Embedded Coder variable mapping.
- Use actuator dynamics and sensor filters between controller and plant.

Controller Tuning
-----------------
Use PID Tuner, rltool, or LQR. Document tuned gains and margins in docs/VERIFICATION_REPORT.md.

Verification Tests
------------------
Eight tests implemented as test_*.m. Each returns name, status, detail, stats and writes PNG + MAT results. Runner collects and writes a JSON/MAT summary for CI.

Code Generation & Reproducibility
--------------------------------
`generated/regen_code.m` automates slbuild and writes `generated/METADATA.json` with MATLAB version, timestamp, and git commit. CI job should run regen_code.m and publish artifacts/diffs.

CLEARANCE Integration
---------------------
Vendor generated C into CLEARANCE/ThirdParty. Provide C++ facade `FClearanceAutopilot` exposing SetCommand, UpdateMeasurements, Step, and Get* methods. Step() called at CODEGEN_STEP frequency from Unreal tick.

Repo Layout
-----------
See README.md for full layout. Key items: model/, verification/, generated/, docs/, integration/.

Deliverables
------------
- Simulink model, plant_params.m, scenarios, tests, regen script, generated C, metadata, and integration facade skeleton.

Extensions
----------
Nonlinear 6-DOF, gain scheduling, yaw damper, turbulence models, Kalman filters, actuator faults, HIL. Each is an extension branch.

Portfolio Bullet
----------------
"Developed a model-based autopilot in MATLAB/Simulink with traceable aerodynamic derivatives, cascaded control with anti-windup and actuator dynamics, verified against formal requirements, and auto-generated to embedded C for integration into a C++ simulator — CI-enabled reproducible workflow."
