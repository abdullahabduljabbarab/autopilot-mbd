# autopilot-mbd

Model-based design of an aircraft autopilot in MATLAB/Simulink. Parameterised Simulink model, verification suite, and reproducible Embedded C codegen for integration into the CLEARANCE simulator.

## Requirements
- MATLAB R2024a+ with Simulink
- Control System Toolbox, Simulink Control Design
- Embedded Coder + MATLAB Coder (for codegen)
- (Recommended) Aerospace Blockset

## Quickstart
1. Open MATLAB, set repo root on path.
2. Load parameters:
   ```matlab
   run(fullfile('model','plant_params.m'));


Open model:

open('model/autopilot.slx');
Run verification (from repo root):

cd verification
results = run_all_tests;
Regenerate C code:

cd generated
run('regen_code.m');