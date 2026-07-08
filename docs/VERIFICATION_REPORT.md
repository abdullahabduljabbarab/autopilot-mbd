# Verification Report — autopilot-mbd

Run date: REPLACE_WITH_DATE

Summary
-------
- Total tests: 8
- Passed: X
- Failed: Y

Artifacts
---------
- verification/results/*.png
- verification/results/*.mat
- generated/METADATA.json

Per-Test Summary
----------------
TEST-AP-HDG-STEP
- Status: PASS/FAIL
- ss_error_deg: ...
- settling_time_s: ...
- Plot: verification/results/hdg_step_YYYYMMDD_HHMMSS.png

TEST-AP-BANK-LIMIT
- Status:
- max_phi_deg:
- Plot:

... (repeat for each test)

Bode / Margins
--------------
- Pitch inner-loop:
  - Phase margin: XX deg
  - Gain margin: YY dB
  - Bode plot: verification/bode.png

Codegen Equivalence
-------------------
- Max abs error per channel:
  - psi: ...
  - phi: ...
  - theta: ...
  - delta_a: ...
  - delta_e: ...
  - delta_t: ...
- Tolerance: TOL_CODEGEN_EQ = ...
- Result: PASS/FAIL

Conclusions
-----------
Short summary and recommended follow-ups (tuning, actuator dynamics adjustments, CI regen).

Signed-off-by: <Your Name>
