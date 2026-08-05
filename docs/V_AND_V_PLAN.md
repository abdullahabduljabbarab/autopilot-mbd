# Verification and Validation Plan: autopilot-mbd

Companion to [`../req_map.csv`](../req_map.csv) (the source-of-truth requirement-to-block mapping) and [`../traceability_report.html`](../traceability_report.html) (the rendered coverage matrix). If `req_map.csv` answers "what is the model supposed to do?", this doc answers "how do we prove it does?".

## 1. Purpose and scope

Verification and validation on a Model-Based Design project is a proportionality exercise. This is a portfolio-scale Simulink model of a cascade autopilot, not a DAL-A flight-critical control law. So it does not need DO-178C / DO-331 tool qualification rigour, but it does need to demonstrate the discipline that a real defence programme would exhibit: traceable requirements, tiered verification, and evidence a reviewer can audit without running MATLAB.

### In scope

| Item | Notes |
|---|---|
| Every requirement in `req_map.csv` | 17 REQ-AP-* entries across bank / pitch / airspeed loops, actuator dynamics, and root port contracts |
| Every block, subsystem, and root port cited by a requirement | The `Block` column of the CSV is grep-able against the model and against the `verify_*.m` scripts |
| Fixed-step solver stability against the ODE4 step-size / derivative-filter cut-off product | Called out explicitly because it drove the `Kd = 0` finding on the attitude PIDs: see [`AUTOPILOT_MBD_DESIGN.md`](AUTOPILOT_MBD_DESIGN.md) |
| Embedded-Coder-generated C's structural equivalence to the model | Regression check via `tools/compare_sim.m` comparing generated-code sim vs `sim('autopilot')` reference |
| Reusable-function packaging isolation between instances | Verified informally through CLEARANCE integration (fleet of aircraft, no shared-state bugs); no automated multi-instance test in this repo |

### Out of scope

| Item | Reason |
|---|---|
| DO-178C / DO-331 model coverage (MC/DC, statement, decision) | Portfolio scale. If this shipped in a certified target, MathWorks Simulink Coverage would run under a qualified tool chain and generate the coverage report. |
| Hardware-in-the-loop testing | No target hardware; CLEARANCE integration IS the loop. |
| Adversarial robustness (out-of-envelope inputs, NaN/Inf poisoning) | Bounds and saturation are modelled; adversarial tests not automated. Would be added for a real programme. |
| Formal methods (bounded model checking, theorem-proving) | Not portfolio scale. |

## 2. Test tiers

Three tiers, each covering different requirement classes at different cost.

| Tier | Definition | Cost | Where they live | When to use |
|---|---|---|---|---|
| **T1 Simulink Test / probe verification** | Model-in-the-loop tests using `verify_*.m` scripts that instrument the block outputs via probes, run `sim(model)`, and assert numeric properties (setpoint tracking within tolerance, saturation active on out-of-range command, actuator lag settling within N tau). | Low. Sub-second per test on a warm session. | `tools/verify_*.m` scripts, referenced from `req_map.csv`. | Any pure-model requirement: control-law behaviour, saturation, actuator dynamics, root-port units. |
| **T2 Regression against baseline** | `tools/compare_sim.m` loads `ci_artifacts/simOut_with_pid_defaults.mat` (the frozen reference), reruns the model, and asserts each signal is within tolerance of the baseline. Catches accidental drift from any block-config change. | Low-medium. Requires a fresh Simulink session but runs headless. | `tools/compare_sim.m`, `tools/run_model_tests_and_build.m` (CI entry point). | Whole-model integrity checks, especially after any tuning parameter or block reconfiguration. |
| **T3 Code-generation + integration** | `rtwbuild('autopilot')` generates C, CLEARANCE builds the wrapper module, aircraft flies under the generated code in-sim. Any behavioural regression versus pure-model sim is caught by pilot-side observation (wing rock, altitude-hold drift, throttle chatter). | High. Requires full toolchain and CLEARANCE build. | `AUTOPILOT_MBD_DESIGN.md` describes the CLEARANCE integration path; `.github/workflows/ci.yml` covers the code-generation half. | Every release. Every time PID gains or actuator constants change. |

### Selection rule

Default to T1. Escalate to T2 when a model-wide integrity concern applies (baseline drift, config change). T3 always runs before a release; the `Kd = 0` finding on the attitude PIDs was discovered at T3 because the fixed-step-solver instability was not visible in T1 pure-model sim.

## 3. Traceability

`req_map.csv` is the traceability matrix. Every row maps one REQ-AP-* to one Simulink block or MATLAB kernel. `traceability_report.html` renders it with hyperlinks; `traceability_report.csv` is the machine-readable form used by CI to fail on missing coverage.

### Coverage discipline

| Rule | How it is enforced |
|---|---|
| Every REQ-AP-* must have exactly one Block cited | `req_map.csv` schema; CI script asserts no orphan REQ-IDs |
| Every Block cited must exist in the current model | `tools/run_model_tests_and_build.m` opens the model and resolves each block path; missing block fails the build |
| Every Block that a T1 probe reads must be reachable | Simulink Test assertions catch a probed-but-missing block as a test failure |

### Currently green

- 17 of 17 REQ-AP-* entries mapped
- 17 of 17 mapped blocks resolve in the current `autopilot.slx`
- Regression check passes against the frozen baseline in `ci_artifacts/simOut_with_pid_defaults.mat`
- CLEARANCE integration flies clean with `Kd = 0` on the attitude loops (verified in-sim after the solver-stability finding)

## 4. Coverage targets

Self-imposed discipline goals, not regulatory obligations.

| # | Target | Rule | Current status |
|---|---|---|---|
| 1 | REQ-AP coverage | Every requirement has at least one T1 verification probe or a T2 regression signal. | 17 of 17. **Target met.** |
| 2 | Solver stability | Any continuous-time filter with a fixed-step-solver eigenvalue must satisfy `h · N < 2.78` (RK4 stability limit for real-negative eigenvalues). | Attitude PIDs `Kd = 0`; airspeed PID retains derivative because its filter cut-off sits inside the stability region. **Target met.** |
| 3 | Code-generation equivalence | Generated C sim output must match pure-model sim within regression tolerance. | Verified by `tools/compare_sim.m`. **Target met.** |
| 4 | Reusable-function isolation | Fleet of concurrent instances must not share state via file-scope globals. | Code-interface packaging = **Reusable function**; verified by inspection of generated `.c`. **Target met.** |

## 5. When to run what

| Trigger | T1 | T2 | T3 (code-gen + CLEARANCE integration) |
|---|:-:|:-:|:-:|
| Any block edit in `autopilot.slx` | ✓ | ✓ | |
| Any tuning parameter change in `model/plant_params.m` | ✓ | ✓ | ✓ |
| Any actuator dynamics change | ✓ | ✓ | ✓ |
| MATLAB / Simulink version upgrade | ✓ | ✓ | ✓ |
| Before shipping to CLEARANCE | ✓ | ✓ | ✓ |
| Before recording a demo video | ✓ | ✓ | ✓ |

## 6. Change control

`req_map.csv` and this doc live with the model in the same repo. Changes to requirements are committed alongside the model change that motivates them.

- **New REQ-AP-***: append to `req_map.csv` with a stable ID, add a `verify_*.m` probe or extend an existing one.
- **Removing a REQ-AP-***: mark the row with a `[DEPRECATED]` suffix in the Description column; don't reuse the ID.
- **Changing a REQ-AP text**: increment the description; the ID stays; the verification probe must still cover the new intent.

## 7. What this doc deliberately doesn't cover

- **Formal certification artefacts** (DO-178C model coverage, DO-331 tool qualification, hazard analysis). Not portfolio scale.
- **Hardware-in-the-loop testing**. No target hardware.
- **Independent verification by a separate team**. Solo portfolio project; the closest thing to independent verification is the CLEARANCE integration exercising the generated code in a different codebase.

If this autopilot were shipping into a certified flight programme, every bullet above would need to be addressed. Documenting what's not done makes the current scope honest rather than pretending everything's covered.
