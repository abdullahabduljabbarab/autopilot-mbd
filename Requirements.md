# Requirements — autopilot-mbd

Every requirement covered by the Simulink model, grouped by control loop and traced to (a) the specific model block that implements it and (b) the external source the requirement derives from. Each REQ-ID is also tagged in [`req_map.csv`](req_map.csv) (the machine-readable form used by the traceability report). This doc adds the Source column that `req_map.csv` doesn't carry.

Companion to [`docs/V_AND_V_PLAN.md`](docs/V_AND_V_PLAN.md) which is the verification strategy behind proving each requirement.

## Numbering scheme

```
REQ-AP-<###>
```

Numbers ascend and are never reused. Deprecated REQ-IDs stay in place with a `[DEPRECATED]` marker rather than being renumbered.

## REQ-AP-001..003 — Bank hold controller

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-AP-001 | Bank hold controller shall drive aileron from `phi_cmd - phi` error | `autopilot/AutopilotSubsystem/PID_phi` | Åström & Murray, *Feedback Systems* (2020), ch. 10 — PID design pattern for attitude hold |
| REQ-AP-002 | Bank error shall be computed as `phi_cmd - phi` at `Sum_phi` | `autopilot/AutopilotSubsystem/Sum_phi` | Standard negative-feedback control topology |
| REQ-AP-003 | Aileron command shall be saturated to ±25° | `autopilot/AutopilotSubsystem/Sat_delta_a` | EASA CS-25.671(c)(3) control-surface travel envelope; transport-category airliner |

## REQ-AP-004..006 — Pitch hold controller

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-AP-004 | Pitch hold controller shall drive elevator from `theta_cmd - theta` error | `autopilot/AutopilotSubsystem/PID_theta` | Åström & Murray, *Feedback Systems* (2020), ch. 10 |
| REQ-AP-005 | Pitch error shall be computed as `theta_cmd - theta` at `Sum_theta` | `autopilot/AutopilotSubsystem/Sum_theta` | Standard negative-feedback topology |
| REQ-AP-006 | Elevator command shall be saturated to ±25° | `autopilot/AutopilotSubsystem/Sat_theta` | EASA CS-25.671(c)(3) |

## REQ-AP-007..009 — Airspeed hold controller

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-AP-007 | Airspeed hold controller shall drive throttle from `V_cmd - V` error | `autopilot/AutopilotSubsystem/PID_V` | Franklin, Powell & Emami-Naeini, *Feedback Control of Dynamic Systems*, PI on first-order thrust plant |
| REQ-AP-008 | Airspeed error shall be computed as `V_cmd - V` at `Sum_V` | `autopilot/AutopilotSubsystem/Sum_V` | Standard negative-feedback topology |
| REQ-AP-009 | Throttle command shall be saturated to the range 0..1 | `autopilot/AutopilotSubsystem/Sat_V` | Normalised throttle convention across the model; ICAO Doc 9760 fuel-flow envelope |

## REQ-AP-010..011 — Actuator dynamics

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-AP-010 | Aileron actuator dynamics shall be modelled as a first-order lag `1/(0.05s+1)` | `autopilot/ail_act` | Representative 50 ms actuator time constant, transport-category airliner; Roskam, *Airplane Flight Dynamics and Automatic Flight Controls* Part II, actuator dynamics for airliner-class hydraulic servos |
| REQ-AP-011 | Elevator actuator dynamics shall be modelled as a first-order lag `1/(0.05s+1)` | `autopilot/elev_act` | Same as REQ-AP-010 |

## REQ-AP-012..014 — Root inport contracts

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-AP-012 | Root inport `phi_cmd` shall accept commanded bank angle in radians | `autopilot/phi_cmd` | CLEARANCE integration contract — outer heading loop produces `phi_cmd` in radians |
| REQ-AP-013 | Root inport `theta_cmd` shall accept commanded pitch angle in radians | `autopilot/theta_cmd` | CLEARANCE integration contract — outer altitude loop produces `theta_cmd` in radians |
| REQ-AP-014 | Root inport `V_cmd` shall accept commanded airspeed in metres per second | `autopilot/V_cmd` | CLEARANCE integration contract — ATC command converted to m/s for SI unit consistency |

## REQ-AP-015..017 — Root outport contracts

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-AP-015 | Root outport `delta_a_out` shall expose the aileron command in radians | `autopilot/delta_a_out` | CLEARANCE integration contract — treated as roll-rate command inside the sim |
| REQ-AP-016 | Root outport `delta_e_out` shall expose the elevator command in radians | `autopilot/delta_e_out` | CLEARANCE integration contract — treated as pitch-rate command inside the sim |
| REQ-AP-017 | Root outport `delta_t_out` shall expose the throttle command in the range 0..1 | `autopilot/delta_t_out` | CLEARANCE integration contract — normalised throttle |

## Coverage summary

| Loop | REQs | Verification tier |
|---|---:|---|
| Bank hold | 3 | T1 probe + T2 baseline regression + T3 CLEARANCE integration |
| Pitch hold | 3 | T1 probe + T2 baseline regression + T3 CLEARANCE integration |
| Airspeed hold | 3 | T1 probe + T2 baseline regression + T3 CLEARANCE integration |
| Actuator dynamics | 2 | T2 baseline regression |
| Root inport contracts | 3 | T2 baseline regression + T3 wrapper-side type check |
| Root outport contracts | 3 | T2 baseline regression + T3 wrapper-side type check |
| **Total** | **17** | |

## Design decisions not captured as REQs

Some design choices are deliberately not requirements because they're implementation tuning that can change without affecting the external contract:

- **`Ki = 0` and `Kd = 0` on attitude PIDs.** Tuning choice driven by pure-integrator plant assumption and the RK4 solver-stability finding (see the README's "Solver-stability finding" section). If the plant model changed, these could be re-tuned without any REQ change.
- **Specific gain values in `model/plant_params.m`.** Same reasoning — tunable within the envelope defined by the REQs above.
- **50 Hz fixed-step (h = 0.02 s), ode4 solver.** Codegen configuration; changeable if a different target platform needs a different rate.

## Adding a new REQ-AP-*

1. Add a probe or extend an existing verification block in the model.
2. Append a row to `req_map.csv` with the next available ID, the block path, and the description.
3. Add a row here in the appropriate loop section with the Source citation.
4. Regenerate `traceability_report.html` via `slreq.generateReport` so CI covers it.

The convention is intentionally lightweight. Heavier process wouldn't survive portfolio-project cadence, and the verification discipline is proven by the test-tier + probe structure, not by the doc's typography.

## References cited in the Source column

- **Åström, K. J. & Murray, R. M.**, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2nd ed. 2020. Free at https://fbswiki.org.
- **Franklin, G. F., Powell, J. D. & Emami-Naeini, A.**, *Feedback Control of Dynamic Systems*, Pearson, 8th ed. 2019.
- **Roskam, J.**, *Airplane Flight Dynamics and Automatic Flight Controls, Part II*, DARcorporation, 2003. Actuator dynamics reference for transport-class airliners.
- **EASA CS-25.671** (Certification Specifications for Large Aeroplanes), Control Systems General. Section 671(c)(3) covers control-surface travel envelopes.
- **ICAO Doc 9760** Airworthiness Manual, Volume II. Referenced for throttle-envelope conventions.
