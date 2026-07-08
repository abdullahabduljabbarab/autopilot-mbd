### I'll start with A — the full upgraded `plant_params.m`

Below is the complete, elevated `model/plant_params.m`. Save it as `model/plant_params.m` in your repo and set the model PreLoadFcn to run it (see file header).

```matlab
% model/plant_params.m
% Authoritative parameter file for autopilot-mbd
% Replace example aerodynamic derivatives with cited source before publishing.
% Example source placeholder: Stevens & Lewis, Aircraft Control and Simulation, App. A
%
% Usage:
%  run(fullfile(fileparts(mfilename('fullpath')),'plant_params.m'));
%
% This file exposes all tunable parameters referenced by autopilot.slx:
%  - aerodynamic derivatives
%  - controller gains
%  - actuator/sensor dynamics
%  - saturation limits
%  - simulation and verification tolerances
%  It also prints a short provenance summary.

%% Metadata
REPO_ROOT = fileparts(fileparts(mfilename('fullpath')));
TIMESTAMP = datestr(now,'yyyy-mm-dd HH:MM:SS');
MATLAB_VERSION = version;
fprintf('Loading autopilot-mbd parameters — %s (MATLAB %s)\n', TIMESTAMP, MATLAB_VERSION);

%% --- Source / citation (replace with the real citation you choose)
% PLANT_DERIV_SOURCE = 'Stevens & Lewis, Aircraft Control and Simulation, Appendix A (example)';
PLANT_DERIV_SOURCE = 'REPLACE_WITH_CHOSEN_REFERENCE_AND_PAGE';

%% --- Units / Conventions
% Angles: radians internally
% Length: meters
% Speed: m/s (1 kt = 0.514444 m/s)
% Time: seconds

%% --- Trim and constants
V0 = 125;           % trim airspeed (m/s)
g  = 9.81;          % gravity (m/s^2)

%% --- Longitudinal aerodynamic derivatives (example values; cite source)
% State ordering: x_long = [u; w; q; theta]
Xu  = -0.021;
Xw  =  0.122;
Zu  = -0.209;
Zw  = -0.530;
Mu  =  0.0;
Mw  = -0.005;
Mq  = -0.980;

% Control derivatives (delta_e elevator, delta_t throttle)
Xde =  0.0;
Xdt =  8.0;
Zde = -12.0;
Mde = -8.5;

% Note: these are example numbers. Replace with values from PLANT_DERIV_SOURCE.

%% --- Lateral aerodynamic derivatives (roll-subsidence approx)
% Lateral state ordering for State-Space block: x_lat = [p; phi; psi]
Lp  = -1.5;    % roll damping
Lda =  15.0;   % aileron control power

%% --- Actuator dynamics (optional, recommended)
% First-order actuator lag time constants (seconds)
TAU_ELEV = 0.05;
TAU_AIL  = 0.05;

% Actuator rate limits (rad/s)
RATE_ELEV = 30 * pi/180;   % 30 deg/s
RATE_AIL  = 30 * pi/180;   % 30 deg/s

%% --- Sensor dynamics / noise (optional)
TS_SENSOR = 0.02;          % sensor LPF time constant (s) for attitude/heading
SENSOR_NOISE_STD.theta = 0.0;   % (rad)
SENSOR_NOISE_STD.phi   = 0.0;   % (rad)
SENSOR_NOISE_STD.psi   = 0.0;   % (rad)
SENSOR_NOISE_STD.h     = 0.0;   % (m)
SENSOR_NOISE_STD.V     = 0.0;   % (m/s)

%% --- Saturation limits (all referenced by name in Simulink)
PHI_LIMIT_RAD   = 25 * pi/180;   % max bank angle (rad)
THETA_LIMIT_RAD = 10 * pi/180;   % max pitch angle (rad)
DELTA_T_MIN     = 0.0;
DELTA_T_MAX     = 1.0;

%% --- Controller gains (initial values — tune these)
% Heading / lateral chain
K_hdg = 1.5;   % heading -> phi_cmd (rad/rad)
K_phi = 3.0;   % phi_err -> p_cmd
K_p   = 0.5;   % p_err -> delta_a

% Altitude / longitudinal chain
K_alt_p = 0.001;    % altitude P (m^-1)
K_alt_i = 0.0002;   % altitude I
K_theta = 2.0;      % theta_err -> q_cmd
K_q     = 0.4;      % q_err -> delta_e

% Airspeed (throttle) PID
K_v_p = 0.05;
K_v_i = 0.005;
K_v_d = 0.01;

%% --- Anti-windup / PID back-calculation
% Use these for PID block anti-windup/back-calculation configuration
ALT_INT_BACKCALC = 0.2;   % back-calculation gain for altitude integrator
IAS_INT_BACKCALC = 0.2;   % back-calculation gain for airspeed integrator

%% --- Sampling / codegen settings
% Default variable-step max step (for interactive sim)
MAX_STEP_VARSTEP = 0.05;

% Fixed-step for code generation and runtime (s). Use this when building ert.tlc.
CODEGEN_STEP = 0.1;   % 10 Hz inner-loop (example)

% Model stop time default
SIM_STOP_TIME = 120;

%% --- Verification tolerances (adjustable)
% Codegen equivalence tolerance (absolute)
TOL_CODEGEN_EQ = 1e-4;    % absolute tolerance between Simulink ref and generated C outputs

% Per-channel tolerances (example; can be tightened per test)
TOL_CHANNEL.psi   = deg2rad(1e-3);   % heading (rad)
TOL_CHANNEL.phi   = deg2rad(1e-3);   % bank (rad)
TOL_CHANNEL.theta = deg2rad(1e-3);   % pitch (rad)
TOL_CHANNEL.h     = 0.01;            % altitude (m)
TOL_CHANNEL.V     = 1e-3;            % speed (m/s)
TOL_CHANNEL.delta = 1e-4;           % control outputs (rad or unitless for throttle)

%% --- Limits and numeric checks
% Derived limits for convenience
PHI_LIMIT_DEG = PHI_LIMIT_RAD * 180/pi;
THETA_LIMIT_DEG = THETA_LIMIT_RAD * 180/pi;

%% --- Helper conversion constants
KT_TO_MPS = 0.514444;
MPS_TO_KT = 1/KT_TO_MPS;

%% --- Logging / debug flags
LOG_TO_WORKSPACE = true;   % when true, tests use To Workspace or To File blocks
VERBOSE = true;

%% --- Print summary
if VERBOSE
    fprintf('Parameters summary:\n');
    fprintf('  Trim V0 = %.1f m/s; PHI_LIMIT = %.1f deg; THETA_LIMIT = %.1f deg\n', ...
        V0, PHI_LIMIT_DEG, THETA_LIMIT_DEG);
    fprintf('  CODEGEN_STEP = %.3f s; MAX_STEP_VARSTEP = %.3f s\n', CODEGEN_STEP, MAX_STEP_VARSTEP);
    fprintf('  TOL_CODEGEN_EQ = %g\n', TOL_CODEGEN_EQ);
    fprintf('  Plant deriv source: %s\n', PLANT_DERIV_SOURCE);
end

%% --- Sanity checks
if DELTA_T_MIN >= DELTA_T_MAX
    error('plant_params:ThrottleLimits', 'DELTA_T_MIN must be less than DELTA_T_MAX.');
end
if CODEGEN_STEP <= 0
    error('plant_params:CodegenStep', 'CODEGEN_STEP must be positive.');
end

%% --- Save metadata for reproducibility (lightweight)
METADATA.MATLAB_VERSION = MATLAB_VERSION;
METADATA.timestamp = TIMESTAMP;
METADATA.plant_source = PLANT_DERIV_SOURCE;
METADATA.V0 = V0;
% Write metadata file to generated/ when regen script runs; we keep in workspace here.
assignin('base','AUTOPILOT_METADATA',METADATA);

% End of plant_params.m
```
