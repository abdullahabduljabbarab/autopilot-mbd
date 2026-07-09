
% model/plant_params.m
% Minimal parameter script for autopilot project (executable script)

% Metadata
TIMESTAMP = datestr(now,'yyyy-mm-dd HH:MM:SS');
MATLAB_VERSION = version;

% Basic vehicle constants
V0 = 125;        % Trim speed (m/s)
g  = 9.81;       % Gravity (m/s^2)

% Longitudinal aerodynamic derivatives (example placeholders)
% State ordering convention: [u; w; q; theta]
Xu  = -0.021;
Xw  = 0.122;
Zu  = -0.209;
Zw  = -0.530;
Mu  = 0.0;
Mw  = -0.005;
Mq  = -0.98;

% Control derivatives (elevator, throttle)
Xde = 0.0;
Xdt = 8.0;
Zde = -12.0;
Mde = -8.5;

% Lateral-directional derivatives (example placeholders)
% Lateral state ordering: [p; phi; psi] or as you choose in model
Lp  = -1.5;
Lda = 15.0;

% Actuator dynamics (time constants and rate limits)
TAU_ELEV = 0.05;    % elevator time constant (s)
TAU_AIL  = 0.05;    % aileron time constant (s)
RATE_ELEV = 30*pi/180; % rad/s
RATE_AIL  = 30*pi/180; % rad/s

% Sensor/sample times
TS_SENSOR = 0.02;  % s

% Saturation limits
PHI_LIMIT_RAD   = 25*pi/180;
THETA_LIMIT_RAD = 10*pi/180;
DELTA_T_MIN = 0.0;
DELTA_T_MAX = 1.0;

% Lowercase aliases used by the Simulink saturation blocks.
% Elevator (delta_e) - deflection in radians, +/- 25 deg travel.
delta_e_max = 25*pi/180;
delta_e_min = -25*pi/180;
% Aileron (delta_a) - deflection in radians, +/- 25 deg travel.
delta_a_max = 25*pi/180;
delta_a_min = -25*pi/180;
% Throttle (delta_t) - normalised 0..1.
delta_t_min = 0.0;
delta_t_max = 1.0;
% Rudder (delta_r) - not currently used but declared for completeness.
delta_r_max = 25*pi/180;
delta_r_min = -25*pi/180;

% Controller initial gains (placeholders)
K_hdg = 1.5;
K_phi = 3.0;
K_p   = 0.5;
K_alt_p = 0.001;
K_alt_i = 0.0002;
K_theta = 2.0;
K_q = 0.4;

% -- Inner-loop PID gains referenced by the Simulink PID blocks.
% Retuned for a pure-integrator plant (bank rate = aileron directly,
% no roll damping) which is how the downstream CLEARANCE aircraft
% dynamics behave. On a pure integrator, P-only with tiny D is stable;
% integral action would build up and cause limit-cycling. - TripleA
% Pitch (theta) PID -> elevator
Kp_theta = 0.5;
Ki_theta = 0.0;
Kd_theta = 0.05;
% Bank (phi) PID -> aileron
Kp_phi = 0.5;
Ki_phi = 0.0;
Kd_phi = 0.05;
% Airspeed (V) PID -> throttle (kept - speed plant is well-damped)
Kp_V = 0.05;
Ki_V = 0.02;
Kd_V = 0.0;

% Anti-windup/backcalc parameters
ALT_INT_BACKCALC = 0.2;
IAS_INT_BACKCALC = 0.2;

% Simulation and codegen settings
MAX_STEP_VARSTEP = 0.05;
CODEGEN_STEP = 0.1;
SIM_STOP_TIME = 120;

% Verification tolerances
TOL_CODEGEN_EQ = 1e-4;
TOL_CHANNEL.psi   = deg2rad(1e-3);
TOL_CHANNEL.phi   = deg2rad(1e-3);
TOL_CHANNEL.theta = deg2rad(1e-3);
TOL_CHANNEL.h     = 0.01;
TOL_CHANNEL.V     = 1e-3;
TOL_CHANNEL.delta = 1e-4;

% Convenience conversions
KT_TO_MPS = 0.514444;
MPS_TO_KT = 1/KT_TO_MPS;
PHI_LIMIT_DEG = PHI_LIMIT_RAD*180/pi;
THETA_LIMIT_DEG = THETA_LIMIT_RAD*180/pi;

% Logging flag
LOG_TO_WORKSPACE = true;
VERBOSE = true;

% Print summary if verbose
if VERBOSE
    fprintf('Loaded plant_params: V0=%.1f m/s, CODEGEN_STEP=%.3f s, PHI_LIMIT=%.1f deg\n', ...
        V0, CODEGEN_STEP, PHI_LIMIT_DEG);
end

% Export minimal metadata to base workspace
AUTOPILOT_METADATA.timestamp = TIMESTAMP;
AUTOPILOT_METADATA.MATLAB_VERSION = MATLAB_VERSION;
assignin('base','AUTOPILOT_METADATA',AUTOPILOT_METADATA);