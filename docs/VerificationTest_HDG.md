function result = test_hdg_step()
% TEST_HDG_STEP Run heading-step verification (REQ-AP-001, REQ-AP-003)
% Returns struct with fields: name, status ('PASS'/'FAIL'), detail, stats.

name = 'TEST-AP-HDG-STEP';
% Load params and model
run(fullfile('model','plant_params.m'));
model = 'model/autopilot';
load_system([model '.slx']);

% Prepare scenario: psi_cmd step 0->90 deg at t=5
simTime = 60;
psi_step_time = 5;
psi_final = deg2rad(90);
t = [0 psi_step_time simTime];
psi_cmd = [0 psi_final psi_final];

% Create timeseries for Signal Editor replacement (psi_cmd only)
ts_psi = timeseries(psi_cmd, t);

% Set model stop time and solver
set_param(model, 'StopTime', num2str(simTime));
set_param(model, 'Solver', 'ode45', 'MaxStep', num2str(MAX_STEP_VARSTEP));

% Assign to base workspace variable expected by model (name your signal input accordingly)
assignin('base','psi_cmd_ts', ts_psi);

% Run sim (assumes top-level model maps psi_cmd_ts into psi_cmd input)
out = sim(model, 'ReturnWorkspaceOutputs', 'on');

% Extract signals (adapt names if your model uses different signal names)
% Expect recorded time series variables in out or To Workspace logs. Try common patterns:
try
    psi = out.logsout.getElement('psi').Values.Data;
    time = out.logsout.getElement('psi').Values.Time;
    psi_cmd_log = out.logsout.getElement('psi_cmd').Values.Data;
catch
    % Fallback: attempt to read workspace variables created by To Workspace blocks
    if exist('psi','var') && exist('psi_cmd','var')
        psi = evalin('base','psi');
        psi_cmd_log = evalin('base','psi_cmd');
        time = (0:CODEGEN_STEP:simTime)';
    else
        result.name = name;
        result.status = 'FAIL';
        result.detail = 'Could not locate output signals. Ensure model logs psi and psi_cmd.';
        return;
    end
end

% Compute error
err = psi_cmd_log - psi;
% Steady-state metric: mean over last 10 s
idx_ss = time >= (simTime - 10);
ss_err = mean(err(idx_ss));
ss_err_deg = rad2deg(abs(ss_err));

% Settling time (2% band relative to final psi_cmd)
final = psi_cmd_log(end);
tol_band = 0.02*abs(final);
within = abs(err) <= tol_band;
% Require 10 s persistence
settle_time = NaN;
for i=1:length(time)
    if time(i) < psi_step_time, continue; end
    % find window of 10s after current time
    idx_window = time >= time(i) & time <= (time(i)+10);
    if all(abs(err(idx_window)) <= tol_band)
        settle_time = time(i) - psi_step_time;
        break;
    end
end

% Pass criteria
pass_ss = abs(ss_err) <= deg2rad(1); % 1 deg
pass_settle = ~isnan(settle_time) && (settle_time <= 30);

if pass_ss && pass_settle
    status = 'PASS';
else
    status = 'FAIL';
end

% Save plot
fig = figure('Visible','off');
plot(time, rad2deg(psi_cmd_log), '--k', 'DisplayName','psi\_cmd');
hold on;
plot(time, rad2deg(psi), 'b', 'DisplayName','psi');
xlabel('Time (s)');
ylabel('Heading (deg)');
title('TEST-AP-HDG-STEP');
legend('Location','best');
grid on;
outdir = fullfile('verification','results');
if ~exist(outdir,'dir'), mkdir(outdir); end
pngname = fullfile(outdir, ['hdg_step_' datestr(now,'yyyymmdd_HHMMSS') '.png']);
saveas(fig, pngname);
close(fig);

% Populate result
result.name = name;
result.status = status;
result.detail = sprintf('ss_err_deg=%.3f deg; settle_time=%.2f s', ss_err_deg, settle_time);
result.stats.ss_error_rad = ss_err;
result.stats.ss_error_deg = ss_err_deg;
result.stats.settle_time_s = settle_time;
result.artifact.plot = pngname;

end
