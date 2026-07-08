
sys = 'autopilot/AutopilotSubsystem';

% Clean existing contents (remove any child blocks if present)
try
    children = find_system(sys,'SearchDepth',1);
    % skip the subsystem itself (first entry)
    for i = 2:numel(children)
        try delete_block(children{i}); catch, end
    end
catch
end

% Create Inports (setpoint and measurement for each loop)
try delete_block([sys '/phi_cmd']); catch, end
add_block('simulink/Sources/In1',[sys '/phi_cmd'],'Position',[30 30 60 50]);
try delete_block([sys '/phi']); catch, end
add_block('simulink/Sources/In1',[sys '/phi'],'Position',[30 90 60 110]);

try delete_block([sys '/theta_cmd']); catch, end
add_block('simulink/Sources/In1',[sys '/theta_cmd'],'Position',[30 190 60 210]);
try delete_block([sys '/theta']); catch, end
add_block('simulink/Sources/In1',[sys '/theta'],'Position',[30 250 60 270]);

try delete_block([sys '/V_cmd']); catch, end
add_block('simulink/Sources/In1',[sys '/V_cmd'],'Position',[30 350 60 370]);
try delete_block([sys '/V']); catch, end
add_block('simulink/Sources/In1',[sys '/V'],'Position',[30 410 60 430]);

% Create Sum blocks (setpoint - measurement)
try delete_block([sys '/Sum_phi']); catch, end
add_block('simulink/Math Operations/Sum',[sys '/Sum_phi'],'Position',[110 40 140 80]);
set_param([sys '/Sum_phi'],'Inputs','+-');

try delete_block([sys '/Sum_theta']); catch, end
add_block('simulink/Math Operations/Sum',[sys '/Sum_theta'],'Position',[110 200 140 240]);
set_param([sys '/Sum_theta'],'Inputs','+-');

try delete_block([sys '/Sum_V']); catch, end
add_block('simulink/Math Operations/Sum',[sys '/Sum_V'],'Position',[110 360 140 400]);
set_param([sys '/Sum_V'],'Inputs','+-');

% Create PID Controller blocks (continuous) — use workspace params: Kp_*, Ki_*, Kd_*
try delete_block([sys '/PID_phi']); catch, end
add_block('simulink/Continuous/PID Controller',[sys '/PID_phi'],'Position',[190 30 320 90]);
set_param([sys '/PID_phi'], 'P','Kp_phi','I','Ki_phi','D','Kd_phi');

try delete_block([sys '/PID_theta']); catch, end
add_block('simulink/Continuous/PID Controller',[sys '/PID_theta'],'Position',[190 190 320 250]);
set_param([sys '/PID_theta'], 'P','Kp_theta','I','Ki_theta','D','Kd_theta');

try delete_block([sys '/PID_V']); catch, end
add_block('simulink/Continuous/PID Controller',[sys '/PID_V'],'Position',[190 350 320 410]);
set_param([sys '/PID_V'], 'P','Kp_V','I','Ki_V','D','Kd_V');

% Create Saturation blocks (limits from workspace: delta_*_min, delta_*_max)
try delete_block([sys '/Sat_delta_a']); catch, end
add_block('simulink/Discontinuities/Saturation',[sys '/Sat_delta_a'],'Position',[380 30 440 90]);
set_param([sys '/Sat_delta_a'],'UpperLimit','delta_a_max','LowerLimit','delta_a_min');

try delete_block([sys '/Sat_delta_e']); catch, end
add_block('simulink/Discontinuities/Saturation',[sys '/Sat_delta_e'],'Position',[380 190 440 250]);
set_param([sys '/Sat_delta_e'],'UpperLimit','delta_e_max','LowerLimit','delta_e_min');

try delete_block([sys '/Sat_delta_t']); catch, end
add_block('simulink/Discontinuities/Saturation',[sys '/Sat_delta_t'],'Position',[380 350 440 410]);
set_param([sys '/Sat_delta_t'],'UpperLimit','delta_t_max','LowerLimit','delta_t_min');

% Create Outports for actuator commands
try delete_block([sys '/delta_a']); catch, end
add_block('simulink/Sinks/Out1',[sys '/delta_a'],'Position',[520 40 560 60]);
try delete_block([sys '/delta_e']); catch, end
add_block('simulink/Sinks/Out1',[sys '/delta_e'],'Position',[520 200 560 220]);
try delete_block([sys '/delta_t']); catch, end
add_block('simulink/Sinks/Out1',[sys '/delta_t'],'Position',[520 360 560 380]);

% Connect setpoints and measurements to sums
% remove existing lines if any (safe)
try delete_line(sys,'phi_cmd/1','Sum_phi/1'); catch, end
try delete_line(sys,'phi/1','Sum_phi/2'); catch, end
add_line(sys,'phi_cmd/1','Sum_phi/1','autorouting','on');
add_line(sys,'phi/1','Sum_phi/2','autorouting','on');

try delete_line(sys,'theta_cmd/1','Sum_theta/1'); catch, end
try delete_line(sys,'theta/1','Sum_theta/2'); catch, end
add_line(sys,'theta_cmd/1','Sum_theta/1','autorouting','on');
add_line(sys,'theta/1','Sum_theta/2','autorouting','on');

try delete_line(sys,'V_cmd/1','Sum_V/1'); catch, end
try delete_line(sys,'V/1','Sum_V/2'); catch, end
add_line(sys,'V_cmd/1','Sum_V/1','autorouting','on');
add_line(sys,'V/1','Sum_V/2','autorouting','on');

% Connect Sum -> PID -> Saturation -> Outport (remove existing lines first)
try delete_line(sys,'Sum_phi/1','PID_phi/1'); catch, end
try delete_line(sys,'PID_phi/1','Sat_delta_a/1'); catch, end
try delete_line(sys,'Sat_delta_a/1','delta_a/1'); catch, end
add_line(sys,'Sum_phi/1','PID_phi/1','autorouting','on');
add_line(sys,'PID_phi/1','Sat_delta_a/1','autorouting','on');
add_line(sys,'Sat_delta_a/1','delta_a/1','autorouting','on');

try delete_line(sys,'Sum_theta/1','PID_theta/1'); catch, end
try delete_line(sys,'PID_theta/1','Sat_delta_e/1'); catch, end
try delete_line(sys,'Sat_delta_e/1','delta_e/1'); catch, end
add_line(sys,'Sum_theta/1','PID_theta/1','autorouting','on');
add_line(sys,'PID_theta/1','Sat_delta_e/1','autorouting','on');
add_line(sys,'Sat_delta_e/1','delta_e/1','autorouting','on');

try delete_line(sys,'Sum_V/1','PID_V/1'); catch, end
try delete_line(sys,'PID_V/1','Sat_delta_t/1'); catch, end
try delete_line(sys,'Sat_delta_t/1','delta_t/1'); catch, end
add_line(sys,'Sum_V/1','PID_V/1','autorouting','on');
add_line(sys,'PID_V/1','Sat_delta_t/1','autorouting','on');
add_line(sys,'Sat_delta_t/1','delta_t/1','autorouting','on');

% Save model
save_system('autopilot');