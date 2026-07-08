% Script to modify autopilot/AutopilotSubsystem: create specified inports/outports and
% Sum->PID->Saturation->Outport paths for roll (phi_cmd->delta_a), pitch (theta_cmd->delta_e),
% throttle (V_cmd->delta_t). Uses workspace gains and saturation limits.

sys = 'autopilot/AutopilotSubsystem';

% Names for inports (use phi_cmd and theta_cmd instead of psi_cmd and h_cmd)
inports = {'phi_cmd','theta_cmd','V_cmd','phi','theta','psi','h','V','p','q'};
outports = {'delta_a','delta_e','delta_t'};

% Helper: remove block if exists
rmIfExists = @(blk) ifelse( ~isempty(find_system(sys,'SearchDepth',1,'Name',blk)), ...
    delete_block([sys '/' blk]), [] );

% inline ifelse function for compactness
function out = ifelse(cond, a, b), if cond, out = a; else out = b; end, end

% Remove existing blocks that will be replaced (inports, outports, named blocks)
for i=1:numel(inports)
    if ~isempty(find_system(sys,'SearchDepth',1,'Name',inports{i}))
        delete_block([sys '/' inports{i}]);
    end
end
for i=1:numel(outports)
    if ~isempty(find_system(sys,'SearchDepth',1,'Name',outports{i}))
        delete_block([sys '/' outports{i}]);
    end
end

% Also remove any existing blocks we will add with these exact names
toRemove = {'Sum_phi','PID_phi','Sat_phi','Sum_theta','PID_theta','Sat_theta','Sum_V','PID_V','Sat_V'};
for i=1:numel(toRemove)
    if ~isempty(find_system(sys,'SearchDepth',1,'Name',toRemove{i}))
        delete_block([sys '/' toRemove{i}]);
    end
end

% Add Inport blocks (set positions)
x0 = 30; y0 = 30; dy = 40;
for i=1:numel(inports)
    name = inports{i};
    pos = [x0, y0 + (i-1)*dy, x0+30, y0+20 + (i-1)*dy];
    add_block('simulink/Sources/In1',[sys '/' name],'Position',pos);
end

% Add Outport blocks
xout = 520; yout0 = 70;
for i=1:numel(outports)
    name = outports{i};
    pos = [xout, yout0 + (i-1)*90, xout+30, yout0+20 + (i-1)*90];
    add_block('simulink/Sinks/Out1',[sys '/' name],'Position',pos);
end

% Add blocks for roll (phi_cmd -> delta_a)
add_block('simulink/Math Operations/Sum',[sys '/Sum_phi'],'Position',[140 30 170 60],'Inputs','+-');
add_block('simulink/Continuous/PID Controller',[sys '/PID_phi'],'Position',[220 30 320 80]);
set_param([sys '/PID_phi'],'P','Kp_phi','I','Ki_phi','D','Kd_phi');
add_block('simulink/Discontinuities/Saturation',[sys '/Sat_phi'],'Position',[360 30 420 80]);
set_param([sys '/Sat_phi'],'UpperLimit','delta_max','LowerLimit','delta_min');

% Add blocks for pitch (theta_cmd -> delta_e)
add_block('simulink/Math Operations/Sum',[sys '/Sum_theta'],'Position',[140 140 170 170],'Inputs','+-');
add_block('simulink/Continuous/PID Controller',[sys '/PID_theta'],'Position',[220 140 320 190]);
set_param([sys '/PID_theta'],'P','Kp_theta','I','Ki_theta','D','Kd_theta');
add_block('simulink/Discontinuities/Saturation',[sys '/Sat_theta'],'Position',[360 140 420 190]);
set_param([sys '/Sat_theta'],'UpperLimit','delta_max','LowerLimit','delta_min');

% Add blocks for throttle (V_cmd -> delta_t)
add_block('simulink/Math Operations/Sum',[sys '/Sum_V'],'Position',[140 250 170 280],'Inputs','+-');
add_block('simulink/Continuous/PID Controller',[sys '/PID_V'],'Position',[220 250 320 300]);
set_param([sys '/PID_V'],'P','Kp_V','I','Ki_V','D','Kd_V');
add_block('simulink/Discontinuities/Saturation',[sys '/Sat_V'],'Position',[360 250 420 300]);
set_param([sys '/Sat_V'],'UpperLimit','delta_max','LowerLimit','delta_min');

% Connect lines (use autorouting)
set_param(sys,'ZoomFactor','100');

% Roll connections: phi_cmd -> Sum_phi(+) ; phi -> Sum_phi(-) ; Sum_phi -> PID_phi -> Sat_phi -> delta_a
add_line(sys,'phi_cmd/1','Sum_phi/1','autorouting','on');
add_line(sys,'phi/1','Sum_phi/2','autorouting','on');
add_line(sys,'Sum_phi/1','PID_phi/1','autorouting','on');
add_line(sys,'PID_phi/1','Sat_phi/1','autorouting','on');
add_line(sys,'Sat_phi/1','delta_a/1','autorouting','on');

% Pitch connections: theta_cmd -> Sum_theta(+) ; theta -> Sum_theta(-) ; Sum_theta -> PID_theta -> Sat_theta -> delta_e
add_line(sys,'theta_cmd/1','Sum_theta/1','autorouting','on');
add_line(sys,'theta/1','Sum_theta/2','autorouting','on');
add_line(sys,'Sum_theta/1','PID_theta/1','autorouting','on');
add_line(sys,'PID_theta/1','Sat_theta/1','autorouting','on');
add_line(sys,'Sat_theta/1','delta_e/1','autorouting','on');

% Throttle connections: V_cmd -> Sum_V(+) ; V -> Sum_V(-) ; Sum_V -> PID_V -> Sat_V -> delta_t
add_line(sys,'V_cmd/1','Sum_V/1','autorouting','on');
add_line(sys,'V/1','Sum_V/2','autorouting','on');
add_line(sys,'Sum_V/1','PID_V/1','autorouting','on');
add_line(sys,'PID_V/1','Sat_V/1','autorouting','on');
add_line(sys,'Sat_V/1','delta_t/1','autorouting','on');

% Clean up: refresh system
drawnow;
