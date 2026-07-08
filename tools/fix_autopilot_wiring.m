
% fix_autopilot_wiring.m
sys = 'autopilot/AutopilotSubsystem';

% Helper: safe delete a line by source and destination (silent if missing)
function safeDeleteLine(sysPath, srcPort, dstPort)
try
    delete_line(sysPath, srcPort, dstPort);
catch
    % ignore
end
end

% --- Clear outgoing lines from internal Inports to remove forks ---
inames = {'phi_cmd','theta_cmd','V_cmd','phi','theta','psi','h','V','p','q','psi_cmd','h_cmd'};
for i=1:numel(inames)
    try
        ph = get_param([sys '/' inames{i}], 'PortHandles');
        if ~isempty(ph) && ~isempty(ph.Outport)
            ln = get_param(ph.Outport, 'Line');
            if ln ~= -1
                % delete the line by handle
                try delete_line(ln); catch, end
            end
        end
    catch
        % ignore missing blocks
    end
end

% --- Ensure key blocks exist (warn if missing) ---
blocksNeeded = {'Sum_phi','PID_phi','Sat_phi','Sum_theta','PID_theta','Sat_theta','Sum_V','PID_V','Sat_V','delta_a','delta_e','delta_t'};
for i=1:numel(blocksNeeded)
    if isempty(find_system(sys,'SearchDepth',1,'Name',blocksNeeded{i}))
        fprintf('WARNING: missing inside subsystem: %s\n', blocksNeeded{i});
    end
end

% --- Connect primary control loops (safe add_line) ---
try add_line(sys,'phi_cmd/1','Sum_phi/1','autorouting','on'); catch, end
try add_line(sys,'phi/1','Sum_phi/2','autorouting','on'); catch, end
try add_line(sys,'Sum_phi/1','PID_phi/1','autorouting','on'); catch, end
try add_line(sys,'PID_phi/1','Sat_phi/1','autorouting','on'); catch, end
try add_line(sys,'Sat_phi/1','delta_a/1','autorouting','on'); catch, end

try add_line(sys,'theta_cmd/1','Sum_theta/1','autorouting','on'); catch, end
try add_line(sys,'theta/1','Sum_theta/2','autorouting','on'); catch, end
try add_line(sys,'Sum_theta/1','PID_theta/1','autorouting','on'); catch, end
try add_line(sys,'PID_theta/1','Sat_theta/1','autorouting','on'); catch, end
try add_line(sys,'Sat_theta/1','delta_e/1','autorouting','on'); catch, end

try add_line(sys,'V_cmd/1','Sum_V/1','autorouting','on'); catch, end
try add_line(sys,'V/1','Sum_V/2','autorouting','on'); catch, end
try add_line(sys,'Sum_V/1','PID_V/1','autorouting','on'); catch, end
try add_line(sys,'PID_V/1','Sat_V/1','autorouting','on'); catch, end
try add_line(sys,'Sat_V/1','delta_t/1','autorouting','on'); catch, end

% --- Add Terminators and connect unused inputs ---
unused = {'psi','h','p','q','psi_cmd','h_cmd'};
xPos = 420; yBase = 40; dy = 30;
for i=1:numel(unused)
    termName = ['Term_' unused{i}];
    if isempty(find_system(sys,'SearchDepth',1,'Name',termName))
        add_block('simulink/Sinks/Terminator',[sys '/' termName],'Position',[xPos, yBase + (i-1)*dy, xPos+20, yBase+10 + (i-1)*dy]);
    end
    if ~isempty(find_system(sys,'SearchDepth',1,'Name',unused{i}))
        try
            ph = get_param([sys '/' unused{i}], 'PortHandles');
            if ~isempty(ph) && ~isempty(ph.Outport)
                ln = get_param(ph.Outport, 'Line');
                if ln == -1
                    try add_line(sys, [unused{i} '/1'], [termName '/1'], 'autorouting','on'); catch, end
                end
            end
        catch
            % ignore
        end
    end
end

drawnow;
fprintf('Done. Open autopilot/AutopilotSubsystem and check Sum->PID->Sat->Out for roll, pitch, throttle.\n');