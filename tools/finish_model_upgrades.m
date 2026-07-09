
function finish_model_upgrades()
% Finish adding/wiring blocks and enable anti-windup. Run from anywhere.

mdl = 'autopilot';
sub = [mdl '/AutopilotSubsystem'];

% Determine script/repo locations
scriptFolder = fileparts(which(mfilename)); % .../tools
repoRoot = fileparts(scriptFolder);         % repo root
modelFile = fullfile(repoRoot,'tools',[mdl '.slx']);

% Ensure model file exists and is loaded
if ~isfile(modelFile)
    error('Model file not found: %s', modelFile);
end
if ~bdIsLoaded(mdl)
    load_system(modelFile);
end
load_system('simulink');

% Library paths to try (common in R2026+)
LIB_SAT = 'simulink/Commonly Used Blocks/Saturation';
LIB_SUM = 'simulink/Math Operations/Sum';
LIB_PID = 'simulink/Continuous/PID Controller';
LIB_OUT = 'simulink/Sinks/Out1';

% Add or configure blocks
ensureBlock([mdl '/AutopilotSubsystem/Sat_phi_cmd_from_psi'], LIB_SAT);
set_param([mdl '/AutopilotSubsystem/Sat_phi_cmd_from_psi'],'UpperLimit','deg2rad(25)','LowerLimit','deg2rad(-25)');

ensureBlock([mdl '/AutopilotSubsystem/probe_phi_cmd_from_psi'], LIB_OUT);
try set_param([mdl '/AutopilotSubsystem/probe_phi_cmd_from_psi'],'Port','100'); catch; end

ensureBlock([mdl '/AutopilotSubsystem/Sum_h_error'], LIB_SUM);
try set_param([mdl '/AutopilotSubsystem/Sum_h_error'],'Inputs','+-'); catch; end

ensureBlock([mdl '/AutopilotSubsystem/PID_h'], LIB_PID);
% guarded PID param update
try
    dlg = get_param([mdl '/AutopilotSubsystem/PID_h'],'DialogParameters');
    if isfield(dlg,'P'), set_param([mdl '/AutopilotSubsystem/PID_h'],'P','0.02'); end
    if isfield(dlg,'I'), set_param([mdl '/AutopilotSubsystem/PID_h'],'I','0.005'); end
    if isfield(dlg,'D'), set_param([mdl '/AutopilotSubsystem/PID_h'],'D','0'); end
catch
end

ensureBlock([mdl '/AutopilotSubsystem/Sat_theta_cmd_from_h'], LIB_SAT);
set_param([mdl '/AutopilotSubsystem/Sat_theta_cmd_from_h'],'UpperLimit','deg2rad(10)','LowerLimit','deg2rad(-10)');

ensureBlock([mdl '/AutopilotSubsystem/probe_theta_cmd_from_h'], LIB_OUT);
try set_param([mdl '/AutopilotSubsystem/probe_theta_cmd_from_h'],'Port','101'); catch; end

% Safe wiring helper: uses subsystem-relative names
safeAddLine(sub,'Sum_psi_error/1','PID_psi/1');
safeAddLine(sub,'PID_psi/1','Sat_phi_cmd_from_psi/1');
safeAddLine(sub,'Sat_phi_cmd_from_psi/1','probe_phi_cmd_from_psi/1');

safeAddLine(sub,'Sum_h_error/1','PID_h/1');
safeAddLine(sub,'PID_h/1','Sat_theta_cmd_from_h/1');
safeAddLine(sub,'Sat_theta_cmd_from_h/1','probe_theta_cmd_from_h/1');

% Try connect probes to PID command ports (attempt 2,3,1)
tryConnect(sub,'probe_phi_cmd_from_psi','PID_phi');
tryConnect(sub,'probe_theta_cmd_from_h','PID_theta');

% Enable anti-windup on common PIDs
pids = {'PID_phi','PID_theta','PID_V','PID_psi','PID_h'};
for k=1:numel(pids)
    blk = [sub '/' pids{k}];
    if ~isempty(find_system(blk,'SearchDepth',0))
        try
            dlg = get_param(blk,'DialogParameters');
            if isfield(dlg,'AntiWindupMethod'), set_param(blk,'AntiWindupMethod','BackCalculation'); end
            if isfield(dlg,'BackCalculationGain'), set_param(blk,'BackCalculationGain','0.5'); end
        catch
        end
    end
end

% Save and open
save_system(mdl);
ts = datestr(now,'yyyymmdd_HHMMSS');
upgraded = fullfile(repoRoot,'tools',[mdl '_upgraded_' ts '.slx']);
save_system(mdl, upgraded);
fprintf('Saved upgraded model to:\n%s\n', upgraded);

% Ensure model loaded then open subsystem
if ~bdIsLoaded(mdl)
    load_system(modelFile);
end
open_system([mdl '/AutopilotSubsystem']);


% ---------------- Local helper functions ----------------
    function ensureBlock(destFull, libPath)
        % Add libPath -> destFull if missing. libPath must exist.
        if isempty(find_system(destFull,'SearchDepth',0))
            if isempty(find_system(libPath,'SearchDepth',0))
                error('Required library block not found: %s', libPath);
            end
            add_block(libPath, destFull, 'MakeNameUnique','off');
        end
    end

    function safeAddLine(bd, src, dst)
        try
            add_line(bd, src, dst, 'autorouting','on');
        catch
            % ignore if line exists or other non-fatal error
        end
    end

    function tryConnect(bd, probeName, pidName)
        % Try ports 2,3,1 on target PID (best-effort)
        for p = [2 3 1]
            try
                add_line(bd, [probeName '/1'], [pidName '/' num2str(p)], 'autorouting','on');
                return;
            catch
            end
        end
        % if we get here, connection failed
    end

end