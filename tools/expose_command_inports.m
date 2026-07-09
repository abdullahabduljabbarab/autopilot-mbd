function expose_command_inports()
% expose_command_inports  Replace the internal step blocks
% (force_phi_step, theta_cmd_step) with root inports so external
% callers (CLEARANCE aircraft behaviour) can drive phi_cmd and
% theta_cmd every step. Preserves the existing wiring into
% AutopilotSubsystem's phi_cmd (port 1) and theta_cmd (port 2)
% input pins.
%
% Idempotent - if the step blocks are already gone or the root
% inports already exist, the script skips those steps and just
% ensures the wiring is correct.

model = 'autopilot';
if ~bdIsLoaded(model)
    load_system(model);
end

% Highest existing root inport number so we can append after it.
existing = find_system(model, 'SearchDepth', 1, 'BlockType', 'Inport');
nextInport = numel(existing) + 1;
fprintf('Model has %d existing root inports; adding starting at port %d\n', ...
    numel(existing), nextInport);

% --- phi_cmd -----------------------------------------------------------
stepBlk = [model '/force_phi_step'];
if ~isempty(find_system(model, 'SearchDepth', 1, 'Name', 'force_phi_step'))
    fprintf('  Removing force_phi_step block\n');
    % Break existing lines feeding AutopilotSubsystem/phi_cmd first
    try
        delete_line(model, 'force_phi_step/1', 'AutopilotSubsystem/1');
    catch
    end
    delete_block(stepBlk);
end

% Add root inport named phi_cmd
if isempty(find_system(model, 'SearchDepth', 1, 'BlockType', 'Inport', 'Name', 'phi_cmd'))
    add_block('built-in/Inport', [model '/phi_cmd'], ...
        'Port', num2str(nextInport), ...
        'Position', [100 60 130 80]);
    add_line(model, 'phi_cmd/1', 'AutopilotSubsystem/1', 'autorouting', 'on');
    fprintf('  Added phi_cmd root inport (port %d) and wired to AutopilotSubsystem/1\n', nextInport);
    nextInport = nextInport + 1;
else
    fprintf('  phi_cmd root inport already exists, skipping\n');
    % Ensure it's wired even if it existed
    try
        add_line(model, 'phi_cmd/1', 'AutopilotSubsystem/1', 'autorouting', 'on');
    catch
    end
end

% --- theta_cmd ---------------------------------------------------------
stepBlk = [model '/theta_cmd_step'];
if ~isempty(find_system(model, 'SearchDepth', 1, 'Name', 'theta_cmd_step'))
    fprintf('  Removing theta_cmd_step block\n');
    try
        delete_line(model, 'theta_cmd_step/1', 'AutopilotSubsystem/2');
    catch
    end
    delete_block(stepBlk);
end

if isempty(find_system(model, 'SearchDepth', 1, 'BlockType', 'Inport', 'Name', 'theta_cmd'))
    add_block('built-in/Inport', [model '/theta_cmd'], ...
        'Port', num2str(nextInport), ...
        'Position', [100 160 130 180]);
    add_line(model, 'theta_cmd/1', 'AutopilotSubsystem/2', 'autorouting', 'on');
    fprintf('  Added theta_cmd root inport (port %d) and wired to AutopilotSubsystem/2\n', nextInport);
else
    fprintf('  theta_cmd root inport already exists, skipping\n');
    try
        add_line(model, 'theta_cmd/1', 'AutopilotSubsystem/2', 'autorouting', 'on');
    catch
    end
end

save_system(model);
fprintf('done. Model saved. phi_cmd and theta_cmd are now root inports.\n');
end
