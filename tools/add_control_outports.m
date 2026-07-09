function add_control_outports()
% add_control_outports  Add root outports for delta_e and delta_a so
%                       Embedded Coder exposes them in the generated
%                       C API alongside delta_t_out.
%
% Idempotent - re-running is safe; existing outports are detected and
% skipped. Signal branches are auto-routed from the existing lines
% feeding the actuator lag models.

model = 'autopilot';
if ~bdIsLoaded(model)
    load_system(model);
end

% Figure out the next free root outport index.
existing = find_system(model, 'SearchDepth', 1, 'BlockType', 'Outport');
nextPort = numel(existing) + 1;
fprintf('Model has %d existing root outports; adding starting at port %d\n', ...
    numel(existing), nextPort);

% --- delta_e_out -------------------------------------------------------
% Wire the top-level delta_e signal (exits AutopilotSubsystem port 2,
% same signal that feeds elev_act) to a new root outport.
if isempty(find_system(model, 'SearchDepth', 1, 'BlockType', 'Outport', 'Name', 'delta_e_out'))
    add_block('built-in/Outport', [model '/delta_e_out'], ...
        'Port', num2str(nextPort), ...
        'Position', [900 100 930 120]);
    try
        add_line(model, 'AutopilotSubsystem/2', 'delta_e_out/1', ...
            'autorouting', 'on');
        fprintf('  Added delta_e_out (port %d) and wired from AutopilotSubsystem/2\n', nextPort);
    catch ME
        warning('Could not wire delta_e_out automatically: %s. Connect it manually.', ME.message);
    end
    nextPort = nextPort + 1;
else
    fprintf('  delta_e_out already exists, skipping\n');
end

% --- delta_a_out -------------------------------------------------------
if isempty(find_system(model, 'SearchDepth', 1, 'BlockType', 'Outport', 'Name', 'delta_a_out'))
    add_block('built-in/Outport', [model '/delta_a_out'], ...
        'Port', num2str(nextPort), ...
        'Position', [900 200 930 220]);
    try
        add_line(model, 'AutopilotSubsystem/1', 'delta_a_out/1', ...
            'autorouting', 'on');
        fprintf('  Added delta_a_out (port %d) and wired from AutopilotSubsystem/1\n', nextPort);
    catch ME
        warning('Could not wire delta_a_out automatically: %s. Connect it manually.', ME.message);
    end
else
    fprintf('  delta_a_out already exists, skipping\n');
end

save_system(model);
fprintf('done. Model saved with new root outports.\n');
end
