function configure_reusable_function()
% configure_reusable_function  Switch the autopilot model from the
% default nonreusable (singleton globals) code interface to a
% reusable-function packaging where every entry point takes a
% pointer to per-instance state. This is what makes it safe to
% instantiate the generated autopilot per-aircraft in CLEARANCE
% without every aircraft trampling the same file-scope globals.
%
% Idempotent: re-running just re-asserts the settings.

model = 'autopilot';
if ~bdIsLoaded(model)
    load_system(model);
end

% Reusable function packaging - moves autopilot_U / autopilot_Y /
% autopilot_X / autopilot_B off file-scope globals into a state
% struct accessed via an RT_MODEL_T pointer that each caller supplies.
set_param(model, 'CodeInterfacePackaging', 'Reusable function');

% Pass root I/O as fields of a single structure reference alongside
% the model pointer. This keeps the C wrapper on the CLEARANCE side
% simple - one pointer per instance, everything else is field access.
set_param(model, 'RootIOFormat', 'Part of model data structure');

% Keep continuous-time support on (the model has continuous PID
% filters + actuator lags) and don't generate a makefile (CLEARANCE
% compiles the source, we only want the .c/.h).
set_param(model, 'SupportContinuousTime', 'on');
set_param(model, 'GenerateMakefile',      'off');

save_system(model);
fprintf('Model configured for reusable function packaging.\n');
end
