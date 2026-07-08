
% tools/create_autopilot_model.m
% Programmatically create a minimal Simulink model scaffold for autopilot

repoRoot = fileparts(fileparts(mfilename('fullpath')));
if isempty(repoRoot), repoRoot = pwd; end
cd(repoRoot);

modelDir = fullfile(pwd,'model');
if ~exist(modelDir,'dir'), mkdir(modelDir); end

modelName = 'autopilot';                      % plain model name (no slashes)
modelFile = fullfile(modelDir,[modelName '.slx']);

% Close/delete existing model if present
if bdIsLoaded(modelName), close_system(modelName,0); end
if exist(modelFile,'file'), delete(modelFile); end

% Create new model
sys = new_system(modelName);
open_system(sys);

% Add Inports for commands and measurements
add_block('simulink/Sources/In1',[modelName '/psi_cmd'],'Position',[30 30 80 50]);
add_block('simulink/Sources/In1',[modelName '/h_cmd'],'Position',[30 80 80 100]);
add_block('simulink/Sources/In1',[modelName '/V_cmd'],'Position',[30 130 80 150]);

add_block('simulink/Sources/In1',[modelName '/phi'],'Position',[30 200 80 220]);
add_block('simulink/Sources/In1',[modelName '/theta'],'Position',[30 250 80 270]);
add_block('simulink/Sources/In1',[modelName '/psi'],'Position',[30 300 80 320]);
add_block('simulink/Sources/In1',[modelName '/h'],'Position',[30 350 80 370]);
add_block('simulink/Sources/In1',[modelName '/V'],'Position',[30 400 80 420]);
add_block('simulink/Sources/In1',[modelName '/p'],'Position',[30 450 80 470]);
add_block('simulink/Sources/In1',[modelName '/q'],'Position',[30 500 80 520]);

% Add placeholder controller subsystem
add_block('simulink/Ports & Subsystems/Subsystem',[modelName '/AutopilotSubsystem'],'Position',[150 60 420 540]);
set_param([modelName '/AutopilotSubsystem'],'Mask','on');

% Inside Subsystem: create named Outports for actuators
add_block('simulink/Ports & Subsystems/Subsystem',[modelName '/AutopilotSubsystem/Inner'],'Position',[10 10 380 380]);
open_system([modelName '/AutopilotSubsystem']);
add_block('simulink/Sources/Out1',[modelName '/AutopilotSubsystem/delta_a'],'Position',[420 80 460 100]);
add_block('simulink/Sources/Out1',[modelName '/AutopilotSubsystem/delta_e'],'Position',[420 140 460 160]);
add_block('simulink/Sources/Out1',[modelName '/AutopilotSubsystem/delta_t'],'Position',[420 200 460 220]);
close_system([modelName '/AutopilotSubsystem']);

% Add actuator dynamics placeholders and outports
add_block('simulink/Continuous/Transfer Fcn',[modelName '/elev_act'],'Position',[520 120 620 170]);
set_param([modelName '/elev_act'],'Numerator','1','Denominator','[0.05 1]');
add_block('simulink/Discrete/Rate Limiter',[modelName '/elev_rate'],'Position',[640 120 740 170]);
add_block('simulink/Sinks/Out1',[modelName '/delta_e_out'],'Position',[820 130 860 150]);

add_block('simulink/Continuous/Transfer Fcn',[modelName '/ail_act'],'Position',[520 60 620 110]);
set_param([modelName '/ail_act'],'Numerator','1','Denominator','[0.05 1]');
add_block('simulink/Discrete/Rate Limiter',[modelName '/ail_rate'],'Position',[640 60 740 110]);
add_block('simulink/Sinks/Out1',[modelName '/delta_a_out'],'Position',[820 70 860 90]);

add_block('simulink/Sinks/Out1',[modelName '/delta_t_out'],'Position',[820 190 860 210]);

% Add State-Space blocks for plant (example matrices)
A_long = '[0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0]';
B_long = '[0 0; 0 0; 0 0; 1 0]';
add_block('simulink/Continuous/State-Space',[modelName '/LongitudinalPlant'],'Position',[940 40 1200 160]);
set_param([modelName '/LongitudinalPlant'],'A',A_long,'B',B_long,'C','eye(4)','D','zeros(4,2)');

A_lat = '[ -1.5 0 0; 1 0 0; 0 125 0 ]';
B_lat = '[15; 0; 0]';
add_block('simulink/Continuous/State-Space',[modelName '/LateralPlant'],'Position',[940 200 1200 320]);
set_param([modelName '/LateralPlant'],'A',A_lat,'B',B_lat,'C','eye(3)','D','zeros(3,1)');

% Save using full path and close
save_system(modelName, modelFile);
close_system(modelName);
fprintf('Created model: %s\n', modelFile);