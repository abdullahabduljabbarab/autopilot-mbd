% Add missing inports psi_cmd and h_cmd inside autopilot/AutopilotSubsystem (safe: remove if exist)
sys = 'autopilot/AutopilotSubsystem';

% Remove existing blocks with these names if they exist
if ~isempty(find_system(sys,'SearchDepth',1,'Name','psi_cmd')), delete_block([sys '/psi_cmd']); end
if ~isempty(find_system(sys,'SearchDepth',1,'Name','h_cmd')),   delete_block([sys '/h_cmd']);   end

% Add psi_cmd and h_cmd Inport blocks (positions chosen to align with existing inports)
add_block('simulink/Sources/In1',[sys '/psi_cmd'],'Position',[30 150 60 170]);
add_block('simulink/Sources/In1',[sys '/h_cmd'],'Position',[30 190 60 210]);

% Refresh diagram
drawnow;
