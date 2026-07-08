% generated/regen_code.m
% Regenerate Embedded Coder code for the Autopilot model and write metadata.
% Run from repo root: run(fullfile('generated','regen_code.m'));

% Minimal robustness
repoRoot = fileparts(fileparts(mfilename('fullpath')));
cd(repoRoot);

% Load params
paramsFile = fullfile('model','plant_params.m');
if ~isfile(paramsFile)
    error('regen_code:MissingParams','%s not found', paramsFile);
end
run(paramsFile);

modelPath = fullfile('model','autopilot.slx');
modelName = 'model/autopilot';

% Load model
if ~bdIsLoaded(modelName)
    load_system(modelPath);
end

% Configure model for code generation
set_param(modelName, 'SystemTargetFile', 'ert.tlc');
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'FixedStep', num2str(CODEGEN_STEP));
set_param(modelName, 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex');

% Build
fprintf('Starting code generation for %s (FixedStep=%g)\n', modelName, CODEGEN_STEP);
buildInfo = slbuild(modelName);

% Locate generated artifacts (typical location: <model>_ert_rtw/)
rtwDir = [fullfile(pwd, [ 'autopilot_ert_rtw' ])];
if ~exist(rtwDir,'dir')
    % try find
    d = dir('**/autopilot_ert_rtw');
    if ~isempty(d), rtwDir = fullfile(d(1).folder,d(1).name);
    else
        error('regen_code:NoRTWDir','Could not find generated rtw directory.');
    end
end

% Files to copy
outDir = fullfile('generated');
if ~exist(outDir,'dir'), mkdir(outDir); end
filesToCopy = {'autopilot.c','autopilot.h','rtwtypes.h'};
for i=1:numel(filesToCopy)
    src = fullfile(rtwDir, filesToCopy{i});
    if isfile(src)
        copyfile(src, fullfile(outDir, filesToCopy{i}));
    else
        warning('regen_code:MissingFile','%s not found in %s', filesToCopy{i}, rtwDir);
    end
end

% Write metadata
meta.MATLAB_VERSION = version;
meta.timestamp = datestr(now,'yyyy-mm-ddTHH:MM:SS');
meta.model = modelName;
meta.fixed_step = CODEGEN_STEP;
meta.plant_source = PLANT_DERIV_SOURCE;
meta.git_commit = get_git_commit_hash(repoRoot);
meta.rtw_dir = rtwDir;
meta_file = fullfile(outDir,'METADATA.json');
fid = fopen(meta_file,'w');
if fid>0
    fprintf(fid, '%s\n', jsonencode(meta));
    fclose(fid);
    fprintf('Wrote metadata to %s\n', meta_file);
else
    warning('regen_code:WriteMeta','Could not write metadata file.');
end

fprintf('Code generation complete. Artifacts in %s\n', outDir);

%% Helper: small function to get git commit (best-effort)
function h = get_git_commit_hash(root)
h = '';
try
    c = sprintf('cd "%s" && git rev-parse --short HEAD', root);
    [s,o] = system(c);
    if s==0
        h = strtrim(o);
    end
catch
    % ignore
end
end
