function compare_sim()
% compare_sim  Regression test: compare current sim output against a
%              stored reference (ci_artifacts/simOut_with_pid_defaults.mat)
%              and fail if the final delta_e drifts outside tolerance.
%
% Used by the CI job (tools/run_model_tests_and_build.m calls this at
% the end of the smoke-sim step). Also runnable standalone from the
% repo root:
%
%   >> addpath(genpath(pwd)); tools/compare_sim
%
% Exits with status 0 on pass, 1 on fail so the CI shell picks it up.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
refFile  = fullfile(repoRoot, 'ci_artifacts', 'simOut_with_pid_defaults.mat');

if ~isfile(refFile)
    fprintf(2, 'FAIL: reference sim file missing: %s\n', refFile);
    exit(1);
end

S = load(refFile);
if ~isfield(S, 'simOut')
    fprintf(2, 'FAIL: reference file %s has no simOut variable\n', refFile);
    exit(1);
end
simOut = S.simOut;

% Pull the delta_e signal out of the sim output. Robust to both
% Dataset and struct-with-time formats.
deltaE = extractDeltaE(simOut);
if isempty(deltaE)
    fprintf(2, 'FAIL: could not locate delta_e signal in simOut\n');
    exit(1);
end

finalDeltaE = deltaE(end);

% Reference value + tolerance. When the model or gains change, update
% both together and note the change in the commit message so the
% traceability trail stays clean.
expected  = 0.58;
tolerance = 1e-2;

fprintf('final delta_e = %.6f  (expected %.6f, tolerance %.4f)\n', ...
    finalDeltaE, expected, tolerance);

if abs(finalDeltaE - expected) <= tolerance
    fprintf('PASS: final delta_e within tolerance\n');
    exit(0);
else
    fprintf(2, 'FAIL: final delta_e outside tolerance\n');
    exit(1);
end
end

% -----------------------------------------------------------------------
function d = extractDeltaE(simOut)
% Handle both Simulink.SimulationOutput and struct-with-time shapes.
d = [];
try
    if isa(simOut, 'Simulink.SimulationOutput')
        logs = simOut.get('logsout');
        if ~isempty(logs)
            el = logs.getElement('delta_e');
            d  = el.Values.Data;
            return;
        end
    end
    if isstruct(simOut) && isfield(simOut, 'delta_e_log2')
        d = simOut.delta_e_log2;
        return;
    end
catch
    d = [];
end
end
