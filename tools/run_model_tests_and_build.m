
% run_model_tests_and_build.m  (script version)
model = 'autopilot';
artifactsDir = fullfile(pwd,'ci_artifacts');
if ~isfolder(artifactsDir), mkdir(artifactsDir); end

% 1) Load model
fprintf('Loading model %s...\n', model);
load_system(model);

% 2) Smoke simulation (short run)
try
    fprintf('Running smoke simulation...\n');
    simOut = sim(model,'SaveOutput','on','SaveFormat','Dataset','Timeout',300);
    save(fullfile(artifactsDir,'simOut_smoke.mat'),'simOut');
catch ME
    warning('Smoke sim failed: %s', ME.message);
end

% 3) Run Test Manager tests if available
hasSimTest = ~isempty(ver('sltest')) || ~isempty(ver('Simulink Test'));
if hasSimTest
    try
        fprintf('Running Test Manager tests...\n');
        import sltest.*;
        tcs = sltest.testmanager.TestCase.getAll('IncludeChildren',true);
        if isempty(tcs)
            ts = TestSuite.fromModel(model);
            r = run(ts);
        else
            r = sltest.testmanager.run;
        end
        save(fullfile(artifactsDir,'test_results.mat'),'r');
    catch ME
        warning('Test Manager run failed: %s', ME.message);
    end
else
    fprintf('Simulink Test not available; skipping Test Manager step.\n');
end

% 4) Collect model coverage if available
try
    if ~isempty(ver('Simulink Coverage')) || exist('coverage','file')==2
        fprintf('Attempting model coverage collection...\n');
        covObj = cvsim(model,'CovMode','on','SaveOutput','on');
        save(fullfile(artifactsDir,'model_coverage.mat'),'covObj');
        try covSummary = cvsummary(covObj); save(fullfile(artifactsDir,'coverage_summary.mat'),'covSummary'); end
    else
        fprintf('Simulink Coverage not present; skipping coverage.\n');
    end
catch ME
    warning('Coverage step failed or not available: %s', ME.message);
end

% 5) Generate traceability report (reads req_map.csv if present)
try
    if exist('req_map.csv','file')
        fprintf('Generating traceability report from req_map.csv...\n');
        Tbl = readtable('req_map.csv','TextType','string');
        n = height(Tbl);
        ReqID = strings(n,1); BlockPath = strings(n,1); BlockType = strings(n,1);
        Exists = false(n,1); UserData = strings(n,1); HasProbe = false(n,1);
        for i=1:n
            bp = char(Tbl.Block{i});
            ReqID(i) = string(Tbl.ReqID{i});
            BlockPath(i) = string(bp);
            try
                bt = get_param(bp,'BlockType'); BlockType(i)=string(bt); Exists(i)=true;
                ud = get_param(bp,'UserData'); if ~isempty(ud), UserData(i)=string(mat2str(ud)); end
                ph = get_param(bp,'PortHandles'); probeFound=false;
                try
                    outs = get_param(ph.Outport,'Line'); if iscell(outs), outs=[outs{:}]; end
                    for L = outs(:).'
                        try dsts = get_param(L,'DstBlockHandle'); catch dsts = []; end
                        if ~isempty(dsts)
                            for d = dsts(:).'
                                try dn = get_param(d,'BlockType'); catch dn = ''; end
                                if strcmp(dn,'ToWorkspace'), probeFound=true; end
                            end
                        end
                    end
                catch, end
                HasProbe(i)=probeFound;
            catch
                BlockType(i) = "<missing>"; Exists(i)=false;
            end
        end
        ReportTable = table(ReqID,BlockPath,BlockType,Exists,UserData,HasProbe, 'VariableNames',{'ReqID','BlockPath','BlockType','Exists','UserData','HasProbe'});
        writetable(ReportTable,fullfile(artifactsDir,'traceability_report.csv'));
        fid = fopen(fullfile(artifactsDir,'traceability_report.html'),'w');
        fprintf(fid,'<html><body><h3>Traceability Report</h3><table border="1"><tr><th>ReqID</th><th>BlockPath</th><th>Type</th><th>Exists</th></tr>');
        for i=1:height(ReportTable)
            fprintf(fid,'<tr><td>%s</td><td>%s</td><td>%s</td><td>%d</td></tr>', ReportTable.ReqID{i}, ReportTable.BlockPath{i}, char(ReportTable.BlockType{i}), ReportTable.Exists(i));
        end
        fprintf(fid,'</table></body></html>');
        fclose(fid);
    else
        fprintf('req_map.csv not found; skipping traceability report.\n');
    end
catch ME
    warning('Traceability report failed: %s', ME.message);
end

% 6) Attempt code generation/build if ERT is configured
try
    fprintf('Attempting model build/codegen (slbuild)...\n');
    buildOutput = slbuild(model);
    genFolder = fullfile(pwd,[model '_grt_rtw']);
    if isfolder(fullfile(pwd,'slprj')), copyfile(fullfile(pwd,'slprj'), fullfile(artifactsDir,'slprj')); end
    if isfolder(genFolder), copyfile(genFolder, fullfile(artifactsDir,'codegen')); end
catch ME
    warning('slbuild failed or not configured: %s', ME.message);
end

% 7) Collect other artifacts
try save(fullfile(artifactsDir,'workspace_snapshot.mat')); catch, end

% 8) Zip artifacts
zipFile = fullfile(pwd,'ci_artifacts.zip');
if exist(zipFile,'file'), delete(zipFile); end
zip(zipFile,artifactsDir);
fprintf('Packaged artifacts to %s\n', zipFile);

fprintf('CI script finished.\n');