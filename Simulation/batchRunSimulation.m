

for testNum = 1:18
    
    if mod(testNum, 2)
        newValue = '1.0f';
        trajNum = 1;
    else
        newValue = '-1.0f';
        trajNum = 2;
    end
    setCDefine('..\common\trajectoryPlanner.c', 'DIRECTION', newValue);
    
    testCaseNum = ceil(testNum / 2);

    cfg = configureSim3(ceil(testNum / 2));
    
    BuildSimulation(cfg)

    RunSimulation(cfg);

    % move to data folder and rename
    fileName = dir(sprintf("cfg_*T%d.mat", testCaseNum));
    movefile(fileName.name, sprintf("data\\T%d_C%d_data.mat", trajNum, testCaseNum));
    
    % plottingScriptLeader
    
    % plottingScriptViewer

end

function setCDefine(fileName, defineName, newValue)
%SETCDEFINE Change the value of a C #define while preserving the rest
% of the line.
%
% Example:
%   setCDefine("config.h", "TEST_CASE", 12)
%
%   #define TEST_CASE 1    // comment
%
% becomes:
%
%   #define TEST_CASE 12    // comment

    txt = fileread(fileName);

    % Match everything up through the define name and whitespace,
    % then match only the current value (non-whitespace characters).
    pattern = sprintf( ...
        '(?m)^(\\s*#\\s*define\\s+%s\\s+)\\S+', ...
        regexptranslate('escape', defineName));

    matches = regexp(txt, pattern, 'match');

    if isempty(matches)
        error('Define "%s" not found in %s.', defineName, fileName);
    elseif numel(matches) > 1
        error('Multiple definitions of "%s" found in %s.', ...
            defineName, fileName);
    end

    replacement = sprintf('$1%s', string(newValue));

    txt = regexprep(txt, pattern, replacement);

    fid = fopen(fileName, 'w');

    if fid == -1
        error('Could not open %s for writing.', fileName);
    end

    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, '%s', txt);
end