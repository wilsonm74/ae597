pdfFile = "testResults.pdf";

if isfile(pdfFile)
    delete(pdfFile);
end

dataFiles = dir('data/*.mat');

numTests = length(dataFiles);

numTests = 1;

for testIdx = 1:numTests

    load(fullfile('data', (dataFiles(testIdx).name)));

    t = cfg.trueStateT{1};

    leaderStates = cfg.trueStates{1};
    viewerStates = cfg.trueStates{2};

    leaderDebug = getDebugFloatTelemetry(cfg, 1);
    viewerDebug = getDebugFloatTelemetry(cfg, 2);

    refTraj = leaderDebug(3:5, :);
    leaderPos = leaderStates(1:3, :);
    viewerPos = viewerStates(1:3, :);

    fig = figure( ...
        'Visible', 'off', ...
        'Units', 'inches', ...
        'Position', [1 1 11 8.5]);

    tl = tiledlayout(fig, 3, 3, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    title(tl, sprintf('Test Case %d', testIdx), ...
        'FontSize', 16, ...
        'FontWeight', 'bold');

    % Plot 1 (3D Trajectory)
    nexttile(tl, 1, [2 2]);
    plot3(refTraj(1,:), refTraj(2,:), refTraj(3,:), 'k--')
    hold on
    cmap = jet(length(t));
    for idx = 1:length(t)
        plot3(leaderPos(1,idx), leaderPos(2,idx), leaderPos(3,idx), '.', 'color', cmap(idx, :), 'MarkerSize', 10)
    end
    plot3(viewerPos(1,1), viewerPos(2,1), viewerPos(3,1), 'k.', 'MarkerSize', 20)
    hold off
    grid on;
    axis equal;
    xlim([-0.8, 0.8]); ylim([-1.2, 1.2]); zlim([-0.8, 0.8]);
    title('Trajectory');

    % Plot 2 (States)
    nexttile(tl, 3, [1, 1])

    nexttile(tl, 6, [1, 1])

    nexttile(tl, 7, [1, 1])

    nexttile(tl, 8, [1, 1])

    nexttile(tl, 9, [1, 1])

    % More plots...

    exportgraphics(fig, pdfFile, ...
        'ContentType', 'vector', ...
        'Append', true);

    close(fig);
end


function debugFloat = getDebugFloatTelemetry(completedCfg, sphereIndex)
    sphereData = completedCfg.data{1, sphereIndex};
    names = fieldnames(sphereData);
    normalized = lower(regexprep(names, '[^a-zA-Z0-9]', ''));
    candidates = find(contains(normalized, 'debug') & ...
        contains(normalized, 'float') & ~contains(normalized, 'time'));
    if isempty(candidates)
        error('No float debug telemetry found.');
    end
    debugFloat = sphereData.(names{candidates(1)});
    if size(debugFloat, 1) ~= 8 && size(debugFloat, 2) == 8
        debugFloat = debugFloat.';
    end
end
