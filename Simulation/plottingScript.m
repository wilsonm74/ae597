
useEstimatedState = false;

if useEstimatedState
    time = cfg.data{1}.BackTel.StdTime;
    state = cfg.data{1}.BackTel.StdState;
else
    time = cfg.trueStateT{1};
    state = cfg.trueStates{1};
end

t_s = time / 1000.0;

debug_float = getDebugFloatTelemetry(cfg, 1);
traj_t = debug_float(1,:);
traj = [debug_float(3:5,:)];

figure(1)
tiledlayout(2,2)

nexttile
plot(time / 1000, state(1:3, :))

nexttile
plot(state(1,:), state(3,:))

nexttile
plot(time, state(4:6, :))

nexttile
plot(time, state(7:10, :))


figure(2)
tiledlayout(1,1)

nexttile
plot3(state(1,:), state(2,:), state(3,:))
grid on
axis equal

hold on
plot3(traj(1,:), traj(2,:), traj(3,:), 'k--')

cmap = jet(length(time));

for idx = 1:length(time)
    plot3(state(1,idx), state(2,idx), state(3,idx), '.', 'color', cmap(idx, :), 'MarkerSize', 10)
end

figure(3)
tiledlayout(2,1)

nexttile
plot(traj_t, debug_float(6:8,:))

nexttile
plot(traj_t, debug_float(3:5,:))

figure(4)
tiledlayout(1,1)

nexttile
plot(t_s, state(1:3,:), '--');
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(traj_t, traj)
hold off


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
