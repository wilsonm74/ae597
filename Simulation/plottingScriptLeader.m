
useEstimatedState = false;
legLocation = 'eastoutside';

if useEstimatedState
    time = cfg.data{1}.BackTel.StdTime;
    state = cfg.data{1}.BackTel.StdState;
else
    time = cfg.trueStateT{1};
    state = cfg.trueStates{1};
end

t_s = time / 1000.0;

debug_float = getDebugFloatTelemetry(cfg, 1);
debug_t = debug_float(1,:);
traj = [debug_float(3:5,:)];

%%

figure(1)
tiledlayout(2,2)

nexttile
plot(t_s, state(1:3, :))
title('Position vs. Time')
xlabel('Time (s)');
ylabel('Position (m)');
legend({'X', 'Y', 'Z'}, 'Location', legLocation)
grid on

nexttile
plot(time, state(4:6, :))
title('Velocity vs. Time')
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend({'X', 'Y', 'Z'}, 'Location', legLocation)
grid on

nexttile
plot(time, state(7:10, :))
title('Attitude vs. Time')
xlabel('Time (s)');
ylabel('Quaternion Component (n/a)');
legend({'W', 'X', 'Y', 'Z'}, 'Location', legLocation)
grid on

nexttile
plot(time, state(11:13, :))
title('Angular Rate vs. Time')
xlabel('Time (s)');
ylabel('Angular Rate (rad/s)');
legend({'X', 'Y', 'Z'}, 'Location', legLocation)
grid on


%%

figure(2)
tiledlayout(1,1)

nexttile
plot3(traj(1,:), traj(2,:), traj(3,:), 'k--')
grid on
axis equal

hold on

cmap = jet(length(time));

for idx = 1:length(time)
    plot3(state(1,idx), state(2,idx), state(3,idx), '.', 'color', cmap(idx, :), 'MarkerSize', 10)
end
hold off

title('3D Path vs. Time')
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
legend({'Trajectory Reference'}, 'Location', legLocation)

%%

figure(3)
tiledlayout(2,1)

ax(1) = nexttile;
plot(t_s, state(1:3,:), '--');
hold on
ax(1).ColorOrderIndex = 1;
plot(debug_t, traj)
hold off
xlim('tight')

title('Trajectory vs. Reference')
xlabel('Time (s)');
ylabel('Position (m)');
legend({'X_{ref}', 'Y_{ref}', 'Z_{ref}', 'X', 'Y', 'Z'}, 'Location', legLocation)
grid on

traj_rs = interp1(debug_t, traj.', t_s).';

ax(2) = nexttile;
plot(t_s, abs(traj_rs-state(1:3,:)));

yline(0.2, 'r:', 'LineWidth', 1.2)
ylim([-0.05, 0.25])
xlim('tight')

title('Trajectory Error')
xlabel('Time (s)');
ylabel('Position (m)');
legend({'X', 'Y', 'Z', 'Threshold'}, 'Location', legLocation)
grid on

linkaxes(ax, 'x')

%%
figure(4)
tiledlayout(1,1)

nexttile
plot(debug_t, debug_float(8,:))
title('Leader State')
xlabel('Time (s)');
ylabel('State (n/a)');
ylim([-0.2 2.2])
yticks([0, 1, 2])
yticklabels({'STANDBY', 'FOLLOWING', 'RETURNING'})
grid on


%%

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
