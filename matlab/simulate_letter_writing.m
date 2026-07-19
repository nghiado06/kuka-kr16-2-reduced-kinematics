%% simulate_letter_writing.m
% Simulate the robot writing the validated letter trajectory.
% Run validate_letter_trajectory.m first to create letter_trajectory_result.mat.

clear; clc; close all;

%% Repository paths
scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);

%% User parameters
resultFile = fullfile(repoRoot, "data", "letter_trajectory_result.mat");
urdfFile   = fullfile(repoRoot, "models", "kuka_robot_model", "urdf", "robot_fixed.urdf");
meshPath   = fullfile(repoRoot, "models", "kuka_robot_model", "meshes_linkframe");
endEffectorName = "end_tip";

branchName = "up";       % "up" or "down"
animationStep = 2;        % larger value = faster animation
pauseTime = 0.015;

% Wider plot space
viewPadXY = 0.55;
viewPadZ  = 0.75;

% Mapping from analytical DH variables [theta1 theta2 theta3]
% to URDF joint variables [q1 q2 q3].
% These offsets match the current visualization convention where
% analytical [0 0 0] is shown in URDF approximately as [0 -90 180] deg.
qSign   = [1, 1, 1];
qOffset = deg2rad([0, 0, 0]);
fixedJoints456 = deg2rad([0, 0, 0]);

%% Load validated trajectory
if ~isfile(resultFile)
    error("Result file not found. Run validate_letter_trajectory.m first.");
end

load(resultFile, "traj", "result", "metadata");

if ~result.isValid
    warning("Loaded trajectory is not fully valid. Animation will still run for available points.");
end

switch lower(branchName)
    case "up"
        qDH = result.qUp;
    case "down"
        qDH = result.qDown;
    otherwise
        error("branchName must be 'up' or 'down'.");
end

%% Import robot
robot = importrobot(urdfFile, ...
    "DataFormat", "row", ...
    "MeshPath", {meshPath});
robot.Gravity = [0 0 -9.81];

nJoints = numel(homeConfiguration(robot));
if nJoints < 3
    error("The imported robot must have at least 3 joints.");
end

qRobot = zeros(size(qDH, 1), nJoints);
qRobot(:, 1:3) = qDH.*qSign + qOffset;

if nJoints >= 6
    qRobot(:, 4:6) = repmat(fixedJoints456, size(qRobot, 1), 1);
end

%% Figure setup
fig = figure('Color', 'w', 'Name', 'Robot letter-writing simulation');
ax = axes('Parent', fig);

uicontrol(fig, ...
    'Style', 'pushbutton', ...
    'String', 'Restart', ...
    'FontSize', 10, ...
    'Units', 'normalized', ...
    'Position', [0.86, 0.93, 0.10, 0.045], ...
    'Callback', @(~,~) runAnimation(robot, qRobot, traj, endEffectorName, ...
                                    animationStep, pauseTime, branchName, ...
                                    ax, viewPadXY, viewPadZ));

runAnimation(robot, qRobot, traj, endEffectorName, animationStep, pauseTime, ...
             branchName, ax, viewPadXY, viewPadZ);

fprintf("\nSimulation finished using elbow-%s branch.\n", branchName);
fprintf("Base point A = [%.4f %.4f %.4f] m\n", metadata.basePointA);
fprintf("Letter size  = %.4f m\n", metadata.letterSize);

%% Local functions
function runAnimation(robot, qRobot, traj, endEffectorName, animationStep, pauseTime, ...
                      branchName, ax, viewPadXY, viewPadZ)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');
    view(ax, -135, 25);
    camproj(ax, 'orthographic');

    xlabel(ax, 'X_0 (m)');
    ylabel(ax, 'Y_0 (m)');
    zlabel(ax, 'Z_0 (m)');
    title(ax, sprintf('Letter writing simulation - elbow %s', branchName));

    setWideView(ax, traj.xyz, viewPadXY, viewPadZ);

    show(robot, qRobot(1, :), ...
        "Frames", "off", ...
        "Visuals", "on", ...
        "Parent", ax);

    camlight(ax);
    lighting(ax, 'flat');
    material(ax, 'dull');

    plotTargetPath(ax, traj);

    trace = animatedline(ax, 'LineWidth', 2.0);
    currentTip = scatter3(ax, nan, nan, nan, 60, 'filled');

    legend(ax, 'Target drawing path', 'Pen-up transition', ...
        'Actual traced path', 'Current tip', 'Location', 'bestoutside');

    for i = 1:animationStep:size(qRobot, 1)
        show(robot, qRobot(i, :), ...
            "Frames", "off", ...
            "Visuals", "on", ...
            "Parent", ax, ...
            "PreservePlot", false, ...
            "FastUpdate", true);

        T = getTransform(robot, qRobot(i, :), endEffectorName);
        pTip = T(1:3, 4).';

        if traj.isDraw(i)
            addpoints(trace, pTip(1), pTip(2), pTip(3));
        end

        currentTip.XData = pTip(1);
        currentTip.YData = pTip(2);
        currentTip.ZData = pTip(3);

        drawnow limitrate;
        pause(pauseTime);
    end
end

function setWideView(ax, xyz, viewPadXY, viewPadZ)
    viewPts = [xyz; 0 0 0];

    xlim(ax, [min(viewPts(:, 1)) - viewPadXY, max(viewPts(:, 1)) + viewPadXY]);
    ylim(ax, [min(viewPts(:, 2)) - viewPadXY, max(viewPts(:, 2)) + viewPadXY]);
    zlim(ax, [0, max(1.50, max(viewPts(:, 3)) + viewPadZ)]);
end

function plotTargetPath(ax, traj)
    drawMask = logical(traj.isDraw(:));
    xyz = traj.xyz;

    % Plot drawing strokes separately
    strokeList = unique(traj.strokeId(traj.strokeId > 0)).';
    for sid = strokeList
        idx = traj.strokeId == sid & drawMask;
        plot3(ax, xyz(idx, 1), xyz(idx, 2), xyz(idx, 3), ...
            'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end

    % Plot pen-up transitions separately
    moveMask = ~drawMask;
    moveStart = find(diff([false; moveMask]) == 1);
    moveEnd   = find(diff([moveMask; false]) == -1);

    for k = 1:numel(moveStart)
        idx = moveStart(k):moveEnd(k);

        if k == 1
            handleVisibility = 'on';
        else
            handleVisibility = 'off';
        end

        plot3(ax, xyz(idx, 1), xyz(idx, 2), xyz(idx, 3), ...
            '--', ...
            'LineWidth', 0.8, ...
            'DisplayName', 'Pen-up transition', ...
            'HandleVisibility', handleVisibility);
    end

    plot3(ax, nan, nan, nan, ...
        'LineWidth', 1.2, ...
        'DisplayName', 'Target drawing path');
end