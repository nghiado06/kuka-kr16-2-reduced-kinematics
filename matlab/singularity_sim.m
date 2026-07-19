clear; clc; close all;

%% ============================================================
%  KUKA KR 16-2 Singularity Animation - Skeleton Model
%  caseID = 1: R = 0 singularity
%  caseID = 2: elbow singularity
%
%  Notes:
%  - Set loopForever = true for continuous preview until you press Stop.
%  - If saveVideo = true, loopForever is automatically disabled so that
%    the video file can be closed correctly.
%% ============================================================

caseID = 2;            % 1 = R = 0 singularity, 2 = elbow singularity
saveVideo = false;     % true = export mp4, false = preview only
loopForever = true;    % true = keep running until MATLAB Stop is pressed

%% Repository paths
scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);
resultsDir = fullfile(repoRoot, "results");
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

%% Robot geometric parameters (mm)
a1 = 260;
a2 = 680;
a3 = 22.5;
d1 = 675;
lt = 476.5;

L = hypot(a3, lt);
gamma = atan2(lt, a3);

fprintf('Effective link L = %.4f mm\n', L);
fprintf('gamma = %.4f deg\n\n', rad2deg(gamma));

if saveVideo && loopForever
    warning('saveVideo = true, so loopForever is automatically set to false.');
    loopForever = false;
end

%% Video setup
fig = figure('Color', 'w', 'Position', [100 100 1000 650]);

if saveVideo
    if caseID == 1
        videoName = fullfile(resultsDir, 'singularity_R_equal_zero.mp4');
    else
        videoName = fullfile(resultsDir, 'singularity_elbow.mp4');
    end

    v = VideoWriter(videoName, 'MPEG-4');
    v.FrameRate = 30;
    open(v);
end

%% Run selected case
switch caseID
    case 1
        animate_R_equal_zero(fig, saveVideo, loopForever, a1, a2, a3, d1, lt);

    case 2
        animate_elbow_singularity(fig, saveVideo, loopForever, a1, a2, a3, d1, lt);

    otherwise
        error('Invalid caseID. Use caseID = 1 or caseID = 2.');
end

if saveVideo
    close(v);
    fprintf('Video saved: %s\n', videoName);
end

%% ============================================================
%  CASE 1: R = 0 singularity
%% ============================================================

function animate_R_equal_zero(fig, saveVideo, loopForever, a1, a2, a3, d1, lt)

    % Selected R = 0 singular configuration.
    % theta3 is fixed at 90 deg, then theta2 is chosen so that R = 0.
    theta2 = deg2rad(-104.104135764);
    theta3 = deg2rad(90);

    R_check = a1 + a2*cos(theta2) + a3*cos(theta2 + theta3) + lt*sin(theta2 + theta3);

    fprintf('Case 1: R = 0 singularity\n');
    fprintf('theta2 = %.4f deg\n', rad2deg(theta2));
    fprintf('theta3 = %.4f deg\n', rad2deg(theta3));
    fprintf('R check = %.12f mm\n\n', R_check);

    nFrames = 180;
    zMax = 2000;
    theta1 = 0;
    dtheta1 = deg2rad(2);
    frameID = 1;

    while ishandle(fig)
        clf(fig);

        if loopForever
            theta1 = theta1 + dtheta1;
            if theta1 > 2*pi
                theta1 = theta1 - 2*pi;
            end
        else
            theta1 = 2*pi * (frameID - 1) / (nFrames - 1);
        end

        q = [theta1, theta2, theta3];

        pts3D = getRobotPoints3D(q, a1, a2, a3, d1, lt);
        pTip = pts3D.P;

        R = hypot(pTip(1), pTip(2));

        hold on; grid on; axis equal;
        view(40, 25);

        % Draw base vertical axis z0 up to the tip height.
        plot3([0 0], [0 0], [0 pTip(3)], 'r--', 'LineWidth', 1.5, ...
            'DisplayName', 'z_0 axis');

        % Draw circular trace of the tip projection.
        % For R = 0, this circle collapses to the z0 axis.
        ang = linspace(0, 2*pi, 200);
        plot3(R*cos(ang), R*sin(ang), pTip(3)*ones(size(ang)), ...
            '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, ...
            'DisplayName', 'Tip trace');

        drawRobot3D(pts3D);

        plot3(pTip(1), pTip(2), pTip(3), 'rx', ...
            'LineWidth', 2.5, 'MarkerSize', 12, ...
            'DisplayName', 'Tip');

        title({
            'Case 1: Singularity when R = 0'
            'The end-effector lies on the base vertical axis z_0'
            sprintf('theta_1 = %.1f deg, R = %.6f mm', rad2deg(theta1), R)
            }, 'FontWeight', 'bold');

        xlabel('X (mm)', 'FontWeight', 'bold');
        ylabel('Y (mm)', 'FontWeight', 'bold');
        zlabel('Z (mm)', 'FontWeight', 'bold');

        xlim([-800 800]);
        ylim([-800 800]);
        zlim([0 zMax]);

        lgd = legend('Location', 'northeastoutside');
        lgd.Position(1) = lgd.Position(1) - 0.08;

        drawnow;

        if saveVideo
            frame = getframe(fig);
            writeVideo(evalin('caller', 'v'), frame);
        end

        if ~loopForever
            frameID = frameID + 1;
            if frameID > nFrames
                break;
            end
        end
    end
end

%% ============================================================
%  CASE 2: Elbow singularity
%% ============================================================

function animate_elbow_singularity(fig, saveVideo, loopForever, a1, a2, a3, d1, lt)

    gamma = atan2(lt, a3);

    % Elbow singularity occurs at theta3 = gamma or theta3 = gamma - pi.
    % theta3_sing = gamma;
    theta3_sing = gamma - pi;

    % Keep theta1 = 0 so the robot is shown in the XZ plane.
    theta1 = 0;

    % Choose a smooth motion toward the singular configuration.
    theta2 = deg2rad(20);
    theta3_start = deg2rad(35);
    theta3List = linspace(theta3_start, theta3_sing, 160);

    fprintf('Case 2: Elbow singularity\n');
    fprintf('theta2 = %.4f deg\n', rad2deg(theta2));
    fprintf('theta3_sing = %.4f deg\n\n', rad2deg(theta3_sing));

    k = 1;
    direction = 1;

    while ishandle(fig)
        clf(fig);

        theta3 = theta3List(k);
        q = [theta1, theta2, theta3];

        pts = getRobotPointsXZ(q, a1, a2, a3, d1, lt);

        hold on; grid on; axis equal;

        drawRobotXZ(pts);

        % Effective link L from elbow point E to tip P.
        plot([pts.E(1), pts.P(1)], [pts.E(2), pts.P(2)], ...
            '--', 'Color', [0.8500 0.3250 0.0980], ...
            'LineWidth', 2.0, 'DisplayName', 'Effective link L');

        % Velocity vectors from joint 2 and joint 3.
        [v2, v3] = getVelocityDirectionsXZ(q, a2, a3, lt);

        scale = 120;
        tip = pts.P;

        quiver(tip(1), tip(2), scale*v2(1), scale*v2(2), 0, ...
            'LineWidth', 2.0, 'MaxHeadSize', 1.2, ...
            'Color', [0 0.4470 0.7410], ...
            'DisplayName', 'v from joint 2');

        quiver(tip(1), tip(2), scale*v3(1), scale*v3(2), 0, ...
            'LineWidth', 2.0, 'MaxHeadSize', 1.2, ...
            'Color', [0.8500 0.3250 0.0980], ...
            'DisplayName', 'v from joint 3');

        angleDiff = abs(rad2deg(theta3 - theta3_sing));

        if angleDiff <= 0.5
            plot(tip(1), tip(2), 'rx', 'LineWidth', 2.5, ...
                'MarkerSize', 12, 'DisplayName', 'Singular tip point');
        end

        if angleDiff > 15
            statusText = 'Regular configuration';
        elseif angleDiff > 2
            statusText = 'Approaching elbow singularity';
        else
            statusText = 'Elbow singularity: velocity directions become dependent';
        end

        title({
            'Case 2: Elbow Singularity'
            'Link a_2 and the effective link L become collinear'
            sprintf('theta_3 = %.2f deg, singular at %.2f deg', ...
            rad2deg(theta3), rad2deg(theta3_sing))
            statusText
            }, 'FontWeight', 'bold');

        xlabel('X (mm)', 'FontWeight', 'bold');
        ylabel('Z (mm)', 'FontWeight', 'bold');

        xlim([-100 1600]);
        ylim([0 1400]);

        lgd = legend('Location', 'northeastoutside');
        lgd.Position(1) = lgd.Position(1) - 0.08;

        drawnow;

        if saveVideo
            frame = getframe(fig);
            writeVideo(evalin('caller', 'v'), frame);
        end

        if loopForever
            k = k + direction;

            if k >= numel(theta3List)
                k = numel(theta3List);
                direction = -1;
            elseif k <= 1
                k = 1;
                direction = 1;
            end
        else
            k = k + 1;
            if k > numel(theta3List)
                break;
            end
        end
    end
end

%% ============================================================
%  Geometry Functions
%% ============================================================

function pts = getRobotPointsXZ(q, a1, a2, a3, d1, lt)

    theta2 = q(2);
    theta3 = q(3);
    theta23 = theta2 + theta3;

    O0 = [0, 0];
    B  = [0, d1];
    S  = [a1, d1];

    E = [
        a1 + a2*cos(theta2), ...
        d1 - a2*sin(theta2)
    ];

    W = [
        a1 + a2*cos(theta2) + a3*cos(theta23), ...
        d1 - a2*sin(theta2) - a3*sin(theta23)
    ];

    P = [
        a1 + a2*cos(theta2) + a3*cos(theta23) + lt*sin(theta23), ...
        d1 - a2*sin(theta2) - a3*sin(theta23) + lt*cos(theta23)
    ];

    pts.O0 = O0;
    pts.B  = B;
    pts.S  = S;
    pts.E  = E;
    pts.W  = W;
    pts.P  = P;
end

function pts = getRobotPoints3D(q, a1, a2, a3, d1, lt)

    theta1 = q(1);
    ptsXZ = getRobotPointsXZ(q, a1, a2, a3, d1, lt);

    names = fieldnames(ptsXZ);

    for i = 1:numel(names)
        name = names{i};

        xLocal = ptsXZ.(name)(1);
        zLocal = ptsXZ.(name)(2);

        x = xLocal*cos(theta1);
        y = xLocal*sin(theta1);
        z = zLocal;

        pts.(name) = [x, y, z];
    end
end

function drawRobotXZ(pts)

    baseWidth = 2.0;
    linkWidth = 2.0;
    jointSize = 5;

    % Base and shoulder offset.
    plot([pts.O0(1), pts.B(1)], [pts.O0(2), pts.B(2)], ...
        'k-', 'LineWidth', baseWidth, 'DisplayName', 'Base link');

    plot([pts.B(1), pts.S(1)], [pts.B(2), pts.S(2)], ...
        'k-', 'LineWidth', baseWidth, 'HandleVisibility', 'off');

    % Main links.
    plot([pts.S(1), pts.E(1)], [pts.S(2), pts.E(2)], ...
        '-', 'LineWidth', linkWidth, ...
        'Color', [0 0.4470 0.7410], ...
        'DisplayName', 'Link a_2');

    plot([pts.E(1), pts.W(1)], [pts.E(2), pts.W(2)], ...
        '-', 'LineWidth', linkWidth, ...
        'Color', [0.4660 0.6740 0.1880], ...
        'DisplayName', 'Link a_3');

    plot([pts.W(1), pts.P(1)], [pts.W(2), pts.P(2)], ...
        '-', 'LineWidth', linkWidth, ...
        'Color', [0.8500 0.3250 0.0980], ...
        'DisplayName', 'Tool offset l_t');

    % Joints.
    jointList = [pts.O0; pts.B; pts.S; pts.E; pts.W; pts.P];

    plot(jointList(:,1), jointList(:,2), 'ko', ...
        'MarkerFaceColor', 'k', ...
        'MarkerSize', jointSize, ...
        'HandleVisibility', 'off');
end

function drawRobot3D(pts)

    baseWidth = 2.0;
    linkWidth = 2.5;
    jointSize = 6;

    % Base and shoulder offset.
    plot3([pts.O0(1), pts.B(1)], [pts.O0(2), pts.B(2)], [pts.O0(3), pts.B(3)], ...
        'k-', 'LineWidth', baseWidth, 'DisplayName', 'Base link');

    plot3([pts.B(1), pts.S(1)], [pts.B(2), pts.S(2)], [pts.B(3), pts.S(3)], ...
        'k-', 'LineWidth', baseWidth, 'HandleVisibility', 'off');

    % Arm links.
    plot3([pts.S(1), pts.E(1)], [pts.S(2), pts.E(2)], [pts.S(3), pts.E(3)], ...
        '-', 'LineWidth', linkWidth, ...
        'Color', [0 0.4470 0.7410], ...
        'DisplayName', 'Link a_2');

    plot3([pts.E(1), pts.W(1)], [pts.E(2), pts.W(2)], [pts.E(3), pts.W(3)], ...
        '-', 'LineWidth', linkWidth, ...
        'Color', [0.4660 0.6740 0.1880], ...
        'DisplayName', 'Link a_3');

    plot3([pts.W(1), pts.P(1)], [pts.W(2), pts.P(2)], [pts.W(3), pts.P(3)], ...
        '-', 'LineWidth', linkWidth, ...
        'Color', [0.8500 0.3250 0.0980], ...
        'DisplayName', 'Tool offset l_t');

    % Joints.
    jointList = [
        pts.O0;
        pts.B;
        pts.S;
        pts.E;
        pts.W;
        pts.P
    ];

    plot3(jointList(:,1), jointList(:,2), jointList(:,3), 'ko', ...
        'MarkerFaceColor', 'k', ...
        'MarkerSize', jointSize, ...
        'HandleVisibility', 'off');
end

%% ============================================================
%  Singularity / Jacobian Related Functions
%% ============================================================

function theta3 = find_theta3_for_R_zero(theta2, a1, a2, a3, lt)

    % R = a1 + a2*cos(theta2) + a3*cos(theta2+theta3)
    %     + lt*sin(theta2+theta3)
    %
    % Let beta = theta2 + theta3.
    % Need: a3*cos(beta) + lt*sin(beta) = -a1 - a2*cos(theta2).

    C = -a1 - a2*cos(theta2);
    A = hypot(a3, lt);
    delta = atan2(lt, a3);

    if abs(C) > A
        theta3 = NaN;
        return;
    end

    % A*cos(beta - delta) = C.
    beta1 = delta + acos(C/A);
    beta2 = delta - acos(C/A);

    theta3_candidates = [beta1 - theta2, beta2 - theta2];

    % Choose a candidate inside a reasonable joint range if possible.
    jointMin = deg2rad(-130);
    jointMax = deg2rad(154);

    valid = theta3_candidates(theta3_candidates >= jointMin & theta3_candidates <= jointMax);

    if isempty(valid)
        theta3 = theta3_candidates(1);
    else
        theta3 = valid(1);
    end
end

function [v2_unit, v3_unit] = getVelocityDirectionsXZ(q, a2, a3, lt)

    theta2 = q(2);
    theta3 = q(3);
    theta23 = theta2 + theta3;

    % XZ plane position:
    % x = a1 + a2*cos(theta2) + a3*cos(theta23) + lt*sin(theta23)
    % z = d1 - a2*sin(theta2) - a3*sin(theta23) + lt*cos(theta23)
    %
    % Velocity direction from theta2:
    dx_dtheta2 = -a2*sin(theta2) - a3*sin(theta23) + lt*cos(theta23);
    dz_dtheta2 = -a2*cos(theta2) - a3*cos(theta23) - lt*sin(theta23);

    % Velocity direction from theta3:
    dx_dtheta3 = -a3*sin(theta23) + lt*cos(theta23);
    dz_dtheta3 = -a3*cos(theta23) - lt*sin(theta23);

    v2 = [dx_dtheta2, dz_dtheta2];
    v3 = [dx_dtheta3, dz_dtheta3];

    v2_unit = v2 / norm(v2);
    v3_unit = v3 / norm(v3);
end
