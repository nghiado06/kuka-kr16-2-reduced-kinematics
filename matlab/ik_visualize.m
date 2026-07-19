clear; clc; close all;

%% Robot geometric parameters (mm)
a1 = 260;
a2 = 680;
a3 = 22.5;
d1 = 675;
lt = 476.5;

%% Target point (choose cases with theta1 = 0 => y = 0)
target = [1267.144, 0, 675.000];   % [x, y, z] in mm

if abs(target(2)) > 1e-9
    error('This plotting script is intended for theta1 = 0 cases, so y must be 0.');
end

%% Solve IK for elbow-up and elbow-down
[qUp, qDown] = solveIK_theta1_zero(target, a1, a2, a3, d1, lt);

fprintf('Elbow-up solution:\n');
fprintf('theta1 = %.6f deg\n', rad2deg(qUp(1)));
fprintf('theta2 = %.6f deg\n', rad2deg(qUp(2)));
fprintf('theta3 = %.6f deg\n\n', rad2deg(qUp(3)));

fprintf('Elbow-down solution:\n');
fprintf('theta1 = %.6f deg\n', rad2deg(qDown(1)));
fprintf('theta2 = %.6f deg\n', rad2deg(qDown(2)));
fprintf('theta3 = %.6f deg\n\n', rad2deg(qDown(3)));

%% Verify FK
pUp = forwardKinematics(qUp, a1, a2, a3, d1, lt);
pDown = forwardKinematics(qDown, a1, a2, a3, d1, lt);

errUp = norm(pUp - target);
errDown = norm(pDown - target);

fprintf('FK verification error:\n');
fprintf('Elbow-up   error = %.12f mm\n', errUp);
fprintf('Elbow-down error = %.12f mm\n', errDown);

%% Plot two subfigures
figure('Color', 'w');
tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

nexttile;
plotRobotSkeletonXZ(qUp, target, a1, a2, a3, d1, lt, 'Elbow-up configuration');

nexttile;
plotRobotSkeletonXZ(qDown, target, a1, a2, a3, d1, lt, 'Elbow-down configuration');

%% Optional: overlay both configurations in one figure
figure('Color', 'w'); hold on; grid on; axis equal;
plotRobotSkeletonXZ_overlay(qUp, target, a1, a2, a3, d1, lt, [0 0.4470 0.7410], 'Elbow-up');
plotRobotSkeletonXZ_overlay(qDown, target, a1, a2, a3, d1, lt, [0.8500 0.3250 0.0980], 'Elbow-down');

xlabel('X (mm)', 'FontWeight', 'bold');
ylabel('Z (mm)', 'FontWeight', 'bold');
title('IK verification using stick-link robot model');
legend('Location','best');

xlim([-100, 1600]);
ylim([0, 1400]);


%% ========================= Local Functions =========================

function [qUp, qDown] = solveIK_theta1_zero(target, a1, a2, a3, d1, lt)
    x = target(1);
    y = target(2);
    z = target(3);

    theta1 = atan2(y, x);

    rho = hypot(x, y);
    r = rho - a1;
    h = d1 - z;

    L = hypot(a3, lt);
    gamma = atan2(lt, a3);

    D = (r^2 + h^2 - a2^2 - L^2) / (2*a2*L);

    if abs(D) > 1
        error('Target is outside the reachable workspace.');
    end

    D = max(-1, min(1, D));

    phiUp = atan2( sqrt(1 - D^2), D );
    phiDown = atan2(-sqrt(1 - D^2), D );

    theta2Up = atan2(h, r) - atan2(L*sin(phiUp), a2 + L*cos(phiUp));
    theta2Down = atan2(h, r) - atan2(L*sin(phiDown), a2 + L*cos(phiDown));

    theta3Up = phiUp + gamma;
    theta3Down = phiDown + gamma;

    qUp = [theta1, theta2Up, theta3Up];
    qDown = [theta1, theta2Down, theta3Down];
end


function p = forwardKinematics(q, a1, a2, a3, d1, lt)
    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);

    c1 = cos(theta1);
    s1 = sin(theta1);
    c2 = cos(theta2);
    s2 = sin(theta2);
    c23 = cos(theta2 + theta3);
    s23 = sin(theta2 + theta3);

    x = c1 * (a1 + a2*c2 + a3*c23 + lt*s23);
    y = s1 * (a1 + a2*c2 + a3*c23 + lt*s23);
    z = d1 - a2*s2 - a3*s23 + lt*c23;

    p = [x, y, z];
end


function plotRobotSkeletonXZ(q, target, a1, a2, a3, d1, lt, figTitle)
    hold on; grid on; axis equal;

    pts = getRobotPointsXZ(q, a1, a2, a3, d1, lt);

    O0 = pts.O0;
    B  = pts.B;
    S  = pts.S;
    E  = pts.E;
    W  = pts.W;
    P  = pts.P;

    linkWidth = 1.8;
    baseWidth = 2.0;
    
    plot([O0(1), B(1)], [O0(2), B(2)], 'k-', 'LineWidth', baseWidth);
    plot([B(1), S(1)], [B(2), S(2)], 'k-', 'LineWidth', baseWidth);
    
    plot([S(1), E(1)], [S(2), E(2)], '-', 'LineWidth', linkWidth, 'Color', [0 0.4470 0.7410]);
    plot([E(1), W(1)], [E(2), W(2)], '-', 'LineWidth', linkWidth, 'Color', [0.4660 0.6740 0.1880]);
    plot([W(1), P(1)], [W(2), P(2)], '-', 'LineWidth', linkWidth, 'Color', [0.8500 0.3250 0.0980]);

    % Joints
    plot(O0(1), O0(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot(B(1),  B(2),  'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot(S(1),  S(2),  'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot(E(1),  E(2),  'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot(W(1),  W(2),  'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot(P(1),  P(2),  'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

    % Target point
    plot(target(1), target(3), 'rx', 'LineWidth', 2.5, 'MarkerSize', 10);

    % Optional dashed line from shoulder to target
    plot([S(1), target(1)], [S(2), target(3)], '--', ...
        'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);

    % Labels
    text(target(1)+15, target(3)+15, 'Target', 'FontSize', 10, 'FontWeight', 'bold');
    text(P(1)+15, P(2)-25, 'Tip', 'FontSize', 10, 'FontWeight', 'bold');

    xlabel('X (mm)', 'FontWeight', 'bold');
    ylabel('Z (mm)', 'FontWeight', 'bold');
    title(figTitle, 'FontWeight', 'bold');

    xlim([-100, 1600]);
    ylim([0, 1400]);
end


function plotRobotSkeletonXZ_overlay(q, target, a1, a2, a3, d1, lt, lineColor, labelName)
    pts = getRobotPointsXZ(q, a1, a2, a3, d1, lt);

    O0 = pts.O0;
    B  = pts.B;
    S  = pts.S;
    E  = pts.E;
    W  = pts.W;
    P  = pts.P;

    plot([O0(1), B(1)], [O0(2), B(2)], 'k-', 'LineWidth', 2, 'HandleVisibility','off');
    plot([B(1), S(1)], [B(2), S(2)], 'k-', 'LineWidth', 2, 'HandleVisibility','off');

    plot([S(1), E(1), W(1), P(1)], [S(2), E(2), W(2), P(2)], ...
        '-', 'LineWidth', 3, 'Color', lineColor, 'DisplayName', labelName);

    plot([S(1), target(1)], [S(2), target(3)], '--', ...
        'Color', lineColor, 'LineWidth', 1.0, 'HandleVisibility','off');

    plot(target(1), target(3), 'rx', 'LineWidth', 2.5, 'MarkerSize', 10, ...
        'DisplayName', 'Target');

    plot(P(1), P(2), 'o', 'Color', lineColor, 'MarkerFaceColor', lineColor, ...
        'MarkerSize', 6, 'HandleVisibility','off');
end


function pts = getRobotPointsXZ(q, a1, a2, a3, d1, lt)
    theta2 = q(2);
    theta3 = q(3);
    theta23 = theta2 + theta3;

    O0 = [0, 0];
    B  = [0, d1];
    S  = [a1, d1];

    E = [ ...
        a1 + a2*cos(theta2), ...
        d1 - a2*sin(theta2)];

    W = [ ...
        a1 + a2*cos(theta2) + a3*cos(theta23), ...
        d1 - a2*sin(theta2) - a3*sin(theta23)];

    P = [ ...
        a1 + a2*cos(theta2) + a3*cos(theta23) + lt*sin(theta23), ...
        d1 - a2*sin(theta2) - a3*sin(theta23) + lt*cos(theta23)];

    pts.O0 = O0;
    pts.B  = B;
    pts.S  = S;
    pts.E  = E;
    pts.W  = W;
    pts.P  = P;
end