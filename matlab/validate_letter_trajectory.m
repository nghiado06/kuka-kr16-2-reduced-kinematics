%% validate_letter_trajectory.m
% Validate a letter-writing trajectory for the first 3 joints of KUKA KR 16-2.
% Flow:
% 1) Parameterize letters in a local writing frame
% 2) Map local points to the plane z = planeZ in the base frame
% 3) Check workspace feasibility
% 4) Solve analytical IK for elbow-up and elbow-down branches
% 5) Check joint limits
% 6) Plot trajectory in workspace and both C-space branches
% 7) Save validated result for simulation

clear; clc; close all;

%% Repository paths
scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);
dataDir   = fullfile(repoRoot, "data");
if ~isfolder(dataDir)
    mkdir(dataDir);
end

%% User parameters
basePointA   = [-0.26, 0.48, 0.77];   % [xA, yA, z0] in meters
letterSize   = 0.12;                  % scale of the letters in meters
pointsPerSeg = 20;                    % interpolation points per line segment

saveFileName = fullfile(dataDir, "letter_trajectory_result.mat");

%% Robot geometric parameters from the DH model
geom.a1 = 0.260;
geom.a2 = 0.680;
geom.a3 = 0.0225;
geom.d1 = 0.675;
% geom.lt = 0.4765;
geom.lt = 0.5000;
geom.leq = hypot(geom.a3, geom.lt);
geom.gamma = atan2(geom.lt, geom.a3);

% Joint limits for analytical variables [theta1 theta2 theta3], in radians.
% Adjust here if your analytical variables use a different offset convention.
jointLimits = deg2rad([
    -185,  185;
    -155,   35;
    -130,  154
]);

%% Generate trajectory
[traj, strokeInfo, keyPoints] = buildLetterTrajectory(basePointA, letterSize, pointsPerSeg);

%% Validate trajectory
[result, report] = validateTrajectory(traj, geom, jointLimits);

printValidationReport(report);

if ~result.isValid
    warning("Trajectory is NOT valid. Adjust basePointA or letterSize and run again.");
else
    fprintf("\nTrajectory is valid. Result saved to: %s\n", saveFileName);
end

%% Plot workspace and C-space
plotTrajectoryInWorkspace(traj, result, geom, basePointA(3), keyPoints);
plotCSpaceBranches(result, jointLimits);

%% Save result for simulation
metadata.basePointA = basePointA;
metadata.letterSize = letterSize;
metadata.pointsPerSeg = pointsPerSeg;
metadata.geom = geom;
metadata.jointLimits = jointLimits;
metadata.strokeInfo = strokeInfo;
metadata.keyPoints = keyPoints;
metadata.createdBy = "validate_letter_trajectory_v2.m";

save(saveFileName, "traj", "result", "metadata");

%% Local functions
function [traj, strokeInfo, keyPoints] = buildLetterTrajectory(basePointA, letterSize, pointsPerSeg)
    s = letterSize;
    z0 = basePointA(3);

    % Local 2D points. First coordinate maps to base X, second to base Y.
    P.A = [0,     0];
    P.B = [0,     s];
    P.C = [s/2,   s/2];
    P.D = [s,     s];
    P.E = [s,     0];

    P.F = [8*s/7,  0];
    P.G = [8*s/7,  s];
    P.H = [15*s/7, 0];
    P.I = [15*s/7, s];

    P.J = [16*s/7, 0];
    P.K = [16*s/7, s];
    P.L = [23*s/7, 0];
    P.M = [23*s/7, s];
    P.N = [24*s/7, s];

    P.O = [29*s/7, s];
    P.P = [31*s/7, 5*s/7];
    P.Q = [31*s/7, 2*s/7];
    P.R = [29*s/7, 0];
    P.S = [24*s/7, 0];

    strokes = {
        ["A", "B", "C", "D", "E"]
        ["F", "G", "H", "I"]
        ["J", "K", "L", "M"]
        ["N", "O", "P", "Q", "R", "S", "N"]
    };

    xyz = [];
    isDraw = [];
    strokeId = [];

    for k = 1:numel(strokes)
        names = strokes{k};
        localPts = zeros(numel(names), 2);

        for i = 1:numel(names)
            localPts(i, :) = P.(names(i));
        end

        % Add a pen-up transition from previous stroke end to current stroke start.
        if ~isempty(xyz)
            pLast = xyz(end, :);
            pNext = localToBase(localPts(1, :), basePointA, z0);
            movePts = interpolateLine(pLast, pNext, max(2, round(pointsPerSeg/2)));
            xyz = [xyz; movePts(2:end, :)]; %#ok<AGROW>
            isDraw = [isDraw; false(size(movePts, 1)-1, 1)]; %#ok<AGROW>
            strokeId = [strokeId; zeros(size(movePts, 1)-1, 1)]; %#ok<AGROW>
        end

        for i = 1:size(localPts, 1)-1
            p1 = localToBase(localPts(i, :), basePointA, z0);
            p2 = localToBase(localPts(i+1, :), basePointA, z0);
            segPts = interpolateLine(p1, p2, pointsPerSeg);

            if isempty(xyz)
                idxStart = 1;
            else
                idxStart = 2;
            end

            xyz = [xyz; segPts(idxStart:end, :)]; %#ok<AGROW>
            isDraw = [isDraw; true(size(segPts, 1)-idxStart+1, 1)]; %#ok<AGROW>
            strokeId = [strokeId; k*ones(size(segPts, 1)-idxStart+1, 1)]; %#ok<AGROW>
        end
    end

    traj.xyz = xyz;
    traj.isDraw = logical(isDraw);
    traj.strokeId = strokeId;

    strokeInfo.strokes = strokes;
    strokeInfo.pointNames = string(fieldnames(P));

    names = fieldnames(P);
    for i = 1:numel(names)
        keyPoints.(names{i}) = localToBase(P.(names{i}), basePointA, z0);
    end
end

function p0 = localToBase(pLocal, basePointA, z0)
    p0 = [basePointA(1) + pLocal(1), basePointA(2) + pLocal(2), z0];
end

function pts = interpolateLine(p1, p2, n)
    t = linspace(0, 1, n).';
    pts = p1 + (p2 - p1).*t;
end

function [result, report] = validateTrajectory(traj, geom, jointLimits)
    n = size(traj.xyz, 1);

    qUp = nan(n, 3);
    qDown = nan(n, 3);
    workspaceOK = false(n, 1);
    ikUpOK = false(n, 1);
    ikDownOK = false(n, 1);
    limitUpOK = false(n, 1);
    limitDownOK = false(n, 1);

    for i = 1:n
        p = traj.xyz(i, :);
        workspaceOK(i) = isInsideWorkspace(p, geom);

        [qUp(i, :), ikUpOK(i)] = solveIK3R(p, geom, +1);
        [qDown(i, :), ikDownOK(i)] = solveIK3R(p, geom, -1);

        limitUpOK(i) = ikUpOK(i) && isWithinLimits(qUp(i, :), jointLimits);
        limitDownOK(i) = ikDownOK(i) && isWithinLimits(qDown(i, :), jointLimits);
    end

    result.qUp = qUp;
    result.qDown = qDown;
    result.workspaceOK = workspaceOK;
    result.ikUpOK = ikUpOK;
    result.ikDownOK = ikDownOK;
    result.limitUpOK = limitUpOK;
    result.limitDownOK = limitDownOK;

    result.validUp = workspaceOK & ikUpOK & limitUpOK;
    result.validDown = workspaceOK & ikDownOK & limitDownOK;
    result.isValid = all(result.validUp) && all(result.validDown);

    report.numPoints = n;
    report.numWorkspaceFail = nnz(~workspaceOK);
    report.numIKUpFail = nnz(~ikUpOK);
    report.numIKDownFail = nnz(~ikDownOK);
    report.numLimitUpFail = nnz(ikUpOK & ~limitUpOK);
    report.numLimitDownFail = nnz(ikDownOK & ~limitDownOK);
    report.firstFailWorkspace = find(~workspaceOK, 1, "first");
    report.firstFailUp = find(~result.validUp, 1, "first");
    report.firstFailDown = find(~result.validDown, 1, "first");
end

function ok = isInsideWorkspace(p, geom)
    rho = hypot(p(1), p(2));
    h = geom.d1 - p(3);
    radialDist = hypot(rho - geom.a1, h);

    rMin = abs(geom.a2 - geom.leq);
    rMax = geom.a2 + geom.leq;
    tol = 1e-9;

    ok = radialDist >= rMin - tol && radialDist <= rMax + tol;
end

function [q, ok] = solveIK3R(p, geom, branchSign)
    x = p(1); y = p(2); z = p(3);

    theta1 = atan2(y, x);
    rho = hypot(x, y);
    r = rho - geom.a1;
    h = geom.d1 - z;

    D = (r^2 + h^2 - geom.a2^2 - geom.leq^2)/(2*geom.a2*geom.leq);
    tol = 1e-9;

    if D < -1 - tol || D > 1 + tol
        q = [nan, nan, nan];
        ok = false;
        return;
    end

    D = min(1, max(-1, D));
    sinPhi = branchSign*sqrt(max(0, 1 - D^2));
    phi = atan2(sinPhi, D);

    theta2 = atan2(h, r) - atan2(geom.leq*sin(phi), geom.a2 + geom.leq*cos(phi));
    theta3 = phi + geom.gamma;

    q = wrapToPiLocal([theta1, theta2, theta3]);
    ok = all(isfinite(q));
end

function ok = isWithinLimits(q, limits)
    ok = all(q(:) >= limits(:, 1) - 1e-9 & q(:) <= limits(:, 2) + 1e-9);
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

function printValidationReport(report)
    fprintf("\n========== Trajectory validation report ==========" + newline);
    fprintf("Total points           : %d\n", report.numPoints);
    fprintf("Workspace failures     : %d\n", report.numWorkspaceFail);
    fprintf("IK elbow-up failures   : %d\n", report.numIKUpFail);
    fprintf("IK elbow-down failures : %d\n", report.numIKDownFail);
    fprintf("Limit elbow-up failures: %d\n", report.numLimitUpFail);
    fprintf("Limit elbow-down fail. : %d\n", report.numLimitDownFail);

    if ~isempty(report.firstFailWorkspace)
        fprintf("First workspace fail index: %d\n", report.firstFailWorkspace);
    end
    if ~isempty(report.firstFailUp)
        fprintf("First elbow-up fail index : %d\n", report.firstFailUp);
    end
    if ~isempty(report.firstFailDown)
        fprintf("First elbow-down fail idx : %d\n", report.firstFailDown);
    end
end

function plotTrajectoryInWorkspace(traj, result, geom, planeZ, keyPoints)
    figure('Color', 'w', 'Name', 'Trajectory in workspace');
    hold on; axis equal; grid on;

    drawMask = logical(traj.isDraw(:));
    trajXYZ = traj.xyz;

    xRange = linspace(-1.6, 1.6, 450);
    yRange = linspace(-1.6, 1.6, 450);
    [X, Y] = meshgrid(xRange, yRange);
    Rho = hypot(X, Y);
    hPlane = geom.d1 - planeZ;
    radialDist = hypot(Rho - geom.a1, hPlane);

    rMin = abs(geom.a2 - geom.leq);
    rMax = geom.a2 + geom.leq;
    workspaceMask = radialDist >= rMin & radialDist <= rMax;

    contourf(X, Y, double(workspaceMask), [0.5 0.5], ...
        'LineStyle', 'none', 'FaceAlpha', 0.16, 'HandleVisibility', 'off');

    % Plot every stroke separately, so pen-up segments do not connect letters visually.
    strokeList = unique(traj.strokeId(traj.strokeId > 0)).';
    for sid = strokeList
        idx = traj.strokeId == sid & drawMask;
        plot(trajXYZ(idx, 1), trajXYZ(idx, 2), 'k-', 'LineWidth', 2.0, ...
            'HandleVisibility', 'off');
    end

    % Plot pen-up transitions as separate segments.
    % This avoids MATLAB connecting E->F, I->J, M->N into one long dashed polyline.
    plotPenUpTransitions(trajXYZ, drawMask);
    plot(nan, nan, 'k-', 'LineWidth', 2.0, 'DisplayName', 'Drawing trajectory');
    scatter(trajXYZ(1, 1), trajXYZ(1, 2), 70, 'filled', 'DisplayName', 'Start');
    scatter(trajXYZ(end, 1), trajXYZ(end, 2), 70, 'filled', 'DisplayName', 'End');

    failIdx = ~(result.validUp & result.validDown);
    if any(failIdx)
        scatter(trajXYZ(failIdx, 1), trajXYZ(failIdx, 2), 20, 'x', ...
            'DisplayName', 'Invalid points');
    end

    names = fieldnames(keyPoints);
    for i = 1:numel(names)
        p = keyPoints.(names{i});
        text(p(1), p(2), " " + names{i}, 'FontWeight', 'bold', 'FontSize', 9);
    end

    % Zoom to include both workspace and letter trajectory.
    xlim([min(-1.5, min(trajXYZ(:,1)) - 0.08), max(1.5, max(trajXYZ(:,1)) + 0.08)]);
    ylim([min(-1.5, min(trajXYZ(:,2)) - 0.08), max(1.5, max(trajXYZ(:,2)) + 0.08)]);

    xlabel('X_0 (m)');
    ylabel('Y_0 (m)');
    title(sprintf('Letter trajectory on workspace section z = %.3f m', planeZ));
    legend('Location', 'bestoutside');
end

function plotCSpaceBranches(result, jointLimits) %#ok<INUSD>
    figure('Color', 'w', 'Name', 'C-space branches');

    qUpDeg = rad2deg(result.qUp);
    qDownDeg = rad2deg(result.qDown);
    tickStep = 20;
    minAxisSpan = 80;

    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot3(qUpDeg(:, 1), qUpDeg(:, 2), qUpDeg(:, 3), 'LineWidth', 1.8); hold on;
    scatter3(qUpDeg(1, 1), qUpDeg(1, 2), qUpDeg(1, 3), 60, 'filled');
    scatter3(qUpDeg(end, 1), qUpDeg(end, 2), qUpDeg(end, 3), 60, 'filled');
    formatCSpaceAxes(qUpDeg, tickStep, minAxisSpan);
    xlabel('\theta_1 (deg)'); ylabel('\theta_2 (deg)'); zlabel('\theta_3 (deg)');
    title('Elbow-up branch in C-space');
    legend('q path', 'Start', 'End', 'Location', 'best');

    nexttile;
    plot3(qDownDeg(:, 1), qDownDeg(:, 2), qDownDeg(:, 3), 'LineWidth', 1.8); hold on;
    scatter3(qDownDeg(1, 1), qDownDeg(1, 2), qDownDeg(1, 3), 60, 'filled');
    scatter3(qDownDeg(end, 1), qDownDeg(end, 2), qDownDeg(end, 3), 60, 'filled');
    formatCSpaceAxes(qDownDeg, tickStep, minAxisSpan);
    xlabel('\theta_1 (deg)'); ylabel('\theta_2 (deg)'); zlabel('\theta_3 (deg)');
    title('Elbow-down branch in C-space');
    legend('q path', 'Start', 'End', 'Location', 'best');
end

function plotPenUpTransitions(trajXYZ, drawMask)
    moveMask = ~drawMask;

    if ~any(moveMask)
        return;
    end

    moveStart = find(diff([false; moveMask]) == 1);
    moveEnd = find(diff([moveMask; false]) == -1);

    for k = 1:numel(moveStart)
        idx = moveStart(k):moveEnd(k);

        if k == 1
            labelMode = 'on';
        else
            labelMode = 'off';
        end

        plot(trajXYZ(idx, 1), trajXYZ(idx, 2), '--', ...
            'Color', [0.35 0.35 0.35], ...
            'LineWidth', 0.9, ...
            'DisplayName', 'Pen-up transition', ...
            'HandleVisibility', labelMode);
    end
end

function formatCSpaceAxes(qDeg, tickStep, minAxisSpan)
    validRows = all(isfinite(qDeg), 2);
    qDeg = qDeg(validRows, :);

    if isempty(qDeg)
        grid on;
        return;
    end

    qMin = min(qDeg, [], 1);
    qMax = max(qDeg, [], 1);

    qCenter = (qMin + qMax) / 2;

    dataSpan = max(qMax - qMin);
    axisSpan = max(minAxisSpan, dataSpan + 2*tickStep);
    axisSpan = ceil(axisSpan / tickStep) * tickStep;

    xLim = qCenter(1) + [-axisSpan/2, axisSpan/2];
    yLim = qCenter(2) + [-axisSpan/2, axisSpan/2];
    zLim = qCenter(3) + [-axisSpan/2, axisSpan/2];

    xTickStart = tickStep * floor(xLim(1) / tickStep);
    xTickEnd   = tickStep * ceil(xLim(2) / tickStep);

    yTickStart = tickStep * floor(yLim(1) / tickStep);
    yTickEnd   = tickStep * ceil(yLim(2) / tickStep);

    zTickStart = tickStep * floor(zLim(1) / tickStep);
    zTickEnd   = tickStep * ceil(zLim(2) / tickStep);

    xlim(xLim);
    ylim(yLim);
    zlim(zLim);

    xticks(xTickStart:tickStep:xTickEnd);
    yticks(yTickStart:tickStep:yTickEnd);
    zticks(zTickStart:tickStep:zTickEnd);

    grid on;
    box on;

    daspect([1 1 1]);
    pbaspect([1 1 1]);
    axis vis3d;
    camproj('orthographic');

    view(45, 25);
end