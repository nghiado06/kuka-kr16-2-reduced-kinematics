%% plot_time_response_overview_only.m
% Plot only the overview figure:
%   1) Joint-variable variation versus time
%   2) End-effector position variation versus time
%
% Run validate_letter_trajectory.m first to create letter_trajectory_result.mat.

clear; clc; close all;

%% Repository paths
scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);

%% User parameters
resultFile = fullfile(repoRoot, "data", "letter_trajectory_result.mat");

branchName = "up";          % "up" or "down"
sampleTime = 0.02;          % time step between two trajectory points [s]
angleUnit  = "deg";         % "deg" or "rad"
positionUnit = "m";         % "m" or "mm"

saveFigure = false;         % true = export PNG figure

%% Load validated trajectory
if ~isfile(resultFile)
    error("Result file not found. Run validate_letter_trajectory.m first.");
end

load(resultFile, "traj", "result", "metadata");

if ~isfield(result, "qUp") || ~isfield(result, "qDown")
    error("The result file must contain result.qUp and result.qDown.");
end

if ~isfield(traj, "xyz")
    error("The trajectory must contain traj.xyz.");
end

if isfield(result, "isValid") && ~result.isValid
    warning("Loaded trajectory is not fully valid. Plot will still be generated for available points.");
end

%% Create time vector
nPts = size(traj.xyz, 1);
t = (0:nPts-1).' * sampleTime;

%% Select branch
switch lower(branchName)
    case "up"
        q = result.qUp;
        branchTitle = "Elbow-up";

    case "down"
        q = result.qDown;
        branchTitle = "Elbow-down";

    otherwise
        error("branchName must be 'up' or 'down'.");
end

%% Convert data
qPlot = convertAngle(q, angleUnit);
pPlot = convertPosition(traj.xyz, positionUnit);

angleLabel = makeAngleLabel(angleUnit);
positionLabel = makePositionLabel(positionUnit);

if isfield(traj, "isDraw")
    drawMask = logical(traj.isDraw(:));
else
    drawMask = true(nPts, 1);
end

penUpMask = ~drawMask;

%% Plot overview only
fig = figure('Color', 'w', 'Name', branchTitle + " time response overview");
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

% -------------------------------------------------------------------------
% Joint variables versus time
% -------------------------------------------------------------------------
ax1 = nexttile;
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');

plotPenUpRegions(ax1, t, qPlot(:), penUpMask);

plot(ax1, t, qPlot(:, 1), 'LineWidth', 1.5, 'DisplayName', '\theta_1');
plot(ax1, t, qPlot(:, 2), 'LineWidth', 1.5, 'DisplayName', '\theta_2');
plot(ax1, t, qPlot(:, 3), 'LineWidth', 1.5, 'DisplayName', '\theta_3');

setTimeAxis(ax1, t);

ylabel(ax1, "Joint angle (" + angleLabel + ")", 'FontWeight', 'bold');
title(ax1, branchTitle + " joint variables and end-effector position versus time", ...
    'FontWeight', 'bold', 'FontSize', 14);
legend(ax1, 'Location', 'best');
set(ax1, 'XTickLabel', []);

% -------------------------------------------------------------------------
% End-effector position versus time
% -------------------------------------------------------------------------
ax2 = nexttile;
hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');

plotPenUpRegions(ax2, t, pPlot(:), penUpMask);

plot(ax2, t, pPlot(:, 1), 'LineWidth', 1.5, 'DisplayName', 'x');
plot(ax2, t, pPlot(:, 2), 'LineWidth', 1.5, 'DisplayName', 'y');
plot(ax2, t, pPlot(:, 3), 'LineWidth', 1.5, 'DisplayName', 'z');

setTimeAxis(ax2, t);

xlabel(ax2, 'Time (s)', 'FontWeight', 'bold');
ylabel(ax2, "Position (" + positionLabel + ")", 'FontWeight', 'bold');
legend(ax2, 'Location', 'best');

if saveFigure
    fileName = lower(strrep(branchTitle, '-', '_')) + "_time_response_overview.png";
    exportgraphics(fig, fileName, 'Resolution', 300);
    fprintf("Saved figure: %s\n", fileName);
end

fprintf("\nOverview time-response plotting finished.\n");
fprintf("Result file : %s\n", resultFile);
fprintf("Branch      : %s\n", branchName);
fprintf("Sample time : %.4f s\n", sampleTime);
fprintf("Total time  : %.4f s\n", t(end));

if exist("metadata", "var") && isfield(metadata, "basePointA")
    fprintf("Base point A = [%.4f %.4f %.4f] m\n", metadata.basePointA);
end

%% ========================================================================
%  Local functions
%% ========================================================================

function qPlot = convertAngle(q, angleUnit)
    switch lower(angleUnit)
        case "deg"
            qPlot = rad2deg(q);
        case "rad"
            qPlot = q;
        otherwise
            error("angleUnit must be 'deg' or 'rad'.");
    end
end

function pPlot = convertPosition(p, positionUnit)
    switch lower(positionUnit)
        case "m"
            pPlot = p;
        case "mm"
            pPlot = 1000*p;
        otherwise
            error("positionUnit must be 'm' or 'mm'.");
    end
end

function labelText = makeAngleLabel(angleUnit)
    switch lower(angleUnit)
        case "deg"
            labelText = "deg";
        case "rad"
            labelText = "rad";
        otherwise
            error("angleUnit must be 'deg' or 'rad'.");
    end
end

function labelText = makePositionLabel(positionUnit)
    switch lower(positionUnit)
        case "m"
            labelText = "m";
        case "mm"
            labelText = "mm";
        otherwise
            error("positionUnit must be 'm' or 'mm'.");
    end
end

function setTimeAxis(ax, t)
    if isempty(t)
        return;
    end

    tStart = t(1);
    tEnd = t(end);

    if abs(tEnd - tStart) < eps
        tEnd = tStart + 1;
    end

    xlim(ax, [tStart, tEnd]);
    ax.XLimMode = 'manual';
end

function plotPenUpRegions(ax, x, y, penUpMask)
    if ~any(penUpMask)
        return;
    end

    y = y(isfinite(y));

    if isempty(y)
        return;
    end

    yMin = min(y);
    yMax = max(y);

    if abs(yMax - yMin) < 1e-9
        yMin = yMin - 1;
        yMax = yMax + 1;
    else
        pad = 0.08 * (yMax - yMin);
        yMin = yMin - pad;
        yMax = yMax + pad;
    end

    moveStart = find(diff([false; penUpMask(:)]) == 1);
    moveEnd   = find(diff([penUpMask(:); false]) == -1);

    for k = 1:numel(moveStart)
        x1 = x(moveStart(k));
        x2 = x(moveEnd(k));

        patch(ax, [x1 x2 x2 x1], [yMin yMin yMax yMax], [0.92 0.92 0.92], ...
            'EdgeColor', 'none', ...
            'FaceAlpha', 0.35, ...
            'HandleVisibility', 'off');
    end

    uistack(findobj(ax, 'Type', 'line'), 'top');
end
