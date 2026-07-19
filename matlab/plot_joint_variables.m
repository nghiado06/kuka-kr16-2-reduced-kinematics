%% plot_joint_variables.m
% Plot the variation of the first three joint variables during letter writing.
% Run validate_letter_trajectory.m first to create letter_trajectory_result.mat.
%
% This script is designed to work with the same result file used by
% simulate_letter_writing.m.

clear; clc; close all;

%% Repository paths
scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);

%% User parameters
resultFile = fullfile(repoRoot, "data", "letter_trajectory_result.mat");
branchName = "up";        % "up", "down", or "both"
angleUnit = "deg";        % "deg" or "rad"
usePointIndex = true;      % true = x-axis is point index, false = use sample time
sampleTime = 0.02;         % used only when usePointIndex = false
saveFigures = false;       % true = export PNG figures

%% Load validated trajectory
if ~isfile(resultFile)
    error("Result file not found. Run validate_letter_trajectory.m first.");
end

load(resultFile, "traj", "result", "metadata");

if ~isfield(result, "qUp") || ~isfield(result, "qDown")
    error("The result file must contain result.qUp and result.qDown.");
end

if ~result.isValid
    warning("Loaded trajectory is not fully valid. Plots will still be generated for available points.");
end

%% Select x-axis
nPts = size(traj.xyz, 1);

if usePointIndex
    x = 1:nPts;
    xLabelText = "Trajectory point index";
else
    x = (0:nPts-1) * sampleTime;
    xLabelText = "Time (s)";
end

%% Select branch data
switch lower(branchName)
    case "up"
        plotSingleBranch(x, result.qUp, traj, "Elbow-up", angleUnit, xLabelText, saveFigures);

    case "down"
        plotSingleBranch(x, result.qDown, traj, "Elbow-down", angleUnit, xLabelText, saveFigures);

    case "both"
        plotSingleBranch(x, result.qUp, traj, "Elbow-up", angleUnit, xLabelText, saveFigures);
        plotSingleBranch(x, result.qDown, traj, "Elbow-down", angleUnit, xLabelText, saveFigures);
        plotCompareBranches(x, result.qUp, result.qDown, angleUnit, xLabelText, saveFigures);

    otherwise
        error("branchName must be 'up', 'down', or 'both'.");
end

fprintf("\nJoint variable plotting finished.\n");
fprintf("Result file : %s\n", resultFile);
fprintf("Branch      : %s\n", branchName);

if exist("metadata", "var") && isfield(metadata, "basePointA")
    fprintf("Base point A = [%.4f %.4f %.4f] m\n", metadata.basePointA);
end

%% ========================================================================
%  Local functions
%% ========================================================================

function plotSingleBranch(x, q, traj, branchTitle, angleUnit, xLabelText, saveFigures)
    qPlot = convertAngle(q, angleUnit);
    yLabelText = makeAngleLabel(angleUnit);

    drawMask = logical(traj.isDraw(:));
    penUpMask = ~drawMask;

    fig = figure('Color', 'w', 'Name', branchTitle + " joint variables");
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    jointNames = ["\theta_1", "\theta_2", "\theta_3"];

    for j = 1:3
        ax = nexttile;
        hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

        plot(ax, x, qPlot(:, j), 'LineWidth', 1.6, ...
            'DisplayName', sprintf('%s', jointNames(j)));

        plotPenUpRegions(ax, x, qPlot(:, j), penUpMask);

        ylabel(ax, sprintf('%s (%s)', jointNames(j), yLabelText), 'FontWeight', 'bold');

        if j == 1
            title(ax, branchTitle + " joint-variable variation", 'FontWeight', 'bold');
        end

        if j == 3
            xlabel(ax, xLabelText, 'FontWeight', 'bold');
        else
            set(ax, 'XTickLabel', []);
        end

        legend(ax, 'Location', 'best');
    end

    if saveFigures
        fileName = lower(strrep(branchTitle, '-', '_')) + "_joint_variables.png";
        exportgraphics(fig, fileName, 'Resolution', 300);
        fprintf("Saved figure: %s\n", fileName);
    end

    % Combined plot for all three joints
    fig2 = figure('Color', 'w', 'Name', branchTitle + " combined joint variables");
    hold on; grid on; box on;

    plot(x, qPlot(:, 1), 'LineWidth', 1.6, 'DisplayName', '\theta_1');
    plot(x, qPlot(:, 2), 'LineWidth', 1.6, 'DisplayName', '\theta_2');
    plot(x, qPlot(:, 3), 'LineWidth', 1.6, 'DisplayName', '\theta_3');

    xlabel(xLabelText, 'FontWeight', 'bold');
    ylabel("Joint angle (" + yLabelText + ")", 'FontWeight', 'bold');
    title(branchTitle + " joint-variable variation", 'FontWeight', 'bold');
    legend('Location', 'best');

    if saveFigures
        fileName = lower(strrep(branchTitle, '-', '_')) + "_joint_variables_combined.png";
        exportgraphics(fig2, fileName, 'Resolution', 300);
        fprintf("Saved figure: %s\n", fileName);
    end
end

function plotCompareBranches(x, qUp, qDown, angleUnit, xLabelText, saveFigures)
    qUpPlot = convertAngle(qUp, angleUnit);
    qDownPlot = convertAngle(qDown, angleUnit);
    yLabelText = makeAngleLabel(angleUnit);

    fig = figure('Color', 'w', 'Name', 'Joint variables comparison');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    jointNames = ["\theta_1", "\theta_2", "\theta_3"];

    for j = 1:3
        ax = nexttile;
        hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

        plot(ax, x, qUpPlot(:, j), 'LineWidth', 1.6, 'DisplayName', 'Elbow-up');
        plot(ax, x, qDownPlot(:, j), '--', 'LineWidth', 1.6, 'DisplayName', 'Elbow-down');

        ylabel(ax, sprintf('%s (%s)', jointNames(j), yLabelText), 'FontWeight', 'bold');

        if j == 1
            title(ax, 'Comparison of elbow-up and elbow-down joint variables', 'FontWeight', 'bold');
        end

        if j == 3
            xlabel(ax, xLabelText, 'FontWeight', 'bold');
        else
            set(ax, 'XTickLabel', []);
        end

        legend(ax, 'Location', 'best');
    end

    if saveFigures
        fileName = "joint_variables_up_down_comparison.png";
        exportgraphics(fig, fileName, 'Resolution', 300);
        fprintf("Saved figure: %s\n", fileName);
    end
end

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

function plotPenUpRegions(ax, x, y, penUpMask)
    if ~any(penUpMask)
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
