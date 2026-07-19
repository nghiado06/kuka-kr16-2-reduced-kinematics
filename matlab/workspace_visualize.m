% ============================================================
% BTL_workspace_robot_OXY_OXZ_clean.m
%
% Visualize OXZ and OXY workspace projections with URDF robot.
%
% Robot display configuration:
% q = [0, -90, 180] deg
%
% Unit convention:
% - DH parameters are defined in meters.
% - URDF model is assumed to be in meters.
% ============================================================

clear; clc; close all;

%% ============================================================
% 1. Load URDF robot model
% ============================================================

scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);
urdfFile  = fullfile(repoRoot, "models", "kuka_robot_model", "urdf", "robot_fixed.urdf");
meshPath  = fullfile(repoRoot, "models", "kuka_robot_model", "meshes_linkframe");

robot = importrobot(urdfFile, ...
    "DataFormat", "row", ...
    "MeshPath", {meshPath});

robot.Gravity = [0 0 -9.81];

n = numel(homeConfiguration(robot));

q = zeros(1, n);
q(1) = deg2rad(0);
q(2) = deg2rad(-90);
q(3) = deg2rad(180);

%% ============================================================
% 2. DH parameters and joint limits
% ============================================================

mm_to_m = 1e-3;

a1 = 260 * mm_to_m;
a2 = 680 * mm_to_m;
a3 = 500 * mm_to_m;

d1 = 675 * mm_to_m;
d2 = 0;
d3 = 0;

alpha1 = deg2rad(-90);
alpha2 = deg2rad(0);
alpha3 = deg2rad(-90);

t1_min = deg2rad(-185);
t1_max = deg2rad(185);

t2_min = deg2rad(-155);
t2_max = deg2rad(35);

t3_min = deg2rad(-130);
t3_max = deg2rad(154);

%% ============================================================
% 3. Sampling settings
% ============================================================

N_xz = 800;
N_xy = 300000;

maxPlotPoints = 70000;

boundaryShrinkFactor = 0.85;

%% ============================================================
% 4. DH transformation function
% ============================================================

DH_matrix = @(theta, a, d, alpha) ...
    [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
     sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0           sin(alpha)             cos(alpha)            d;
     0           0                      0                     1];

%% ============================================================
% 5. Compute OXZ workspace using DH model
% ============================================================

th2_vals = linspace(t2_min, t2_max, N_xz);
th3_vals = linspace(t3_min, t3_max, N_xz);

[TH2, TH3] = meshgrid(th2_vals, th3_vals);

TH2_vec = TH2(:);
TH3_vec = TH3(:);

num_xz = numel(TH2_vec);

X_xz = zeros(num_xz, 1);
Y_xz = zeros(num_xz, 1);
Z_xz = zeros(num_xz, 1);

A1_0 = DH_matrix(0, a1, d1, alpha1);

for i = 1:num_xz
    A2 = DH_matrix(TH2_vec(i), a2, d2, alpha2);
    A3 = DH_matrix(TH3_vec(i), a3, d3, alpha3);

    T = A1_0 * A2 * A3;

    X_xz(i) = T(1,4);
    Y_xz(i) = 0;
    Z_xz(i) = T(3,4);
end

idx_xz = downsampleIndex(numel(X_xz), maxPlotPoints);

X_xz_plot = X_xz(idx_xz);
Y_xz_plot = Y_xz(idx_xz);
Z_xz_plot = Z_xz(idx_xz);

k_xz = boundary(X_xz, Z_xz, boundaryShrinkFactor);

%% ============================================================
% 6. Compute OXY workspace using DH model
% ============================================================

rng(1);

th1 = t1_min + (t1_max - t1_min) * rand(N_xy, 1);
th2 = t2_min + (t2_max - t2_min) * rand(N_xy, 1);
th3 = t3_min + (t3_max - t3_min) * rand(N_xy, 1);

X_xy = zeros(N_xy, 1);
Y_xy = zeros(N_xy, 1);
Z_xy = zeros(N_xy, 1);

for i = 1:N_xy
    A1 = DH_matrix(th1(i), a1, d1, alpha1);
    A2 = DH_matrix(th2(i), a2, d2, alpha2);
    A3 = DH_matrix(th3(i), a3, d3, alpha3);

    T = A1 * A2 * A3;

    X_xy(i) = T(1,4);
    Y_xy(i) = T(2,4);
    Z_xy(i) = 0;
end

idx_xy = downsampleIndex(numel(X_xy), maxPlotPoints);

X_xy_plot = X_xy(idx_xy);
Y_xy_plot = Y_xy(idx_xy);
Z_xy_plot = Z_xy(idx_xy);

k_xy = boundary(X_xy, Y_xy, boundaryShrinkFactor);

%% ============================================================
% 7. Figure 1: OXZ workspace + robot
% ============================================================

fig1 = figure('Name', 'OXZ Workspace + Robot', ...
    'Color', 'w', ...
    'Renderer', 'opengl', ...
    'Position', [100 120 850 650]);

ax1 = axes(fig1);
hold(ax1, 'on');

plot3(ax1, X_xz_plot, Y_xz_plot, Z_xz_plot, '.', ...
    'Color', [0.80 0.80 0.80], ...
    'MarkerSize', 4);

plot3(ax1, X_xz(k_xz), zeros(size(k_xz)), Z_xz(k_xz), ...
    'b-', ...
    'LineWidth', 2.0);

show(robot, q, ...
    "Parent", ax1, ...
    "Frames", "off", ...
    "Visuals", "on", ...
    "PreservePlot", true, ...
    "FastUpdate", false);

axis(ax1, 'equal');
grid(ax1, 'on');
box(ax1, 'on');

xlabel(ax1, 'X (m)');
ylabel(ax1, 'Y (m)');
zlabel(ax1, 'Z (m)');

title(ax1, 'OXZ Workspace + Robot at q = [0, -90, 180] deg');

view(ax1, 0, 0);
camproj(ax1, 'orthographic');

xlim(ax1, [-1.5 1.5]);
ylim(ax1, [-0.2 0.2]);
zlim(ax1, [0 2.2]);

delete(findall(fig1, 'Type', 'Light'));

camlight(ax1, 'headlight');
camlight(ax1, 45, 25);

lighting(ax1, 'gouraud');
material(ax1, [0.45 0.55 0.20 6 0.35]);

rotate3d(fig1, 'on');
drawnow;

%% ============================================================
% 8. Figure 2: OXY workspace + robot
% ============================================================

fig2 = figure('Name', 'OXY Workspace + Robot', ...
    'Color', 'w', ...
    'Renderer', 'opengl', ...
    'Position', [980 120 850 650]);

ax2 = axes(fig2);
hold(ax2, 'on');

plot3(ax2, X_xy_plot, Y_xy_plot, Z_xy_plot, '.', ...
    'Color', [0.80 0.80 0.80], ...
    'MarkerSize', 4);

plot3(ax2, X_xy(k_xy), Y_xy(k_xy), zeros(size(k_xy)), ...
    'b-', ...
    'LineWidth', 2.0);

show(robot, q, ...
    "Parent", ax2, ...
    "Frames", "off", ...
    "Visuals", "on", ...
    "PreservePlot", true, ...
    "FastUpdate", false);

axis(ax2, 'equal');
grid(ax2, 'on');
box(ax2, 'on');

xlabel(ax2, 'X (m)');
ylabel(ax2, 'Y (m)');
zlabel(ax2, 'Z (m)');

title(ax2, 'OXY Workspace + Robot at q = [0, -90, 180] deg');

view(ax2, 0, 90);
camproj(ax2, 'orthographic');

xlim(ax2, [-1.5 1.5]);
ylim(ax2, [-1.5 1.5]);
zlim(ax2, [0 2.2]);

delete(findall(fig2, 'Type', 'Light'));

camlight(ax2, 'headlight');
camlight(ax2, 45, 25);

lighting(ax2, 'gouraud');
material(ax2, [0.45 0.55 0.20 6 0.35]);

lighting(ax2, 'flat');
material(ax2, [0.4 0.8 0.2 8 1]);

rotate3d(fig2, 'on');
drawnow;

%% ============================================================
% 9. End-effector position check
% ============================================================

T_tip = getTransform(robot, q, "end_tip");
p_tip = T_tip(1:3, 4);

fprintf("\nEnd-effector position from URDF:\n");
fprintf("x = %.6f m = %.3f mm\n", p_tip(1), p_tip(1)*1000);
fprintf("y = %.6f m = %.3f mm\n", p_tip(2), p_tip(2)*1000);
fprintf("z = %.6f m = %.3f mm\n", p_tip(3), p_tip(3)*1000);

%% ============================================================
% Local function
% ============================================================

function idx = downsampleIndex(N, maxN)
    if N <= maxN
        idx = 1:N;
    else
        idx = round(linspace(1, N, maxN));
    end
end