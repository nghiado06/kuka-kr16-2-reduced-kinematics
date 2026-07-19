clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);
urdfFile  = fullfile(repoRoot, "models", "kuka_robot_model", "urdf", "robot_fixed.urdf");
meshPath  = fullfile(repoRoot, "models", "kuka_robot_model", "meshes_linkframe");

robot = importrobot(urdfFile, ...
    "DataFormat", "row", ...
    "MeshPath", {meshPath});
robot.Gravity = [0 0 -9.81];

showdetails(robot);
n = numel(homeConfiguration(robot));
disp(n);

%% Configuration Space

% Offset
o1 = deg2rad(0);
o2 = deg2rad(0);
o3 = deg2rad(0);

q = zeros(1, n);

q(1) = deg2rad(0) + o1;
q(2) = deg2rad(24.217) + o2;
q(3) = deg2rad(27.296) + o3;

%% Plot 3D Model

figure('Color','w');
show(robot, q, "Frames", "on", "Visuals", "on");

% Light
% camproj orthographic;
% 
% delete(findall(gcf,'Type','Light'))
% camlight('left')
% camlight('right')
% camlight(0, 90)
% 
% lighting gouraud
% material dull

axis equal;
grid off;
view(45,20);
hold on;

p = findobj(gca, 'Type', 'Patch');
for k = 1:numel(p)
    p(k).FaceAlpha = 1.0;
end

xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
zlim([0.0 2.5]);

% % XY plane
% [xXY, yXY] = meshgrid(-1.5:0.1:1.5, -1.5:0.1:1.5);
% zXY = zeros(size(xXY));
% surf(xXY, yXY, zXY, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
% 
% % YZ plane
% [yYZ, zYZ] = meshgrid(-1.5:0.1:1.5, 0:0.1:1.5);
% xYZ = zeros(size(yYZ));
% surf(xYZ, yYZ, zYZ, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
% 
% % XZ plane
% [xXZ, zXZ] = meshgrid(-1.5:0.1:1.5, 0:0.1:1.5);
% yXZ = zeros(size(xXZ));
% surf(xXZ, yXZ, zXZ, 'FaceAlpha', 0.12, 'EdgeColor', 'none');

%% XY plane view
figure('Color','w');
show(robot, q, "Frames", "on", "Visuals", "on");
axis equal; grid on;
view(0, 90);
camproj orthographic
% camlight headlight;
% lighting gouraud;
% material dull;
camlight;
lighting flat;
material dull;

title("XY View");
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);

%% XZ plane view
figure('Color','w');
show(robot, q, "Frames", "on", "Visuals", "on");
axis equal; grid on;
view(0, 0);
camproj orthographic
% Option 1:
% camlight headlight;
% lighting gouraud;
% material dull;

% Option 2:
camlight;
lighting flat;
material dull;

% Option 3:
% delete(findall(gcf,'Type','Light'))
% camlight left
% lighting flat
% material dull

title("XZ View");
xlim([-1.5 1.5]);
zlim([0.0 2.5]);

%% YZ plane view
figure('Color','w');
show(robot, q, "Frames", "on", "Visuals", "on");
axis equal; grid on;
view(90, 0);
camproj orthographic
% camlight headlight;
% lighting gouraud;
% material dull;
camlight;
lighting flat;
material dull;

title("YZ View");
ylim([-1.5 1.5]);
zlim([0.0 2.5]);

%% Get the end tip position
T = getTransform(robot, q, "end_tip");
p = T(1:3, 4);

disp("Transform of end-effector:");
disp(T)

fprintf("End-effector position:\n");
fprintf("x = %.6f mm\n", p(1)*1000);
fprintf("y = %.6f mm\n", p(2)*1000);
fprintf("z = %.6f mm\n", p(3)*1000);



