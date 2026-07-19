clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);
urdf = fullfile(repoRoot, "models", "kuka_robot_model", "urdf", "kuka_robot_model.urdf");
meshPath = fullfile(repoRoot, "models", "kuka_robot_model", "meshes");
robot = importrobot(urdf, "DataFormat","row", "MeshPath", {meshPath});

disp("BaseName: " + robot.BaseName);
disp("Bodies:"); disp(robot.BodyNames');
disp("NumJoints: " + numel(homeConfiguration(robot)));

figure;
show(robot, "Frames","on", "Visuals","on");
axis equal; grid on; view(45,20);
title("Raw Visual Meshes");