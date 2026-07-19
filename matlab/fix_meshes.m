clear; clc; close all;

%% User settings
scriptDir  = fileparts(mfilename('fullpath'));
repoRoot   = fileparts(scriptDir);
projectRoot = fullfile(repoRoot, "models", "kuka_robot_model");

urdfInputFile  = "urdf\kuka_robot_model.urdf";
urdfOutputFile = "urdf\robot_fixed.urdf";

meshInputDir   = "meshes";
meshOutputDir  = "meshes_linkframe";

showRawRobot   = true;
showFixedRobot = true;

rebaseBodies = [
    "link_0"
    "link_1"
    "link_2"
    "end_tip"
];

% ============================================================
% Common correction for all moving-link meshes
% Try one of the candidate rotations below
% ============================================================

% Option A: rotate +90 deg about X
% Rcorr = rotx(pi/2);

% Option B: rotate -90 deg about X
% Rcorr = rotx(-pi/2);

% Option C: rotate +90 deg about Y
% Rcorr = roty(pi/2);

% Option D: rotate -90 deg about Y
% Rcorr = roty(-pi/2);

% Option E: rotate +90 deg about Z
% Rcorr = rotz(pi/2);

% Option F: rotate -90 deg about Z
% Rcorr = rotz(-pi/2);

% Option G: rotate 180 deg about X
% Rcorr = rotx(pi);

% Option H: rotate 180 deg about Y
% Rcorr = roty(pi);

% Option I: rotate 180 deg about Z
% Rcorr = rotz(pi);

Rcorr = eye(3);

% Rcorr = rotz(-pi/2)*Rcorr;

Tcorr = eye(4);
Tcorr(1:3,1:3) = Rcorr;

%% Build paths
urdfInputPath  = fullfile(projectRoot, urdfInputFile);
urdfOutputPath = fullfile(projectRoot, urdfOutputFile);

meshInputPath  = fullfile(projectRoot, meshInputDir);
meshOutputPath = fullfile(projectRoot, meshOutputDir);

validateEnvironment(projectRoot, urdfInputPath, meshInputPath);
prepareOutputFolder(meshOutputPath);

%% Import raw robot
robotRaw = importrobot(urdfInputPath, ...
    "DataFormat", "row", ...
    "MeshPath", {meshInputPath});

qZero     = zeros(1, numel(homeConfiguration(robotRaw)));
baseName  = string(robotRaw.BaseName);
bodyNames = string(robotRaw.BodyNames);

if showRawRobot
    figure("Name", "Raw Robot", "Color", "w");
    show(robotRaw, qZero, "Frames", "on", "Visuals", "on");
    axis equal;
    grid on;
    view(45, 20);
    title("Raw Visual Meshes");
end

%% Copy base mesh explicitly
fprintf("=== Mesh rebasing with common correction ===\n");
fprintf("URDF input      : %s\n", urdfInputPath);
fprintf("Mesh input dir  : %s\n", meshInputPath);
fprintf("Mesh output dir : %s\n\n", meshOutputPath);

copyBaseMeshIfPresent(baseName, meshInputPath, meshOutputPath);

%% Process moving-link meshes
for i = 1:numel(bodyNames)
    bodyName = bodyNames(i);

    inputMeshFile  = fullfile(meshInputPath,  bodyName + ".STL");
    outputMeshFile = fullfile(meshOutputPath, bodyName + ".STL");

    if ~isfile(inputMeshFile)
        warning("Missing mesh for body '%s': %s", bodyName, inputMeshFile);
        continue;
    end

    if ismember(bodyName, rebaseBodies)
        [faces, vertices] = readStlMesh(inputMeshFile);

        % Step 1: apply common correction in source mesh frame
        vertices = transformVertices(vertices, Tcorr);

        % Step 2: rebase into local link frame
        TBaseToBody = getTransform(robotRaw, qZero, bodyName, baseName);
        TBodyToBase = inv(TBaseToBody);

        rebasedVertices = transformVertices(vertices, TBodyToBase);
        writeStlMesh(outputMeshFile, faces, rebasedVertices);

        fprintf("[REBASING] %s\n", bodyName);
    else
        copyfile(inputMeshFile, outputMeshFile);
        fprintf("[KEEP]     %s\n", bodyName);
    end
end

%% Rewrite URDF mesh paths
rewriteUrdfMeshPaths(urdfInputPath, urdfOutputPath, meshInputDir, meshOutputDir);

fprintf("\nFixed URDF saved to: %s\n", urdfOutputPath);

%% Validate fixed robot
if showFixedRobot
    robotFixed = importrobot(urdfOutputPath, ...
        "DataFormat", "row", ...
        "MeshPath", {meshOutputPath});

    figure("Name", "Fixed Robot", "Color", "w");
    show(robotFixed, qZero, "Frames", "on", "Visuals", "on");
    axis equal;
    grid on;
    view(45, 20);
    title("Fixed Robot - Common Correction");
end

%% Local functions
function validateEnvironment(projectRoot, urdfInputPath, meshInputPath)
    if ~isfolder(projectRoot)
        error("Project root does not exist: %s", projectRoot);
    end

    if ~isfile(urdfInputPath)
        error("URDF file not found: %s", urdfInputPath);
    end

    if ~isfolder(meshInputPath)
        error("Mesh input folder not found: %s", meshInputPath);
    end
end

function prepareOutputFolder(meshOutputPath)
    if ~isfolder(meshOutputPath)
        mkdir(meshOutputPath);
    end
end

function copyBaseMeshIfPresent(baseName, meshInputPath, meshOutputPath)
    inputBaseMesh  = fullfile(meshInputPath,  baseName + ".STL");
    outputBaseMesh = fullfile(meshOutputPath, baseName + ".STL");

    if isfile(inputBaseMesh)
        copyfile(inputBaseMesh, outputBaseMesh);
        fprintf("[KEEP]     %s\n", baseName);
    else
        warning("Missing mesh for base '%s': %s", baseName, inputBaseMesh);
    end
end

function [faces, vertices] = readStlMesh(stlFile)
    meshTri  = stlread(stlFile);
    faces    = meshTri.ConnectivityList;
    vertices = meshTri.Points;
end

function transformedVertices = transformVertices(vertices, T)
    verticesH = [vertices, ones(size(vertices, 1), 1)];
    transformedVerticesH = (T * verticesH')';
    transformedVertices  = transformedVerticesH(:, 1:3);
end

function writeStlMesh(stlFile, faces, vertices)
    meshTri = triangulation(faces, vertices);
    stlwrite(meshTri, stlFile);
end

function rewriteUrdfMeshPaths(urdfInputPath, urdfOutputPath, meshInputDir, meshOutputDir)
    urdfText = fileread(urdfInputPath);

    urdfText = strrep(urdfText, meshInputDir + "/",  meshOutputDir + "/");
    urdfText = strrep(urdfText, meshInputDir + "\",  meshOutputDir + "\");
    urdfText = strrep(urdfText, "./" + meshInputDir + "/", "./" + meshOutputDir + "/");
    urdfText = strrep(urdfText, ".\" + meshInputDir + "\", ".\" + meshOutputDir + "\");

    fileId = fopen(urdfOutputPath, "w");
    if fileId == -1
        error("Cannot create output URDF: %s", urdfOutputPath);
    end

    fwrite(fileId, urdfText);
    fclose(fileId);
end

function R = rotx(a)
    R = [1 0 0;
         0 cos(a) -sin(a);
         0 sin(a)  cos(a)];
end

function R = roty(a)
    R = [ cos(a) 0 sin(a);
          0      1 0;
         -sin(a) 0 cos(a)];
end

function R = rotz(a)
    R = [cos(a) -sin(a) 0;
         sin(a)  cos(a) 0;
         0       0      1];
end