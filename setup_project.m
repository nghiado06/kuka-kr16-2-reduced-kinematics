function setup_project()
%SETUP_PROJECT Add repository MATLAB sources to the active path.

    repoRoot = fileparts(mfilename('fullpath'));
    matlabDir = fullfile(repoRoot, 'matlab');
    dataDir = fullfile(repoRoot, 'data');
    resultsDir = fullfile(repoRoot, 'results');

    addpath(genpath(matlabDir));

    if ~isfolder(dataDir)
        mkdir(dataDir);
    end
    if ~isfolder(resultsDir)
        mkdir(resultsDir);
    end

    fprintf('Repository configured: %s\n', repoRoot);
    fprintf('MATLAB source path added: %s\n', matlabDir);
end
