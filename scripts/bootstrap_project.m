%% BOOTSTRAP_PROJECT Add repository paths and print next MATLAB commands.
% Description:
%   Adds the Stage-1 source, script, and test folders to the MATLAB path
%   without creating a binary project file.
%
% Inputs:
%   none
%
% Outputs:
%   none
%
% Units:
%   not applicable
%
% Assumptions:
%   The script is executed from a checkout of this repository.

repoRoot = fileparts(fileparts(mfilename('fullpath')));

addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
addpath(fullfile(repoRoot, 'tests'));

fprintf('Project paths added from:\n  %s\n\n', repoRoot);
fprintf('Next MATLAB commands:\n');
fprintf('  params = uav.sim.default_params_quad_x250();\n');
fprintf('  demo = uav.sim.run_hover_demo();\n');
fprintf('  results = runtests(''tests'');\n');
fprintf('  table(results)\n');
