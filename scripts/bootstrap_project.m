%% BOOTSTRAP_PROJECT Add repository paths and print next MATLAB commands.
% Description:
%   Adds the Stage-1 / Stage-1.5 source, script, and test folders to the
%   MATLAB path without creating a binary project file.
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
fprintf('  run(''scripts/run_motor_spool_demo.m'');\n');
fprintf('  run(''scripts/run_openloop_hover_demo.m'');\n');
fprintf('  results = runtests(''tests'');\n');
fprintf('  table(results)\n');
