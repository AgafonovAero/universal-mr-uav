function tests = test_build_mil_top_atc
%TEST_BUILD_MIL_TOP_ATC Tests for thin ATC MIL model generation.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function teardown(~)
if bdIsLoaded('mil_top_atc')
    close_system('mil_top_atc', 0);
end
end

function testBuildMilTopAtcCreatesThinShellModel(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'build_mil_top_atc.m'));
mil_top_atc_build = evalin('base', 'mil_top_atc_build');

verifyEqual(testCase, exist(mil_top_atc_build.model_file, 'file'), 2);

load_system(mil_top_atc_build.model_file);
verifyTrue(testCase, bdIsLoaded(mil_top_atc_build.model_name));
verifyEqual(testCase, get_param(mil_top_atc_build.shell_block, 'System'), ...
    'uav.sl.Stage15ATCMILSystem');
verifyEqual(testCase, get_param(mil_top_atc_build.model_name, 'Solver'), ...
    'FixedStepDiscrete');
verifyEqual(testCase, get_param(mil_top_atc_build.model_name, 'SolverType'), ...
    'Fixed-step');
end
