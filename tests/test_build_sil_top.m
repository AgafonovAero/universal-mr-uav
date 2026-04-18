function tests = test_build_sil_top
%TEST_BUILD_SIL_TOP Tests for programmatic thin SIL-prep model generation.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function teardown(~)
if bdIsLoaded('sil_top')
    close_system('sil_top', 0);
end
end

function testBuildSilTopCreatesThinBridgeModel(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'build_sil_top.m'));
sil_top_build = evalin('base', 'sil_top_build');

verifyEqual(testCase, exist(sil_top_build.model_file, 'file'), 2);

load_system(sil_top_build.model_file);
verifyTrue(testCase, bdIsLoaded(sil_top_build.model_name));
verifyEqual(testCase, get_param(sil_top_build.stub_block, 'System'), ...
    'uav.sl.StubExternalFCSSystem');
verifyEqual(testCase, get_param(sil_top_build.bridge_block, 'System'), ...
    'uav.sl.Stage15SILBridgeSystem');
verifyEqual(testCase, get_param(sil_top_build.model_name, 'Solver'), ...
    'FixedStepDiscrete');
verifyEqual(testCase, get_param(sil_top_build.model_name, 'SolverType'), ...
    'Fixed-step');
end
