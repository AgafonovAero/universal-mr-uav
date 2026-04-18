function tests = test_run_sil_stub_hover
%TEST_RUN_SIL_STUB_HOVER Smoke tests for the SIL-prep hover shell.

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

function testSilHoverRunCompletesAndKeepsQuatNormsNearOne(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'run_sil_stub_hover.m'));
demo = evalin('base', 'sil_stub_hover_demo');

verifyTrue(testCase, isfield(demo.final_truth, 'state'));
verifyTrue(testCase, isfield(demo.final_truth, 'estimator'));
verifyTrue(testCase, isfield(demo.final_diag, 'omega_m_radps'));
verifyEqual(testCase, numel(demo.final_hover_actuator_cmd), 4);
verifyGreaterThanOrEqual(testCase, demo.final_hover_actuator_cmd, 0.0);
verifyLessThanOrEqual(testCase, demo.final_hover_actuator_cmd, 1.0);
verifyLessThan(testCase, abs(demo.final_true_quat_norm - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(demo.final_estimated_quat_norm - 1.0), 1.0e-9);
end
