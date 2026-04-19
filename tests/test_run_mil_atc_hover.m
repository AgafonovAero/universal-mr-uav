function tests = test_run_mil_atc_hover
%TEST_RUN_MIL_ATC_HOVER Smoke tests for the ATC MIL hover demo.

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

function testRunMilAtcHoverProducesFiniteDiagnostics(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'run_mil_atc_hover.m'));
demo = evalin('base', 'mil_atc_hover_demo');

verifyTrue(testCase, isstruct(demo));
verifyTrue(testCase, isfield(demo, 'final_truth'));
verifyTrue(testCase, isfield(demo, 'final_estimator'));
verifyTrue(testCase, isfield(demo, 'final_atc_cmd'));
verifyTrue(testCase, isfield(demo, 'final_diag'));

verifySize(testCase, demo.final_atc_cmd.motor_norm_01, [4 1]);
verifyGreaterThanOrEqual(testCase, demo.final_atc_cmd.motor_norm_01, zeros(4, 1));
verifyLessThanOrEqual(testCase, demo.final_atc_cmd.motor_norm_01, ones(4, 1));
verifyLessThan(testCase, abs(demo.final_diag.quat_norm_true - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(demo.final_diag.quat_norm_est - 1.0), 1.0e-9);
verifyTrue(testCase, isfinite(demo.final_true_altitude_m));
verifyTrue(testCase, isfinite(demo.final_estimated_altitude_m));
end
