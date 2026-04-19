function tests = test_demo_pitch_step_minus10deg
%TEST_DEMO_PITCH_STEP_MINUS10DEG Smoke tests for the -10 deg pitch demo.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function testPitchDemoProducesArtifactsAndTracksCommand(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'run_demo_pitch_step_minus10deg.m'));
demo = evalin('base', 'demo_pitch_step_minus10deg');

verifyGreaterThan(testCase, numel(demo.series.time_s), 0);
verifyTrue(testCase, isfield(demo.log, 'reference'));
verifyTrue(testCase, isfield(demo.log, 'controller_diag'));

verifyGreaterThan(testCase, demo.metrics.final_altitude_m, 15.0);

verifyLessThan(testCase, abs(demo.metrics.final_pitch_deg - ...
    demo.profile.pitch_cmd_deg), 4.0);
verifyLessThan(testCase, abs(demo.metrics.final_estimated_pitch_deg - ...
    demo.profile.pitch_cmd_deg), 4.0);
verifyLessThan(testCase, ...
    abs(demo.metrics.final_pitch_estimation_error_deg), 3.0);
verifyLessThan(testCase, demo.metrics.max_pitch_estimation_error_deg, 6.0);
verifyLessThan(testCase, demo.metrics.min_accel_correction_weight, 0.2);

verifyLessThan(testCase, abs(demo.metrics.final_true_quat_norm - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(demo.metrics.final_estimated_quat_norm - 1.0), 1.0e-9);

verifyTrue(testCase, isfile(demo.mat_file));
verifyTrue(testCase, isfile(demo.csv_file));
end
