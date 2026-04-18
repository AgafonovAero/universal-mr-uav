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
verifyGreaterThan(testCase, demo.metrics.final_altitude_m, 15.0);
verifyLessThan(testCase, abs(demo.metrics.final_pitch_deg - ...
    demo.profile.pitch_cmd_deg), 4.0);
verifyTrue(testCase, isfinite(demo.metrics.final_estimated_pitch_deg));
verifyLessThan(testCase, max(abs(demo.series.pitch_est_rad)), deg2rad(15.0));
verifyLessThan(testCase, abs(demo.metrics.final_true_quat_norm - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(demo.metrics.final_estimated_quat_norm - 1.0), 1.0e-9);
verifyTrue(testCase, isfile(demo.mat_file));
verifyTrue(testCase, isfile(demo.csv_file));
end
