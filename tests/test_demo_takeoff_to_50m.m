function tests = test_demo_takeoff_to_50m
%TEST_DEMO_TAKEOFF_TO_50M Smoke tests for the 50 m takeoff demo.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function testTakeoffDemoProducesArtifactsAndBoundedState(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'run_demo_takeoff_to_50m.m'));
demo = evalin('base', 'demo_takeoff_to_50m');

verifyGreaterThan(testCase, numel(demo.series.time_s), 0);
verifyEqual(testCase, demo.series.altitude_ref_m(end), 50.0, 'AbsTol', 1.0e-12);
verifyGreaterThan(testCase, demo.metrics.final_altitude_m, 45.0);
verifyLessThan(testCase, demo.metrics.final_altitude_m, 55.0);
verifyLessThan(testCase, abs(demo.metrics.final_estimated_altitude_m - ...
    demo.metrics.final_altitude_m), 2.0);
verifyLessThan(testCase, abs(demo.metrics.final_true_quat_norm - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(demo.metrics.final_estimated_quat_norm - 1.0), 1.0e-9);
verifyTrue(testCase, isfile(demo.mat_file));
verifyTrue(testCase, isfile(demo.csv_file));
end
