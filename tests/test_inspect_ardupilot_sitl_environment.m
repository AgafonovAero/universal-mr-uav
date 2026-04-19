function tests = test_inspect_ardupilot_sitl_environment
%TEST_INSPECT_ARDUPILOT_SITL_ENVIRONMENT Tests for environment inspection.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testInspectionReturnsStructuredStatusWithoutThrowing(testCase)
cfg = uav.ardupilot.default_json_config();
info = uav.ardupilot.inspect_sitl_environment(cfg);

verifyTrue(testCase, isstruct(info));
verifyTrue(testCase, isfield(info, 'is_ready'));
verifyTrue(testCase, isfield(info, 'has_wsl'));
verifyTrue(testCase, isfield(info, 'has_python'));
verifyTrue(testCase, isfield(info, 'has_sim_vehicle'));
verifyTrue(testCase, isfield(info, 'ardupilot_root'));
verifyTrue(testCase, isfield(info, 'messages'));
verifyTrue(testCase, islogical(info.is_ready));
verifyTrue(testCase, islogical(info.has_python));
verifyTrue(testCase, islogical(info.has_sim_vehicle));
verifyTrue(testCase, isstring(info.messages));
verifyGreaterThanOrEqual(testCase, numel(info.messages), 1);
end
