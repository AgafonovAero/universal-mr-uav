function tests = test_ardupilot_loopback_runner
%TEST_ARDUPILOT_LOOPBACK_RUNNER Tests for the ArduPilot loopback runner.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testLoopbackRunnerReturnsPacketsAndBoundedQuatNorms(testCase)
params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = 3.0;
case_cfg.loopback_mode = "hover";
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_loopback(case_cfg);

verifyGreaterThan(testCase, numel(log.time_s), 0);
verifyTrue(testCase, isfield(log, 'ardupilot_packet'));
verifyTrue(testCase, isfield(log, 'servo_packet'));
verifyTrue(testCase, isfield(log, 'motor_cmd_radps'));
verifyEqual(testCase, numel(log.ardupilot_packet), numel(log.time_s));
verifyEqual(testCase, numel(log.servo_packet), numel(log.time_s));
verifyLessThan(testCase, max(abs(log.quat_norm_true - 1.0)), 1.0e-10);
verifyLessThan(testCase, max(abs(log.quat_norm_est - 1.0)), 1.0e-10);
verifyTrue(testCase, all([log.servo.valid]));
verifyGreaterThan(testCase, norm(log.motor_cmd_radps(end, :)), 0.0);
end
