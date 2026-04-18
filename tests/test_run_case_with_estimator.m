function tests = test_run_case_with_estimator
%TEST_RUN_CASE_WITH_ESTIMATOR Tests for the case runner with estimator history.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testHoverCaseReturnsEstimatorHistoryAndBoundedAltitude(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
case_cfg = local_hover_case_cfg(params);

log = uav.sim.run_case_with_estimator(case_cfg);
final_est = log.estimator(end);
all_alt_m = [log.estimator.alt_m];

verifyGreaterThan(testCase, numel(log.time_s), 0);
verifyTrue(testCase, isfield(log, 'estimator'));
verifyEqual(testCase, numel(log.estimator), numel(log.time_s));
verifyTrue(testCase, isfield(final_est, 'q_nb'));
verifyLessThan(testCase, max(abs(log.quat_norm_est - 1.0)), 1.0e-12);
verifyLessThan(testCase, abs(final_est.alt_m - log.sensors(end).baro.alt_m), 0.1);
verifyLessThan(testCase, max(abs(all_alt_m)), 10.0);
end

function case_cfg = local_hover_case_cfg(params)
%LOCAL_HOVER_CASE_CFG Build the hover runner configuration for testing.

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = params.demo.case_hover_t_final_s;
case_cfg.command_fun = @local_hover_command;
end

function motor_cmd_radps = local_hover_command(~, ~, params)
%LOCAL_HOVER_COMMAND Return the constant hover motor command.

motor_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.imu.gyro_bias_b_radps = zeros(3, 1);
params.sensors.imu.gyro_noise_std_b_radps = zeros(3, 1);
params.sensors.baro.alt_bias_m = 0.0;
params.sensors.baro.alt_noise_std_m = 0.0;
params.sensors.gnss.pos_bias_ned_m = zeros(3, 1);
params.sensors.gnss.pos_noise_std_ned_m = zeros(3, 1);
params.sensors.gnss.vel_bias_ned_mps = zeros(3, 1);
params.sensors.gnss.vel_noise_std_ned_mps = zeros(3, 1);
params.sensors.mag.field_bias_b_uT = zeros(3, 1);
params.sensors.mag.field_noise_std_b_uT = zeros(3, 1);
end
