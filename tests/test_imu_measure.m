function tests = test_imu_measure
%TEST_IMU_MEASURE Tests for the stateless IMU measurement model.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testLevelHoverSpecificForceAndGyro(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);
state.omega_m_radps = repmat(params.hover_omega_radps, 4, 1);

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);
diag = struct('forces_b_N', fm.forces_b_N);

imu = uav.sensors.imu_measure(state, diag, params);

verifyEqual(testCase, imu.gyro_b_radps, zeros(3, 1), 'AbsTol', 1.0e-12);
verifyEqual(testCase, imu.accel_b_mps2(1:2), zeros(2, 1), 'AbsTol', 1.0e-12);
verifyEqual(testCase, imu.accel_b_mps2(3), -params.gravity_mps2, 'AbsTol', 1.0e-10);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.imu.gyro_bias_b_radps = zeros(3, 1);
params.sensors.imu.gyro_noise_std_b_radps = zeros(3, 1);
end
