function tests = test_sensors_step
%TEST_SENSORS_STEP Tests for the stateless sensor-layer aggregator.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testAggregatorContainsAllSensorsAndMatchesIndividualCalls(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);
state.p_ned_m = [7.0; -4.0; -15.0];
state.v_b_mps = [2.0; 0.5; -0.2];
state.omega_m_radps = repmat(params.hover_omega_radps, 4, 1);

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);
diag = struct();
diag.forces_b_N = fm.forces_b_N;
diag.moments_b_Nm = fm.moments_b_Nm;

sens = uav.sensors.sensors_step(state, diag, params);
imu = uav.sensors.imu_measure(state, diag, params);
baro = uav.sensors.baro_measure(state, params);
mag = uav.sensors.mag_measure(state, params);
gnss = uav.sensors.gnss_measure(state, params);

verifyTrue(testCase, isfield(sens, 'imu'));
verifyTrue(testCase, isfield(sens, 'baro'));
verifyTrue(testCase, isfield(sens, 'mag'));
verifyTrue(testCase, isfield(sens, 'gnss'));
verifyEqual(testCase, sens.imu.accel_b_mps2, imu.accel_b_mps2, 'AbsTol', 1.0e-12);
verifyEqual(testCase, sens.imu.gyro_b_radps, imu.gyro_b_radps, 'AbsTol', 1.0e-12);
verifyEqual(testCase, sens.baro.alt_m, baro.alt_m, 'AbsTol', 1.0e-12);
verifyEqual(testCase, sens.baro.pressure_pa, baro.pressure_pa, 'AbsTol', 1.0e-12);
verifyEqual(testCase, sens.mag.field_b_uT, mag.field_b_uT, 'AbsTol', 1.0e-12);
verifyEqual(testCase, sens.gnss.pos_ned_m, gnss.pos_ned_m, 'AbsTol', 1.0e-12);
verifyEqual(testCase, sens.gnss.vel_ned_mps, gnss.vel_ned_mps, 'AbsTol', 1.0e-12);
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
