function tests = test_altitude_filter_step
%TEST_ALTITUDE_FILTER_STEP Tests for the minimal altitude filter.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testAltitudeEstimateConvergesToBaro(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
params.estimator.altitude.use_imu_prediction = false;
alt_est = uav.est.altitude_filter_init(params);
att_est = struct('q_nb', [1.0; 0.0; 0.0; 0.0]);
sens = local_level_sensor_sample(15.0, params);

for k = 1:400
    [alt_est, ~] = uav.est.altitude_filter_step( ...
        alt_est, sens, att_est, params.demo.dt_s, params);
end

verifyEqual(testCase, alt_est.alt_m, 15.0, 'AbsTol', 1.0e-2);
end

function testZeroVerticalAccelerationKeepsVzBounded(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
alt_est = uav.est.altitude_filter_init(params);
alt_est.alt_m = 8.0;
att_est = struct('q_nb', [1.0; 0.0; 0.0; 0.0]);
sens = local_level_sensor_sample(8.0, params);

for k = 1:400
    [alt_est, diag] = uav.est.altitude_filter_step( ...
        alt_est, sens, att_est, params.demo.dt_s, params);
end

verifyEqual(testCase, diag.az_ned_mps2, 0.0, 'AbsTol', 1.0e-12);
verifyLessThan(testCase, abs(alt_est.vz_mps), 1.0e-6);
verifyEqual(testCase, alt_est.alt_m, 8.0, 'AbsTol', 1.0e-6);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.baro.alt_bias_m = 0.0;
params.sensors.baro.alt_noise_std_m = 0.0;
end

function sens = local_level_sensor_sample(alt_m, params)
%LOCAL_LEVEL_SENSOR_SAMPLE Build one level, zero-acceleration sensor sample.

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', [0.0; 0.0; -params.gravity_mps2], ...
    'gyro_b_radps', zeros(3, 1));
sens.baro = struct( ...
    'alt_m', alt_m, ...
    'pressure_pa', uav.env.isa_pressure_pa(alt_m));
end
