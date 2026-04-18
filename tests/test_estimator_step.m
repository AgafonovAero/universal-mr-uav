function tests = test_estimator_step
%TEST_ESTIMATOR_STEP Tests for the combined Stage-1.5 estimator layer.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testEstimatorStepReturnsRequiredFields(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
sens = local_level_sensor_sample(6.0, params);
est_prev = uav.est.estimator_init(params, sens);

[est, diag] = uav.est.estimator_step(est_prev, sens, params.demo.dt_s, params);

verifyTrue(testCase, isfield(est, 'q_nb'));
verifyTrue(testCase, isfield(est, 'euler_rpy_rad'));
verifyTrue(testCase, isfield(est, 'alt_m'));
verifyTrue(testCase, isfield(est, 'vz_mps'));
verifySize(testCase, est.q_nb, [4, 1]);
verifySize(testCase, est.euler_rpy_rad, [3, 1]);
verifyTrue(testCase, isfield(diag, 'attitude'));
verifyTrue(testCase, isfield(diag, 'altitude'));
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.imu.gyro_bias_b_radps = zeros(3, 1);
params.sensors.imu.gyro_noise_std_b_radps = zeros(3, 1);
params.sensors.baro.alt_bias_m = 0.0;
params.sensors.baro.alt_noise_std_m = 0.0;
params.sensors.mag.field_bias_b_uT = zeros(3, 1);
params.sensors.mag.field_noise_std_b_uT = zeros(3, 1);
end

function sens = local_level_sensor_sample(alt_m, params)
%LOCAL_LEVEL_SENSOR_SAMPLE Build one deterministic level sensor sample.

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', [0.0; 0.0; -params.gravity_mps2], ...
    'gyro_b_radps', zeros(3, 1));
sens.baro = struct( ...
    'alt_m', alt_m, ...
    'pressure_pa', uav.env.isa_pressure_pa(alt_m));
sens.mag = struct( ...
    'field_b_uT', params.env.mag_ned_uT(:));
sens.gnss = struct( ...
    'pos_ned_m', [0.0; 0.0; -alt_m], ...
    'vel_ned_mps', zeros(3, 1));
end
