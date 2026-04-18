function tests = test_attitude_cf_step
%TEST_ATTITUDE_CF_STEP Tests for the quaternion complementary attitude filter.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testIdentityZeroGyroPreservesOrientation(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
att_est = uav.est.attitude_cf_init(params);
sens = local_static_sensor_sample([0.0; 0.0; 0.0], params);

[att_est, diag] = uav.est.attitude_cf_step(att_est, sens, params.demo.dt_s, params);

verifyEqual(testCase, att_est.q_nb, [1.0; 0.0; 0.0; 0.0], 'AbsTol', 1.0e-12);
verifyEqual(testCase, att_est.euler_rpy_rad, zeros(3, 1), 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.quat_norm, 1.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.accel_correction_weight, 1.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.accel_consistency_metric, 0.0, 'AbsTol', 1.0e-12);
end

function testQuaternionNormRemainsNearUnity(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
att_est = uav.est.attitude_cf_init(params);
sens = local_static_sensor_sample([0.0; 0.0; 20.0 * pi / 180.0], params);

quat_norm_hist = zeros(300, 1);
for k = 1:numel(quat_norm_hist)
    [att_est, diag] = uav.est.attitude_cf_step(att_est, sens, params.demo.dt_s, params);
    quat_norm_hist(k) = diag.quat_norm;
end

verifyLessThan(testCase, max(abs(quat_norm_hist - 1.0)), 1.0e-12);
end

function testLevelHoverKeepsRollPitchNearZero(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);
state.omega_m_radps = repmat(params.hover_omega_radps, 4, 1);

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);
diag = struct('forces_b_N', fm.forces_b_N);
sens = uav.sensors.sensors_step(state, diag, params);
att_est = uav.est.attitude_cf_init(params);

for k = 1:200
    [att_est, diag] = uav.est.attitude_cf_step(att_est, sens, params.demo.dt_s, params);
end

verifyLessThan(testCase, abs(att_est.euler_rpy_rad(1)), 1.0e-9);
verifyLessThan(testCase, abs(att_est.euler_rpy_rad(2)), 1.0e-9);
verifyGreaterThan(testCase, diag.accel_correction_weight, 0.99);
end

function testAcceleratedPitchDoesNotCollapseBackToLevel(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
pitch_rad = deg2rad(-10.0);
att_est = uav.est.attitude_cf_init(params);
att_est.q_nb = local_euler_to_quat_rpy([0.0; pitch_rad; 0.0]);
att_est.euler_rpy_rad = [0.0; pitch_rad; 0.0];
sens = local_accelerated_pitch_sensor_sample(pitch_rad, params);

for k = 1:200
    [att_est, diag] = uav.est.attitude_cf_step(att_est, sens, params.demo.dt_s, params);
end

verifyLessThan(testCase, abs(att_est.euler_rpy_rad(2) - pitch_rad), deg2rad(0.5));
verifyLessThan(testCase, diag.accel_correction_weight, 0.05);
verifyGreaterThan(testCase, diag.accel_consistency_metric, ...
    params.estimator.attitude.accel_consistency_zero_weight_mps2);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.imu.gyro_bias_b_radps = zeros(3, 1);
params.sensors.imu.gyro_noise_std_b_radps = zeros(3, 1);
params.sensors.mag.field_bias_b_uT = zeros(3, 1);
params.sensors.mag.field_noise_std_b_uT = zeros(3, 1);
end

function sens = local_static_sensor_sample(euler_rpy_rad, params)
%LOCAL_STATIC_SENSOR_SAMPLE Build one deterministic synthetic sensor sample.

q_nb = local_euler_to_quat_rpy(euler_rpy_rad);
c_nb = uav.core.quat_to_dcm(q_nb);
gravity_ned_mps2 = uav.env.gravity_ned(params.gravity_mps2);

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', -c_nb.' * gravity_ned_mps2, ...
    'gyro_b_radps', zeros(3, 1));
sens.mag = struct( ...
    'field_b_uT', c_nb.' * params.env.mag_ned_uT(:));
end

function sens = local_accelerated_pitch_sensor_sample(pitch_rad, params)
%LOCAL_ACCELERATED_PITCH_SENSOR_SAMPLE Build one pitched accelerated sample.

q_nb = local_euler_to_quat_rpy([0.0; pitch_rad; 0.0]);
c_nb = uav.core.quat_to_dcm(q_nb);

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', [0.0; 0.0; -params.gravity_mps2 / cos(pitch_rad)], ...
    'gyro_b_radps', zeros(3, 1));
sens.mag = struct( ...
    'field_b_uT', c_nb.' * params.env.mag_ned_uT(:));
end

function q_nb = local_euler_to_quat_rpy(euler_rpy_rad)
%LOCAL_EULER_TO_QUAT_RPY Convert roll/pitch/yaw into q_nb.

half_rpy_rad = 0.5 .* euler_rpy_rad(:);
cr = cos(half_rpy_rad(1));
sr = sin(half_rpy_rad(1));
cp = cos(half_rpy_rad(2));
sp = sin(half_rpy_rad(2));
cy = cos(half_rpy_rad(3));
sy = sin(half_rpy_rad(3));

q_nb = [ ...
    cr * cp * cy + sr * sp * sy; ...
    sr * cp * cy - cr * sp * sy; ...
    cr * sp * cy + sr * cp * sy; ...
    cr * cp * sy - sr * sp * cy];
q_nb = uav.core.quat_normalize(q_nb);
end
