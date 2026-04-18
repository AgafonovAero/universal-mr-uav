%% RUN_ESTIMATOR_SANITY_DEMO Execute a deterministic estimator sanity scenario.
% Description:
%   Runs the minimal estimator layer on a fixed, noise-free synthetic
%   sensor sample to confirm transparent convergence of yaw and altitude.
%
% Inputs:
%   none
%
% Outputs:
%   demo - returned in base workspace when assigned by the caller
%
% Units:
%   SI only, angles printed in radians
%
% Assumptions:
%   The synthetic sample represents zero linear acceleration and zero body
%   rates, so the demo targets estimator sanity rather than plant realism.

params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
dt_s = params.demo.dt_s;
n_steps = 400;

true_euler_rpy_rad = [0.0; 0.0; 15.0 * pi / 180.0];
true_alt_m = 12.0;
sens = local_static_sensor_sample(true_euler_rpy_rad, true_alt_m, params);

est = uav.est.estimator_init(params, sens);
diag = struct();
for k = 1:n_steps
    [est, diag] = uav.est.estimator_step(est, sens, dt_s, params);
end

fprintf('Estimator sanity demo:\n');
fprintf('  estimated roll [rad]        : %.6f\n', est.euler_rpy_rad(1));
fprintf('  estimated pitch [rad]       : %.6f\n', est.euler_rpy_rad(2));
fprintf('  estimated yaw [rad]         : %.6f\n', est.euler_rpy_rad(3));
fprintf('  estimated altitude [m]      : %.6f\n', est.alt_m);
fprintf('  estimated vertical speed [m/s]: %.6f\n', est.vz_mps);
fprintf('  quaternion norm [-]         : %.12f\n', diag.attitude.quat_norm);

demo = struct();
demo.true_euler_rpy_rad = true_euler_rpy_rad;
demo.true_alt_m = true_alt_m;
demo.sensors = sens;
demo.estimator = est;
demo.diag = diag;

assignin('base', 'estimator_sanity_demo', demo);

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

function sens = local_static_sensor_sample(euler_rpy_rad, alt_m, params)
%LOCAL_STATIC_SENSOR_SAMPLE Build one deterministic synthetic sensor sample.

q_nb = local_euler_to_quat_rpy(euler_rpy_rad);
c_nb = uav.core.quat_to_dcm(q_nb);
gravity_ned_mps2 = uav.env.gravity_ned(params.gravity_mps2);

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', -c_nb.' * gravity_ned_mps2, ...
    'gyro_b_radps', zeros(3, 1));
sens.baro = struct( ...
    'alt_m', alt_m, ...
    'pressure_pa', uav.env.isa_pressure_pa(alt_m));
sens.mag = struct( ...
    'field_b_uT', c_nb.' * params.env.mag_ned_uT(:));
sens.gnss = struct( ...
    'pos_ned_m', [0.0; 0.0; -alt_m], ...
    'vel_ned_mps', zeros(3, 1));
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
