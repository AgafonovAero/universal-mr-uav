%% RUN_CASE_HOVER_WITH_ESTIMATOR Execute the hover case with estimator logging.
% Description:
%   Runs the deterministic hover case through uav.sim.run_case_with_estimator
%   and prints compact end-of-run diagnostics from the estimator history.
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
%   The hover command is constant and all sensor noise is disabled.

params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = params.demo.case_hover_t_final_s;
case_cfg.command_fun = @local_hover_command;

log = uav.sim.run_case_with_estimator(case_cfg);
final_est = log.estimator(end);

fprintf('Run-case hover with estimator:\n');
fprintf('  final estimated Euler [rad] : [%.6f %.6f %.6f]\n', ...
    final_est.euler_rpy_rad);
fprintf('  final estimated altitude [m]: %.6f\n', final_est.alt_m);
fprintf('  final estimated vz [m/s]    : %.6f\n', final_est.vz_mps);
fprintf('  final estimated quat norm[-]: %.12f\n', log.quat_norm_est(end));

demo = struct();
demo.case_cfg = case_cfg;
demo.log = log;

assignin('base', 'case_hover_with_estimator_demo', demo);

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
