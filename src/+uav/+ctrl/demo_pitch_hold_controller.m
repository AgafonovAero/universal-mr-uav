function [motor_cmd_radps, ctrl_state, diag] = demo_pitch_hold_controller( ...
        ctrl_input, ctrl_state, dt_s, params, cfg)
%DEMO_PITCH_HOLD_CONTROLLER Demo-level estimator-driven altitude/pitch hold.
% Description:
%   Uses estimator altitude and attitude together with measured body rates
%   to build a minimal closed-loop demo controller for safe-altitude climb
%   and pitch tracking. The function never reads true plant state and is
%   intended only for transparent verification scenarios.
%
% Inputs:
%   ctrl_input - struct with estimator, sensors, and reference substructs
%   ctrl_state - controller state struct with rate_pid_state
%   dt_s       - controller sample time [s]
%   params     - vehicle parameter struct
%   cfg        - controller configuration struct
%
% Outputs:
%   motor_cmd_radps - 4x1 motor speed command [rad/s]
%   ctrl_state      - updated controller state
%   diag            - compact controller diagnostics
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   ctrl_input.reference provides altitude and vertical-speed references.

ctrl_input = local_validate_ctrl_input(ctrl_input);
ctrl_state = local_prepare_state(ctrl_state);
cfg = local_validate_cfg(cfg, params);

est = ctrl_input.estimator;
sens = ctrl_input.sensors;
ref = ctrl_input.reference;

est_euler_rpy_rad = est.euler_rpy_rad(:);
gyro_b_radps = sens.imu.gyro_b_radps(:);

altitude_error_m = ref.altitude_ref_m - est.alt_m;
vertical_speed_error_mps = ref.vertical_speed_ref_mps - est.vz_mps;
up_accel_cmd_mps2 = cfg.altitude_kp_per_s2 * altitude_error_m + ...
    cfg.vertical_speed_kp_per_s * vertical_speed_error_mps;
up_accel_cmd_mps2 = local_clip(up_accel_cmd_mps2, ...
    -cfg.up_accel_limit_mps2, cfg.up_accel_limit_mps2);

body_z_up_gain = local_body_z_up_gain(est.q_nb, cfg.min_body_z_up_gain);
total_thrust_N = params.mass_kg * ...
    (params.gravity_mps2 + up_accel_cmd_mps2) ./ body_z_up_gain;
total_thrust_N = local_clip(total_thrust_N, ...
    cfg.total_thrust_min_N, cfg.total_thrust_max_N);

attitude_error_rad = [ ...
    local_wrap_angle_pi(ref.roll_ref_rad - est_euler_rpy_rad(1)); ...
    local_wrap_angle_pi(ref.pitch_ref_rad - est_euler_rpy_rad(2)); ...
    local_wrap_angle_pi(ref.yaw_ref_rad - est_euler_rpy_rad(3))];
rate_cmd_radps = cfg.angle_kp_radps_per_rad(:) .* attitude_error_rad;
rate_cmd_radps = local_clip(rate_cmd_radps, ...
    -cfg.body_rate_cmd_limit_radps(:), cfg.body_rate_cmd_limit_radps(:));

[body_moments_Nm, ctrl_state.rate_pid_state] = uav.ctrl.rate_pid( ...
    rate_cmd_radps, gyro_b_radps, ctrl_state.rate_pid_state, ...
    cfg.rate_pid_gains, dt_s, cfg.rate_pid_limits);

[motor_cmd_radps, ~] = uav.vmg.mixer_quad_x(total_thrust_N, body_moments_Nm, params);
motor_cmd_radps = local_clip(motor_cmd_radps, ...
    params.motor.omega_min_radps, params.motor.omega_max_radps);

diag = struct();
diag.total_thrust_N = total_thrust_N;
diag.up_accel_cmd_mps2 = up_accel_cmd_mps2;
diag.altitude_error_m = altitude_error_m;
diag.vertical_speed_error_mps = vertical_speed_error_mps;
diag.attitude_error_rad = attitude_error_rad;
diag.rate_cmd_radps = rate_cmd_radps;
diag.body_moments_Nm = body_moments_Nm;
end

function ctrl_input = local_validate_ctrl_input(ctrl_input)
%LOCAL_VALIDATE_CTRL_INPUT Validate the estimator-driven controller input.

if ~isstruct(ctrl_input) || ~isscalar(ctrl_input)
    error('uav:ctrl:demo_pitch_hold_controller:InputType', ...
        'Expected ctrl_input to be a scalar struct.');
end

required_fields = {'estimator', 'sensors', 'reference'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(ctrl_input, field_name)
        error('uav:ctrl:demo_pitch_hold_controller:MissingInputField', ...
            'Expected ctrl_input.%s to be present.', field_name);
    end
end

ctrl_input.estimator = local_validate_estimator(ctrl_input.estimator);
ctrl_input.sensors = local_validate_sensors(ctrl_input.sensors);
ctrl_input.reference = local_validate_reference(ctrl_input.reference);
end

function est = local_validate_estimator(est)
%LOCAL_VALIDATE_ESTIMATOR Validate required estimator outputs.

if ~isstruct(est) || ~isscalar(est)
    error('uav:ctrl:demo_pitch_hold_controller:EstimatorType', ...
        'Expected ctrl_input.estimator to be a scalar struct.');
end

est.q_nb = local_validate_vec(est, 'q_nb', 4, 'ctrl_input.estimator.q_nb');
est.euler_rpy_rad = local_validate_vec(est, 'euler_rpy_rad', 3, ...
    'ctrl_input.estimator.euler_rpy_rad');
est.alt_m = local_validate_scalar(est, 'alt_m', 'ctrl_input.estimator.alt_m');
est.vz_mps = local_validate_scalar(est, 'vz_mps', 'ctrl_input.estimator.vz_mps');
end

function sens = local_validate_sensors(sens)
%LOCAL_VALIDATE_SENSORS Validate required sensor fields.

if ~isstruct(sens) || ~isscalar(sens) || ~isfield(sens, 'imu')
    error('uav:ctrl:demo_pitch_hold_controller:SensorsType', ...
        'Expected ctrl_input.sensors.imu to be present.');
end

sens.imu.gyro_b_radps = local_validate_vec(sens.imu, 'gyro_b_radps', 3, ...
    'ctrl_input.sensors.imu.gyro_b_radps');
end

function ref = local_validate_reference(ref)
%LOCAL_VALIDATE_REFERENCE Validate controller reference channels.

if ~isstruct(ref) || ~isscalar(ref)
    error('uav:ctrl:demo_pitch_hold_controller:ReferenceType', ...
        'Expected ctrl_input.reference to be a scalar struct.');
end

ref.altitude_ref_m = local_validate_scalar(ref, 'altitude_ref_m', ...
    'ctrl_input.reference.altitude_ref_m');
ref.vertical_speed_ref_mps = local_validate_scalar(ref, ...
    'vertical_speed_ref_mps', 'ctrl_input.reference.vertical_speed_ref_mps');
ref.roll_ref_rad = local_get_scalar(ref, 'roll_ref_rad', 0.0);
ref.pitch_ref_rad = local_get_scalar(ref, 'pitch_ref_rad', 0.0);
ref.yaw_ref_rad = local_get_scalar(ref, 'yaw_ref_rad', 0.0);
end

function cfg = local_validate_cfg(cfg, params)
%LOCAL_VALIDATE_CFG Validate the demo controller configuration.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ctrl:demo_pitch_hold_controller:CfgType', ...
        'Expected cfg to be a scalar struct.');
end

cfg.altitude_kp_per_s2 = local_get_scalar(cfg, 'altitude_kp_per_s2', 0.0);
cfg.vertical_speed_kp_per_s = local_get_scalar(cfg, 'vertical_speed_kp_per_s', 0.0);
cfg.up_accel_limit_mps2 = local_get_scalar(cfg, 'up_accel_limit_mps2', 1.0e6);
cfg.min_body_z_up_gain = local_get_scalar(cfg, 'min_body_z_up_gain', 0.35);
cfg.total_thrust_min_N = local_get_scalar(cfg, 'total_thrust_min_N', 0.0);
cfg.total_thrust_max_N = local_get_scalar(cfg, 'total_thrust_max_N', 1.0e6);
cfg.angle_kp_radps_per_rad = local_get_vec(cfg, 'angle_kp_radps_per_rad', 3, ...
    zeros(3, 1));
cfg.body_rate_cmd_limit_radps = local_get_vec(cfg, ...
    'body_rate_cmd_limit_radps', 3, 1.0e6 .* ones(3, 1));
cfg.rate_pid_gains = local_get_struct(cfg, 'rate_pid_gains', params.rate_pid_gains);

body_moment_limit_Nm = local_get_vec(cfg, ...
    'body_moment_limit_Nm', 3, 1.0e6 .* ones(3, 1));
cfg.rate_pid_limits = struct( ...
    'integrator_min', -body_moment_limit_Nm, ...
    'integrator_max', body_moment_limit_Nm, ...
    'output_min', -body_moment_limit_Nm, ...
    'output_max', body_moment_limit_Nm);
end

function ctrl_state = local_prepare_state(ctrl_state)
%LOCAL_PREPARE_STATE Build one deterministic controller-state struct.

if nargin < 1 || isempty(ctrl_state)
    ctrl_state = struct();
end

if ~isfield(ctrl_state, 'rate_pid_state') || isempty(ctrl_state.rate_pid_state)
    ctrl_state.rate_pid_state = struct();
end
end

function value = local_validate_scalar(data, field_name, label_name)
%LOCAL_VALIDATE_SCALAR Validate one scalar numeric field.

if ~isfield(data, field_name)
    error('uav:ctrl:demo_pitch_hold_controller:MissingField', ...
        'Expected %s to be present.', label_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, {'real', 'scalar', 'finite'}, ...
    mfilename, label_name);
end

function vec = local_validate_vec(data, field_name, expected_len, label_name)
%LOCAL_VALIDATE_VEC Validate one fixed-length numeric vector field.

if ~isfield(data, field_name)
    error('uav:ctrl:demo_pitch_hold_controller:MissingField', ...
        'Expected %s to be present.', label_name);
end

vec = data.(field_name);
validateattributes(vec, {'numeric'}, {'real', 'finite', 'numel', expected_len}, ...
    mfilename, label_name);
vec = vec(:);
end

function value = local_get_scalar(data, field_name, default_value)
%LOCAL_GET_SCALAR Read one optional scalar field with validation.

if isfield(data, field_name) && ~isempty(data.(field_name))
    value = data.(field_name);
else
    value = default_value;
end

validateattributes(value, {'numeric'}, {'real', 'scalar', 'finite'}, ...
    mfilename, field_name);
end

function vec = local_get_vec(data, field_name, expected_len, default_value)
%LOCAL_GET_VEC Read one optional vector field with validation.

if isfield(data, field_name) && ~isempty(data.(field_name))
    vec = data.(field_name);
else
    vec = default_value;
end

validateattributes(vec, {'numeric'}, {'real', 'finite'}, mfilename, field_name);
if isscalar(vec)
    vec = repmat(vec, expected_len, 1);
else
    vec = vec(:);
    if numel(vec) ~= expected_len
        error('uav:ctrl:demo_pitch_hold_controller:VectorLength', ...
            'Expected %s to have %d elements, got %d.', ...
            field_name, expected_len, numel(vec));
    end
end
end

function value = local_get_struct(data, field_name, default_value)
%LOCAL_GET_STRUCT Read one optional struct field.

if isfield(data, field_name) && ~isempty(data.(field_name))
    value = data.(field_name);
else
    value = default_value;
end

if ~isstruct(value) || ~isscalar(value)
    error('uav:ctrl:demo_pitch_hold_controller:StructField', ...
        'Expected %s to be a scalar struct.', field_name);
end
end

function gain = local_body_z_up_gain(q_nb, min_gain)
%LOCAL_BODY_Z_UP_GAIN Return the estimated vertical thrust projection.

c_nb = uav.core.quat_to_dcm(q_nb);
gain = max(c_nb(3, 3), min_gain);
end

function value = local_clip(value, min_value, max_value)
%LOCAL_CLIP Saturate a scalar or vector between two bounds.

value = min(max(value, min_value), max_value);
end

function wrapped_rad = local_wrap_angle_pi(angle_rad)
%LOCAL_WRAP_ANGLE_PI Wrap one angle to [-pi, pi].

wrapped_rad = atan2(sin(angle_rad), cos(angle_rad));
end
