function [att_est, diag] = attitude_cf_step(est_prev, sens, dt_s, params)
%ATTITUDE_CF_STEP Advance the minimal attitude complementary filter.
% Description:
%   Propagates quaternion attitude with measured body gyro rates and then
%   applies low-frequency complementary corrections from accelerometer
%   roll/pitch and magnetometer yaw observations.
%
%   Accelerometer feedback is weighted by a specific-force consistency
%   metric. This keeps the estimator simple and transparent while avoiding
%   the common failure mode where sustained accelerated flight pulls the
%   estimated pitch and roll back toward level.
%
% Inputs:
%   est_prev - previous attitude estimator state
%   sens     - sensor sample with imu and mag substructs
%   dt_s     - estimator time step [s]
%   params   - parameter struct with estimator.attitude settings
%
% Outputs:
%   att_est - updated attitude estimate with q_nb and euler_rpy_rad
%   diag    - diagnostic struct with quaternion norm and correction norms
%
% Units:
%   q_nb [-]
%   euler_rpy_rad [rad]
%   gyro_b_radps [rad/s]
%
% Assumptions:
%   Accelerometer measurements are specific force in body axes and are
%   used only as a low-frequency roll/pitch reference while the measured
%   specific force remains consistent with gravity-only flight.

% Validate the previous state and sensor channels used by the filter.
est_prev = local_validate_attitude_state(est_prev);

gyro_b_radps = local_validate_vec3( ...
    sens, ...
    {'imu', 'gyro_b_radps'}, ...
    'sens.imu.gyro_b_radps');

accel_b_mps2 = local_validate_vec3( ...
    sens, ...
    {'imu', 'accel_b_mps2'}, ...
    'sens.imu.accel_b_mps2');

mag_b_uT = local_validate_vec3( ...
    sens, ...
    {'mag', 'field_b_uT'}, ...
    'sens.mag.field_b_uT');

validateattributes( ...
    dt_s, ...
    {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ...
    'dt_s');

local_validate_attitude_params(params);

gravity_ned_mps2 = uav.env.gravity_ned(params.gravity_mps2);

% Stage 1. Quaternion propagation by measured body rates only.
q_pred = local_integrate_quaternion( ...
    est_prev.q_nb, ...
    gyro_b_radps, ...
    dt_s);

euler_pred = local_quat_to_euler_rpy(q_pred);

% Stage 2. Accelerometer correction only when specific-force consistency
% still supports the gravity-only assumption.
[roll_meas_rad, pitch_meas_rad, acc_valid] = local_accel_attitude( ...
    accel_b_mps2);

if acc_valid
    accel_consistency_metric = local_accel_consistency_metric( ...
        q_pred, ...
        accel_b_mps2, ...
        gravity_ned_mps2);

    accel_correction_weight = local_specific_force_weight( ...
        accel_consistency_metric, ...
        params);

    acc_alpha = local_complementary_alpha( ...
        params.estimator.attitude.k_acc, ...
        dt_s);

    acc_alpha = acc_alpha .* accel_correction_weight;

    acc_error_rad = [ ...
        local_wrap_angle_pi(roll_meas_rad - euler_pred(1)); ...
        local_wrap_angle_pi(pitch_meas_rad - euler_pred(2)); ...
        0.0];
else
    accel_consistency_metric = nan;
    accel_correction_weight = 0.0;
    acc_alpha = 0.0;
    acc_error_rad = zeros(3, 1);
end

euler_corr = euler_pred + acc_alpha .* acc_error_rad;

euler_corr(1:2) = arrayfun( ...
    @local_wrap_angle_pi, ...
    euler_corr(1:2));

% Stage 3. Magnetometer yaw correction on top of corrected roll and pitch.
[yaw_meas_rad, mag_valid] = local_mag_yaw( ...
    mag_b_uT, ...
    euler_corr(1:2), ...
    params);

if mag_valid
    mag_alpha = local_complementary_alpha( ...
        params.estimator.attitude.k_mag, ...
        dt_s);

    mag_error_rad = [ ...
        0.0; ...
        0.0; ...
        local_wrap_angle_pi(yaw_meas_rad - euler_corr(3))];
else
    mag_alpha = 0.0;
    mag_error_rad = zeros(3, 1);
end

% Stage 4. Pack the updated estimate and transparent diagnostics.
euler_corr(3) = local_wrap_angle_pi( ...
    euler_corr(3) + mag_alpha * mag_error_rad(3));

q_est = uav.core.quat_normalize( ...
    local_euler_to_quat_rpy(euler_corr));

euler_est = local_quat_to_euler_rpy(q_est);

att_est = struct();
att_est.q_nb = q_est;
att_est.euler_rpy_rad = euler_est;

diag = struct();
diag.quat_norm = norm(q_est);
diag.accel_correction_weight = accel_correction_weight;
diag.accel_consistency_metric = accel_consistency_metric;
diag.acc_correction_norm = norm(acc_alpha .* acc_error_rad(1:2));
diag.mag_correction_norm = abs(mag_alpha * mag_error_rad(3));
end


function est_prev = local_validate_attitude_state(est_prev)
%LOCAL_VALIDATE_ATTITUDE_STATE Validate the attitude estimator state.

if ~isstruct(est_prev) || ~isscalar(est_prev)
    error( ...
        'uav:est:attitude_cf_step:StateType', ...
        'Expected est_prev to be a scalar struct.');
end

est_prev.q_nb = local_validate_numeric_vec( ...
    est_prev, ...
    'q_nb', ...
    4);

est_prev.euler_rpy_rad = local_validate_numeric_vec( ...
    est_prev, ...
    'euler_rpy_rad', ...
    3);

est_prev.q_nb = uav.core.quat_normalize(est_prev.q_nb);
end


function local_validate_attitude_params(params)
%LOCAL_VALIDATE_ATTITUDE_PARAMS Validate required attitude estimator fields.

if ~isstruct(params) || ...
        ~isfield(params, 'estimator') || ...
        ~isfield(params.estimator, 'attitude')
    error( ...
        'uav:est:attitude_cf_step:MissingParams', ...
        'Expected params.estimator.attitude to be present.');
end

validateattributes( ...
    params.estimator.attitude.k_acc, ...
    {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ...
    'params.estimator.attitude.k_acc');

validateattributes( ...
    params.estimator.attitude.k_mag, ...
    {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ...
    'params.estimator.attitude.k_mag');

validateattributes( ...
    params.estimator.attitude.accel_consistency_full_weight_mps2, ...
    {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ...
    'params.estimator.attitude.accel_consistency_full_weight_mps2');

validateattributes( ...
    params.estimator.attitude.accel_consistency_zero_weight_mps2, ...
    {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, ...
    'params.estimator.attitude.accel_consistency_zero_weight_mps2');

if params.estimator.attitude.accel_consistency_zero_weight_mps2 < ...
        params.estimator.attitude.accel_consistency_full_weight_mps2
    error( ...
        'uav:est:attitude_cf_step:AccelConsistencyThresholdOrder', ...
        ['Expected accel_consistency_zero_weight_mps2 to be greater than ' ...
        'or equal to accel_consistency_full_weight_mps2.']);
end
end


function value = local_validate_vec3(data, fields, label_name)
%LOCAL_VALIDATE_VEC3 Validate one nested 3x1 numeric vector field.

value = data;

for k = 1:numel(fields)
    field_name = fields{k};

    if ~isstruct(value) || ~isfield(value, field_name)
        error( ...
            'uav:est:attitude_cf_step:MissingField', ...
            'Expected %s to be present.', ...
            label_name);
    end

    value = value.(field_name);
end

validateattributes( ...
    value, ...
    {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, ...
    mfilename, ...
    label_name);

value = value(:);
end


function value = local_validate_numeric_vec(data, field_name, expected_len)
%LOCAL_VALIDATE_NUMERIC_VEC Validate one numeric estimator state field.

if ~isfield(data, field_name)
    error( ...
        'uav:est:attitude_cf_step:MissingStateField', ...
        'Expected est_prev.%s to be present.', ...
        field_name);
end

value = data.(field_name);

validateattributes( ...
    value, ...
    {'numeric'}, ...
    {'real', 'finite', 'numel', expected_len}, ...
    mfilename, ...
    ['est_prev.' field_name]);

value = value(:);
end


function alpha = local_complementary_alpha(gain_per_s, dt_s)
%LOCAL_COMPLEMENTARY_ALPHA Convert a rate gain into a blend coefficient.

alpha = min(max(gain_per_s * dt_s, 0.0), 1.0);
end


function metric_mps2 = local_accel_consistency_metric( ...
        q_pred, accel_b_mps2, gravity_ned_mps2)
%LOCAL_ACCEL_CONSISTENCY_METRIC Compare measured and gravity-only force.

c_nb_pred = uav.core.quat_to_dcm(q_pred);

expected_accel_b_mps2 = -c_nb_pred.' * gravity_ned_mps2(:);

metric_mps2 = norm(accel_b_mps2 - expected_accel_b_mps2);
end


function weight = local_specific_force_weight(metric_mps2, params)
%LOCAL_SPECIFIC_FORCE_WEIGHT Gate accelerometer correction by consistency.

full_weight_mps2 = ...
    params.estimator.attitude.accel_consistency_full_weight_mps2;

zero_weight_mps2 = ...
    params.estimator.attitude.accel_consistency_zero_weight_mps2;

if metric_mps2 <= full_weight_mps2
    weight = 1.0;
elseif metric_mps2 >= zero_weight_mps2
    weight = 0.0;
else
    blend = (zero_weight_mps2 - metric_mps2) / ...
        max(zero_weight_mps2 - full_weight_mps2, eps);

    weight = blend * blend * (3.0 - 2.0 * blend);
end
end


function q_next = local_integrate_quaternion(q_prev, gyro_b_radps, dt_s)
%LOCAL_INTEGRATE_QUATERNION Propagate quaternion attitude with gyro rates.

q_dot = 0.5 .* local_quat_multiply( ...
    q_prev, ...
    [0.0; gyro_b_radps]);

q_next = uav.core.quat_normalize(q_prev + dt_s .* q_dot);
end


function q_out = local_quat_multiply(q_left, q_right)
%LOCAL_QUAT_MULTIPLY Multiply two scalar-first Hamilton quaternions.

s_left = q_left(1);
v_left = q_left(2:4);

s_right = q_right(1);
v_right = q_right(2:4);

q_out = [ ...
    s_left * s_right - dot(v_left, v_right); ...
    s_left * v_right + s_right * v_left + cross(v_left, v_right)];
end


function [roll_rad, pitch_rad, is_valid] = local_accel_attitude(accel_b_mps2)
%LOCAL_ACCEL_ATTITUDE Estimate roll and pitch from body specific force.

accel_norm = norm(accel_b_mps2);

if accel_norm <= eps
    roll_rad = 0.0;
    pitch_rad = 0.0;
    is_valid = false;
    return;
end

roll_rad = atan2(-accel_b_mps2(2), -accel_b_mps2(3));

pitch_rad = asin( ...
    max( ...
        min(accel_b_mps2(1) ./ accel_norm, 1.0), ...
        -1.0));

is_valid = true;
end


function [yaw_rad, is_valid] = local_mag_yaw(mag_b_uT, roll_pitch_rad, params)
%LOCAL_MAG_YAW Estimate yaw from body magnetometer and roll/pitch.

ref_heading_rad = atan2( ...
    params.env.mag_ned_uT(2), ...
    params.env.mag_ned_uT(1));

if norm(params.env.mag_ned_uT(1:2)) <= eps || norm(mag_b_uT) <= eps
    yaw_rad = 0.0;
    is_valid = false;
    return;
end

euler_rpy_rad = [roll_pitch_rad(:); 0.0];

q_rp = local_euler_to_quat_rpy(euler_rpy_rad);
c_rp = uav.core.quat_to_dcm(q_rp);
mag_level_ned_uT = c_rp * mag_b_uT;

if norm(mag_level_ned_uT(1:2)) <= eps
    yaw_rad = 0.0;
    is_valid = false;
    return;
end

yaw_rad = local_wrap_angle_pi( ...
    ref_heading_rad - atan2(mag_level_ned_uT(2), mag_level_ned_uT(1)));

is_valid = true;
end


function q_nb = local_euler_to_quat_rpy(euler_rpy_rad)
%LOCAL_EULER_TO_QUAT_RPY Convert roll, pitch, yaw into q_nb.

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


function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
%LOCAL_QUAT_TO_EULER_RPY Convert q_nb into roll, pitch, yaw angles.

q_nb = uav.core.quat_normalize(q_nb);

qw = q_nb(1);
qx = q_nb(2);
qy = q_nb(3);
qz = q_nb(4);

sin_pitch = 2.0 .* (qw * qy - qz * qx);

euler_rpy_rad = [ ...
    atan2( ...
        2.0 .* (qw * qx + qy * qz), ...
        1.0 - 2.0 .* (qx^2 + qy^2)); ...
    asin(max(min(sin_pitch, 1.0), -1.0)); ...
    atan2( ...
        2.0 .* (qw * qz + qx * qy), ...
        1.0 - 2.0 .* (qy^2 + qz^2))];
end


function wrapped_rad = local_wrap_angle_pi(angle_rad)
%LOCAL_WRAP_ANGLE_PI Wrap one angle to [-pi, pi].

wrapped_rad = atan2(sin(angle_rad), cos(angle_rad));
end
