function [alt_est, diag] = altitude_filter_step(est_prev, sens, att_est, dt_s, params)
%ALTITUDE_FILTER_STEP Advance the minimal altitude complementary filter.
% Description:
%   Estimates altitude and vertical speed from barometric altitude with an
%   optional IMU-based vertical prediction channel.
%
% Inputs:
%   est_prev - previous altitude estimator state
%   sens     - sensor sample with imu and baro substructs
%   att_est  - attitude estimate with q_nb
%   dt_s     - estimator time step [s]
%   params   - parameter struct with estimator.altitude settings
%
% Outputs:
%   alt_est - updated altitude estimate with alt_m and vz_mps
%   diag    - diagnostic struct with az_ned_mps2 and baro_residual_m
%
% Units:
%   alt_m [m], vz_mps [m/s], az_ned_mps2 [m/s^2]
%
% Assumptions:
%   alt_m and vz_mps are altitude-above-origin and upward vertical speed,
%   while az_ned_mps2 follows the NED down-positive convention.

est_prev = local_validate_altitude_state(est_prev);
baro_alt_m = local_validate_scalar(sens, {'baro', 'alt_m'}, 'sens.baro.alt_m');
accel_b_mps2 = local_validate_vec3(sens, {'imu', 'accel_b_mps2'}, ...
    'sens.imu.accel_b_mps2');
att_q_nb = local_validate_vec3_or4(att_est, 'q_nb', 4);

validateattributes(dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, 'dt_s');
local_validate_altitude_params(params);

damping_factor = max(0.0, 1.0 - params.estimator.altitude.vz_damping * dt_s);
vz_pred_mps = damping_factor * est_prev.vz_mps;
alt_pred_m = est_prev.alt_m + vz_pred_mps * dt_s;
az_ned_mps2 = 0.0;

if logical(params.estimator.altitude.use_imu_prediction)
    c_nb = uav.core.quat_to_dcm(att_q_nb);
    specific_force_ned_mps2 = c_nb * accel_b_mps2;
    accel_ned_mps2 = specific_force_ned_mps2 + ...
        uav.env.gravity_ned(params.gravity_mps2);
    az_ned_mps2 = accel_ned_mps2(3);

    az_up_mps2 = -az_ned_mps2;
    vz_pred_mps = damping_factor * (est_prev.vz_mps + az_up_mps2 * dt_s);
    alt_pred_m = est_prev.alt_m + vz_pred_mps * dt_s;
end

baro_residual_m = baro_alt_m - alt_pred_m;
baro_alpha = min(max(params.estimator.altitude.k_baro * dt_s, 0.0), 1.0);
baro_beta = min(max(0.05 * baro_alpha, 0.0), 1.0);

alt_est = struct();
alt_est.alt_m = alt_pred_m + baro_alpha * baro_residual_m;
alt_est.vz_mps = vz_pred_mps;
if dt_s > 0.0
    alt_est.vz_mps = alt_est.vz_mps + ...
        baro_beta * (baro_residual_m / dt_s);
end

diag = struct();
diag.az_ned_mps2 = az_ned_mps2;
diag.baro_residual_m = baro_residual_m;
end

function est_prev = local_validate_altitude_state(est_prev)
%LOCAL_VALIDATE_ALTITUDE_STATE Validate the altitude estimator state.

if ~isstruct(est_prev) || ~isscalar(est_prev)
    error('uav:est:altitude_filter_step:StateType', ...
        'Expected est_prev to be a scalar struct.');
end

est_prev.alt_m = local_validate_numeric_field(est_prev, 'alt_m');
est_prev.vz_mps = local_validate_numeric_field(est_prev, 'vz_mps');
end

function local_validate_altitude_params(params)
%LOCAL_VALIDATE_ALTITUDE_PARAMS Validate required altitude estimator fields.

if ~isstruct(params) || ~isfield(params, 'estimator') || ...
        ~isfield(params.estimator, 'altitude')
    error('uav:est:altitude_filter_step:MissingParams', ...
        'Expected params.estimator.altitude to be present.');
end

validateattributes(params.estimator.altitude.k_baro, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, ...
    'params.estimator.altitude.k_baro');
validateattributes(params.estimator.altitude.use_imu_prediction, ...
    {'logical', 'numeric'}, {'real', 'scalar', 'finite'}, mfilename, ...
    'params.estimator.altitude.use_imu_prediction');
validateattributes(params.estimator.altitude.vz_damping, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, ...
    'params.estimator.altitude.vz_damping');
end

function value = local_validate_scalar(data, fields, label_name)
%LOCAL_VALIDATE_SCALAR Validate one nested scalar numeric field.

value = data;
for k = 1:numel(fields)
    field_name = fields{k};
    if ~isstruct(value) || ~isfield(value, field_name)
        error('uav:est:altitude_filter_step:MissingField', ...
            'Expected %s to be present.', label_name);
    end
    value = value.(field_name);
end

validateattributes(value, {'numeric'}, {'real', 'scalar', 'finite'}, ...
    mfilename, label_name);
end

function value = local_validate_vec3(data, fields, label_name)
%LOCAL_VALIDATE_VEC3 Validate one nested 3x1 numeric vector field.

value = data;
for k = 1:numel(fields)
    field_name = fields{k};
    if ~isstruct(value) || ~isfield(value, field_name)
        error('uav:est:altitude_filter_step:MissingField', ...
            'Expected %s to be present.', label_name);
    end
    value = value.(field_name);
end

validateattributes(value, {'numeric'}, {'real', 'finite', 'numel', 3}, ...
    mfilename, label_name);
value = value(:);
end

function value = local_validate_vec3_or4(data, field_name, expected_len)
%LOCAL_VALIDATE_VEC3_OR4 Validate one numeric vector field by length.

if ~isstruct(data) || ~isfield(data, field_name)
    error('uav:est:altitude_filter_step:MissingAttitudeField', ...
        'Expected att_est.%s to be present.', field_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, {'real', 'finite', 'numel', expected_len}, ...
    mfilename, ['att_est.' field_name]);
value = value(:);
end

function value = local_validate_numeric_field(data, field_name)
%LOCAL_VALIDATE_NUMERIC_FIELD Validate one scalar estimator state field.

if ~isfield(data, field_name)
    error('uav:est:altitude_filter_step:MissingStateField', ...
        'Expected est_prev.%s to be present.', field_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, {'real', 'scalar', 'finite'}, ...
    mfilename, ['est_prev.' field_name]);
end
