function [accel_body_json_mps2, diag] = convert_imu_accel_for_json(state, sensors, time_s, params, cfg)
%CONVERT_IMU_ACCEL_FOR_JSON Prepare body-frame acceleration for ArduPilot JSON.
% Description:
%   Converts the internal IMU acceleration sample of the MATLAB model to
%   the body-frame quantity expected by the ArduPilot JSON interface. The
%   function supports several diagnostic conventions and returns a
%   diagnostic struct describing the applied transformation.
%
% Inputs:
%   state   - canonical plant state struct
%   sensors - canonical sensor snapshot struct
%   time_s  - model time [s]
%   params  - vehicle parameter struct with gravity_mps2
%   cfg     - JSON transport configuration struct
%
% Outputs:
%   accel_body_json_mps2 - 3x1 vector imu.accel_body for JSON [m/s^2]
%   diag                 - struct with mode and intermediate vectors
%
% Units:
%   accel_body_json_mps2 [m/s^2], time_s [s]
%
% Assumptions:
%   Quaternion q_nb maps body-frame vectors to the NED frame, while the
%   body frame follows FRD.

if nargin < 5 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

state = uav.core.state_validate(state);
sensors = local_validate_sensors(sensors);
cfg = local_validate_cfg(cfg);

validateattributes(time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'time_s');
validateattributes(params.gravity_mps2, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.gravity_mps2');

accel_current_mps2 = double(sensors.imu.accel_b_mps2(:));
gravity_ned_mps2 = [0.0; 0.0; double(params.gravity_mps2)];
gravity_body_mps2 = uav.core.quat_to_dcm(state.q_nb).' * gravity_ned_mps2;
linear_accel_body_mps2 = accel_current_mps2 + gravity_body_mps2;

diag = struct();
diag.mode = string(cfg.json_accel_mode);
diag.accel_current_mps2 = accel_current_mps2;
diag.gravity_body_mps2 = gravity_body_mps2;
diag.linear_accel_body_mps2 = linear_accel_body_mps2;
diag.ground_candidate = local_is_ground_candidate(state, time_s, cfg);
diag.explanation = "";

switch string(cfg.json_accel_mode)
    case "current"
        accel_body_json_mps2 = accel_current_mps2;
        diag.explanation = "В JSON передан текущий вектор sensors.imu.accel_b_mps2 без преобразования.";
    case "specific_force_frd"
        accel_body_json_mps2 = accel_current_mps2;
        diag.explanation = "В JSON передана удельная сила в связанной системе координат FRD.";
    case "minus_specific_force_frd"
        accel_body_json_mps2 = -accel_current_mps2;
        diag.explanation = "В JSON передана удельная сила с инверсией знака по всем осям.";
    case "linear_accel_body"
        accel_body_json_mps2 = linear_accel_body_mps2;
        diag.explanation = "В JSON передано линейное ускорение в связанной системе координат без выделения удельной силы.";
    case "zero_on_ground"
        if diag.ground_candidate
            accel_body_json_mps2 = -gravity_body_mps2;
            diag.explanation = "До выхода из наземного состояния в JSON передана удельная сила покоящегося аппарата: -g в связанной системе координат.";
        else
            accel_body_json_mps2 = accel_current_mps2;
            diag.explanation = "Наземное состояние не подтверждено; в JSON передан текущий вектор sensors.imu.accel_b_mps2.";
        end
    otherwise
        error('uav:ardupilot:convert_imu_accel_for_json:Mode', ...
            'Неподдерживаемый режим json_accel_mode: %s.', ...
            string(cfg.json_accel_mode));
end

validateattributes(accel_body_json_mps2, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, ...
    mfilename, 'accel_body_json_mps2');
accel_body_json_mps2 = accel_body_json_mps2(:);
end

function sensors = local_validate_sensors(sensors)
%LOCAL_VALIDATE_SENSORS Validate required IMU fields.

if ~isstruct(sensors) || ~isscalar(sensors) || ~isfield(sensors, 'imu')
    error('uav:ardupilot:convert_imu_accel_for_json:Sensors', ...
        'Expected sensors to contain sensors.imu.');
end

validateattributes(sensors.imu.accel_b_mps2, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, ...
    mfilename, 'sensors.imu.accel_b_mps2');
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Validate required JSON configuration fields.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:convert_imu_accel_for_json:Cfg', ...
        'Expected cfg to be a scalar struct.');
end

required_fields = { ...
    'json_accel_mode', ...
    'json_ground_contact_window_s', ...
    'json_ground_contact_alt_tol_m', ...
    'json_ground_contact_speed_tol_mps', ...
    'json_ground_contact_rate_tol_radps', ...
    'json_ground_contact_motor_tol_radps'};

for idx = 1:numel(required_fields)
    field_name = required_fields{idx};
    if ~isfield(cfg, field_name)
        error('uav:ardupilot:convert_imu_accel_for_json:CfgField', ...
            'Expected cfg.%s to be present.', field_name);
    end
end
end

function is_ground = local_is_ground_candidate(state, time_s, cfg)
%LOCAL_IS_GROUND_CANDIDATE Detect a near-ground pre-arm state.

is_ground = time_s <= double(cfg.json_ground_contact_window_s) ...
    && abs(double(state.p_ned_m(3))) <= double(cfg.json_ground_contact_alt_tol_m) ...
    && norm(double(state.v_b_mps(:))) <= double(cfg.json_ground_contact_speed_tol_mps) ...
    && norm(double(state.w_b_radps(:))) <= double(cfg.json_ground_contact_rate_tol_radps) ...
    && max(abs(double(state.omega_m_radps(:)))) <= double(cfg.json_ground_contact_motor_tol_radps);
end
