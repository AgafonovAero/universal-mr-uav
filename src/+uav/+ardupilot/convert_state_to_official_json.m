function [sample, diag] = convert_state_to_official_json(state, sensors, time_s, params, cfg, varargin)
%CONVERT_STATE_TO_OFFICIAL_JSON Преобразовать состояние в официальный JSON-кадр.
% Назначение:
%   Формирует минимальный набор величин для официального MATLAB backend
%   ArduPilot: timestamp, imu.gyro, imu.accel_body, position, velocity и
%   attitude. Формула accel_body приведена к официальному примеру
%   SIM_multicopter.m.
%
% Входы:
%   state   - каноническая структура состояния объекта
%   sensors - снимок подсистемы датчиков
%   time_s  - абсолютное физическое время [s]
%   params  - параметры модели
%   cfg     - конфигурация ArduPilot JSON
%
% Параметры:
%   'SnapshotDiag' - диагностическая структура с полем forces_b_N
%
% Выходы:
%   sample - структура с полями timestamp, imu, position, velocity,
%            attitude и optional quaternion
%   diag   - структура промежуточных величин преобразования
%
% Единицы измерения:
%   SI, земная система координат NED, связанная система координат FRD
%
% Допущения:
%   q_nb переводит векторы из связанной системы координат в NED.

if nargin < 5 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

parser = inputParser();
parser.addParameter('SnapshotDiag', struct(), @(x) isstruct(x) && isscalar(x));
parser.parse(varargin{:});
snapshot_diag = parser.Results.SnapshotDiag;

state = uav.core.state_validate(state);
sensors = local_validate_sensors(sensors);

validateattributes(time_s, {'numeric'}, {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'time_s');
validateattributes(params.gravity_mps2, {'numeric'}, {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.gravity_mps2');
validateattributes(params.mass_kg, {'numeric'}, {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.mass_kg');

c_nb = uav.core.quat_to_dcm(state.q_nb);
velocity_ned_mps = c_nb * state.v_b_mps;
attitude_rpy_rad = local_quat_to_euler_rpy(state.q_nb);
[accel_body_mps2, accel_diag] = local_compute_official_accel_body( ...
    state, snapshot_diag, params, sensors);

sample = struct();
sample.timestamp = double(time_s);
sample.imu = struct( ...
    'gyro', reshape(double(sensors.imu.gyro_b_radps(:)), 1, 3), ...
    'accel_body', reshape(double(accel_body_mps2(:)), 1, 3));
sample.position = reshape(double(state.p_ned_m(:)), 1, 3);
sample.velocity = reshape(double(velocity_ned_mps(:)), 1, 3);
sample.attitude = reshape(double(attitude_rpy_rad(:)), 1, 3);

if isfield(cfg, 'json_send_quaternion') && logical(cfg.json_send_quaternion)
    sample.quaternion = reshape(double(state.q_nb(:)), 1, 4);
end

diag = struct();
diag.velocity_ned_mps = velocity_ned_mps;
diag.attitude_rpy_rad = attitude_rpy_rad;
diag.accel_diag = accel_diag;
end

function sensors = local_validate_sensors(sensors)
if ~isstruct(sensors) || ~isscalar(sensors) || ~isfield(sensors, 'imu')
    error('uav:ardupilot:convert_state_to_official_json:SensorsType', ...
        'Ожидалась структура sensors с полем imu.');
end

validateattributes(sensors.imu.gyro_b_radps, {'numeric'}, {'real', 'finite', 'numel', 3}, ...
    mfilename, 'sensors.imu.gyro_b_radps');
validateattributes(sensors.imu.accel_b_mps2, {'numeric'}, {'real', 'finite', 'numel', 3}, ...
    mfilename, 'sensors.imu.accel_b_mps2');
end

function [accel_body_mps2, diag] = local_compute_official_accel_body(state, snapshot_diag, params, sensors)
if isfield(snapshot_diag, 'forces_b_N') && ~isempty(snapshot_diag.forces_b_N)
    forces_b_N = double(snapshot_diag.forces_b_N(:));
else
    forces_b_N = double(sensors.imu.accel_b_mps2(:)) .* double(params.mass_kg);
end

c_nb = uav.core.quat_to_dcm(state.q_nb);
specific_force_body_mps2 = forces_b_N ./ double(params.mass_kg);
gravity_ned_mps2 = [0.0; 0.0; double(params.gravity_mps2)];
accel_ned_mps2 = c_nb * specific_force_body_mps2 + gravity_ned_mps2;

ground_clamped = false;
if double(state.p_ned_m(3)) >= 0.0 && accel_ned_mps2(3) > 0.0
    accel_ned_mps2(3) = 0.0;
    ground_clamped = true;
end

accel_body_mps2 = c_nb.' * (accel_ned_mps2 + [0.0; 0.0; -double(params.gravity_mps2)]);

diag = struct();
diag.forces_b_N = forces_b_N;
diag.specific_force_body_mps2 = specific_force_body_mps2;
diag.accel_ned_mps2 = accel_ned_mps2;
diag.ground_clamped = ground_clamped;
end

function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
q_nb = q_nb(:) ./ norm(q_nb);
q0 = q_nb(1);
q1 = q_nb(2);
q2 = q_nb(3);
q3 = q_nb(4);

roll_rad = atan2(2.0 .* (q0 .* q1 + q2 .* q3), ...
    1.0 - 2.0 .* (q1.^2 + q2.^2));
pitch_arg = 2.0 .* (q0 .* q2 - q3 .* q1);
pitch_arg = min(max(pitch_arg, -1.0), 1.0);
pitch_rad = asin(pitch_arg);
yaw_rad = atan2(2.0 .* (q0 .* q3 + q1 .* q2), ...
    1.0 - 2.0 .* (q2.^2 + q3.^2));
euler_rpy_rad = [roll_rad; pitch_rad; yaw_rad];
end
