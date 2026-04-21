function json_frame = build_official_json_frame(sample, cfg)
%BUILD_OFFICIAL_JSON_FRAME Собрать JSON-структуру официального MATLAB backend.
% Назначение:
%   Нормализует подготовленный sample в минимальную структуру полей,
%   требуемых официальным JSON интерфейсом ArduPilot.
%
% Входы:
%   sample - структура с полями timestamp, imu, position, velocity,
%            attitude и optional quaternion
%   cfg    - конфигурация ArduPilot JSON
%
% Выходы:
%   json_frame - структура, готовая к jsonencode
%
% Единицы измерения:
%   SI, attitude в радианах
%
% Допущения:
%   timestamp задается абсолютным физическим временем, а не шагом dt.

if nargin < 2 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

if ~isstruct(sample) || ~isscalar(sample)
    error('uav:ardupilot:build_official_json_frame:SampleType', ...
        'Ожидалась скалярная структура sample.');
end

json_frame = struct();
json_frame.timestamp = local_scalar(sample, 'timestamp');
json_frame.imu = struct( ...
    'gyro', local_row(sample.imu, 'gyro', 3), ...
    'accel_body', local_row(sample.imu, 'accel_body', 3));
json_frame.position = local_row(sample, 'position', 3);
json_frame.velocity = local_row(sample, 'velocity', 3);
json_frame.attitude = local_row(sample, 'attitude', 3);

if isfield(sample, 'quaternion') ...
        && ~isempty(sample.quaternion) ...
        && isfield(cfg, 'json_send_quaternion') ...
        && logical(cfg.json_send_quaternion)
    json_frame.quaternion = local_row(sample, 'quaternion', 4);
end
end

function value = local_scalar(data, field_name)
if ~isfield(data, field_name)
    error('uav:ardupilot:build_official_json_frame:MissingField', ...
        'Ожидалось поле %s.', field_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, field_name);
value = double(value);
end

function row_value = local_row(data, field_name, expected_len)
if ~isfield(data, field_name)
    error('uav:ardupilot:build_official_json_frame:MissingField', ...
        'Ожидалось поле %s.', field_name);
end

row_value = data.(field_name);
validateattributes(row_value, {'numeric'}, {'real', 'finite', 'numel', expected_len}, ...
    mfilename, field_name);
row_value = reshape(double(row_value), 1, expected_len);
end
