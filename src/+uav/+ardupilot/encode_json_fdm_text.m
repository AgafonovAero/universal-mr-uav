function json_text = encode_json_fdm_text(packet)
%ENCODE_JSON_FDM_TEXT Закодировать пакет данных в строку JSON.
% Назначение:
%   Преобразует канонический пакет данных TASK-10/TASK-11 в строку
%   `JSON`, пригодную для передачи во внешний комплекс `ArduPilot`.
%   Функция проверяет наличие обязательных полей и завершает результирующую
%   строку символом перевода строки.
%
% Входы:
%   packet - скалярная структура пакета данных
%
% Выходы:
%   json_text - строка JSON с завершающим символом перевода строки
%
% Единицы измерения:
%   используются единицы СИ; векторы в земной системе координат задаются
%   в `NED`, векторы связанной системы координат - в `FRD`
%
% Допущения:
%   В качестве ориентации передается кватернион `q_nb`
%   в скалярно-первой форме.

packet = uav.ardupilot.validate_json_packet(packet);
local_require_fields(packet, { ...
    'time_s', ...
    'position_ned_m', ...
    'velocity_ned_mps', ...
    'q_nb', ...
    'imu'});
local_require_fields(packet.imu, {'gyro_b_radps', 'accel_b_mps2'});

json_packet = struct();
json_packet.timestamp = double(packet.time_s);
json_packet.imu = struct( ...
    'gyro', local_row(packet.imu.gyro_b_radps), ...
    'accel_body', local_row(packet.imu.accel_b_mps2));
json_packet.position = local_row(packet.position_ned_m);
json_packet.velocity = local_row(packet.velocity_ned_mps);
json_packet.quaternion = local_row(packet.q_nb);
json_packet.body_rates = local_row(packet.w_b_radps);

if isfield(packet, 'baro') && isstruct(packet.baro) && ...
        isfield(packet.baro, 'alt_m')
    json_packet.baro = struct( ...
        'altitude_m', double(packet.baro.alt_m));
end

if isfield(packet, 'mag') && isstruct(packet.mag) && ...
        isfield(packet.mag, 'field_b_uT')
    json_packet.mag = struct( ...
        'body_uT', local_row(packet.mag.field_b_uT));
end

if isfield(packet, 'gnss') && isstruct(packet.gnss) && ...
        isfield(packet.gnss, 'pos_ned_m') && ...
        isfield(packet.gnss, 'vel_ned_mps')
    json_packet.gnss = struct( ...
        'position_ned_m', local_row(packet.gnss.pos_ned_m), ...
        'velocity_ned_mps', local_row(packet.gnss.vel_ned_mps));
end

json_text = string(jsonencode(json_packet)) + newline;
end

function local_require_fields(data, field_names)
%LOCAL_REQUIRE_FIELDS Проверить наличие обязательных полей.

if ~isstruct(data) || ~isscalar(data)
    error('uav:ardupilot:encode_json_fdm_text:StructType', ...
        'Ожидалась скалярная структура данных.');
end

for k = 1:numel(field_names)
    field_name = field_names{k};
    if ~isfield(data, field_name)
        error('uav:ardupilot:encode_json_fdm_text:MissingField', ...
            'Ожидалось наличие поля %s.', field_name);
    end
end
end

function row_value = local_row(vec_value)
%LOCAL_ROW Преобразовать вектор в строку JSON-массива.

validateattributes(vec_value, {'numeric'}, ...
    {'real', 'finite', 'vector'}, mfilename, 'vec_value');
row_value = reshape(double(vec_value), 1, []);
end
