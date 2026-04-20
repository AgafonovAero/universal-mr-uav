function result = read_pymavlink_sequence_result(json_path)
%READ_PYMAVLINK_SEQUENCE_RESULT Прочитать результат сценария pymavlink из JSON.
% Назначение:
%   Загружает файл `JSON`, сформированный временным Python-сценарием
%   `pymavlink`, и приводит его к удобной для MATLAB структуре с
%   таблицей телеметрии и строковыми массивами сообщений.
%
% Входы:
%   json_path - путь к файлу результата `JSON`
%
% Выходы:
%   result - структура результата сценария
%
% Единицы измерения:
%   высота задается в метрах;
%   время задается в секундах
%
% Допущения:
%   Файл JSON был сформирован функцией
%   `uav.ardupilot.make_pymavlink_sequence_command`.

json_path = string(json_path);
if ~isfile(char(json_path))
    error( ...
        'uav:ardupilot:read_pymavlink_sequence_result:MissingFile', ...
        'Файл результата не найден: %s.', ...
        json_path);
end

parsed = jsondecode(fileread(char(json_path)));
result = parsed;

if isfield(parsed, 'status_texts') && ~isempty(parsed.status_texts)
    result.status_texts = string(parsed.status_texts(:));
else
    result.status_texts = strings(0, 1);
end

if isfield(parsed, 'connection_attempts') && ~isempty(parsed.connection_attempts)
    result.connection_attempts = parsed.connection_attempts;
else
    result.connection_attempts = struct('connection_string', {}, 'heartbeat_received', {}, 'error', {});
end

if isfield(parsed, 'command_acks') && ~isempty(parsed.command_acks)
    result.command_acks = parsed.command_acks;
else
    result.command_acks = struct('command', {}, 'result', {});
end

result.telemetry_table = local_make_telemetry_table(parsed);
result.max_relative_alt_m = local_optional_scalar(parsed, 'max_relative_alt_m');
result.arm_ack_result = local_optional_scalar(parsed, 'arm_ack_result');
result.takeoff_ack_result = local_optional_scalar(parsed, 'takeoff_ack_result');
result.elapsed_s = local_optional_scalar(parsed, 'elapsed_s');
end

function telemetry_table = local_make_telemetry_table(parsed)
%LOCAL_MAKE_TELEMETRY_TABLE Построить таблицу телеметрии.

if ~isfield(parsed, 'telemetry') || isempty(parsed.telemetry)
    telemetry_table = table();
    return;
end

telemetry = parsed.telemetry;
row_count = numel(telemetry);

t_s = nan(row_count, 1);
armed = false(row_count, 1);
mode = strings(row_count, 1);
relative_alt_m = nan(row_count, 1);
heartbeat_count = zeros(row_count, 1);
global_position_count = zeros(row_count, 1);
status_text_count = zeros(row_count, 1);

for idx = 1:row_count
    item = telemetry(idx);
    t_s(idx) = local_optional_scalar(item, 't_s');
    armed(idx) = logical(local_optional_scalar(item, 'armed'));
    if isfield(item, 'mode') && ~isempty(item.mode)
        mode(idx) = string(item.mode);
    end
    relative_alt_m(idx) = local_optional_scalar(item, 'relative_alt_m');
    heartbeat_count(idx) = local_optional_scalar(item, 'heartbeat_count');
    global_position_count(idx) = local_optional_scalar(item, 'global_position_count');
    status_text_count(idx) = local_optional_scalar(item, 'status_text_count');
end

telemetry_table = table( ...
    t_s, ...
    armed, ...
    mode, ...
    relative_alt_m, ...
    heartbeat_count, ...
    global_position_count, ...
    status_text_count);
end

function value = local_optional_scalar(data_struct, field_name)
%LOCAL_OPTIONAL_SCALAR Безопасно прочитать числовое поле.

if isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    value = double(data_struct.(field_name));
else
    value = nan;
end
end
