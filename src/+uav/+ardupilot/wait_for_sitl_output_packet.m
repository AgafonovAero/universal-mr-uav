function result = wait_for_sitl_output_packet(cfg)
%WAIT_FOR_SITL_OUTPUT_PACKET Дождаться первого двоичного пакета ArduPilot.
% Назначение:
%   Открывает средство обмена по `UDP`, ожидает входящий двоичный пакет
%   от `ArduPilot SITL` в течение ограниченного времени, разбирает его
%   через `decode_sitl_output_packet` и возвращает результат без зависания.
%   На этом шаге не выполняется отправка строки `JSON`.
%
% Входы:
%   cfg - optional config of the exchange layer
%
% Выходы:
%   result - struct with receive status, timeout, elapsed time and packet
%
% Единицы измерения:
%   timeout_s и elapsed_s задаются в секундах, raw_length - в байтах
%
% Допущения:
%   Отсутствие пакета от `ArduPilot` в течение интервала ожидания не
%   считается аварией и должно возвращать понятный статус.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_validate_cfg(cfg);
availability = uav.ardupilot.json_udp_is_available();

result = struct();
result.received = false;
result.timeout_s = double(cfg.udp_handshake_timeout_s);
result.elapsed_s = 0.0;
result.sitl_output = uav.ardupilot.decode_sitl_output_packet([], cfg);
result.raw_length = 0;
result.message = "";

if ~availability.is_available
    result.message = ...
        "Средство UDP недоступно в текущей среде MATLAB.";
    return;
end

transport = uav.ardupilot.json_udp_open(cfg);
if ~transport.is_open
    result.message = string(transport.message);
    return;
end

start_tic = tic;
raw_bytes = uint8([]);

while toc(start_tic) < double(cfg.udp_handshake_timeout_s)
    raw_bytes = local_receive_bytes(transport, cfg);
    if ~isempty(raw_bytes)
        break;
    end

    pause(double(cfg.udp_receive_pause_s));
end

result.elapsed_s = toc(start_tic);
result.raw_length = numel(raw_bytes);

if ~isempty(raw_bytes)
    result.sitl_output = uav.ardupilot.decode_sitl_output_packet(raw_bytes, cfg);
    result.received = logical(result.sitl_output.valid);

    if result.sitl_output.valid
        result.message = ...
            "Входящий двоичный пакет ArduPilot получен и разобран.";
    else
        result.message = ...
            "Входящий пакет получен, но не соответствует ожидаемому формату: " + ...
            string(result.sitl_output.message);
    end
else
    result.message = ...
        "Входящий двоичный пакет ArduPilot не получен до истечения тайм-аута.";
end

transport = uav.ardupilot.json_udp_close(transport); %#ok<NASGU>
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить конфигурацию ожидания пакета.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:wait_for_sitl_output_packet:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

required_fields = { ...
    'udp_handshake_timeout_s', ...
    'udp_receive_pause_s', ...
    'udp_max_rx_bytes'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name)
        error( ...
            'uav:ardupilot:wait_for_sitl_output_packet:MissingCfgField', ...
            'Ожидалось наличие поля cfg.%s.', ...
            field_name);
    end
end

validateattributes(cfg.udp_handshake_timeout_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'cfg.udp_handshake_timeout_s');
validateattributes(cfg.udp_receive_pause_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'cfg.udp_receive_pause_s');
end

function raw_bytes = local_receive_bytes(transport, cfg)
%LOCAL_RECEIVE_BYTES Принять один пакет байтов, если он доступен.

raw_bytes = uint8([]);

try
    switch string(transport.method)
        case "udpport"
            datagrams_available = double(transport.handle.NumDatagramsAvailable);
            if datagrams_available > 0
                datagram = read(transport.handle, 1, "uint8");
                raw_bytes = uint8(datagram.Data(:));
            end
        case "udp"
            bytes_available = double(get(transport.handle, 'BytesAvailable'));
            if bytes_available > 0
                bytes_to_read = min(bytes_available, double(cfg.udp_max_rx_bytes));
                raw_bytes = fread(transport.handle, bytes_to_read, 'uint8');
                raw_bytes = uint8(raw_bytes(:));
            end
        otherwise
            raw_bytes = uint8([]);
    end
catch
    raw_bytes = uint8([]);
end
end
