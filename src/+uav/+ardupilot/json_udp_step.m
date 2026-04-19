function [transport, rx_out, diag] = json_udp_step(transport, json_text, cfg)
%JSON_UDP_STEP Выполнить один шаг приема и передачи по UDP.
% Назначение:
%   Выполняет один неблокирующий шаг обмена по `UDP`. Сначала функция
%   пытается принять двоичный пакет от `ArduPilot SITL`, затем при
%   наличии строки `JSON` выполняет передачу. Диагностика шага явно
%   различает три состояния:
%   1. прием не подтвержден;
%   2. исходящая пробная передача без принятого пакета;
%   3. ответная передача после принятого и разобранного пакета.
%
% Входы:
%   transport - структура транспортного уровня
%   json_text - строка JSON для передачи
%   cfg       - конфигурация средства сопряжения
%
% Выходы:
%   transport - обновленная структура транспортного уровня
%   rx_out    - результат разбора входного двоичного пакета
%   diag      - структура диагностических признаков шага обмена
%
% Единицы измерения:
%   объем принятого пакета задается в байтах
%
% Допущения:
%   Функция не должна зависать и использует только уже открытое средство
%   обмена. При отсутствии пакета от `ArduPilot` допускается только
%   исходящая пробная передача строки `JSON`.

if nargin < 3 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_validate_cfg(cfg);
transport = local_validate_transport(transport);
rx_out = uav.ardupilot.decode_sitl_output_packet([], cfg);
diag = local_make_empty_diag(cfg, transport);

if ~isstring(json_text) && ~ischar(json_text)
    error( ...
        'uav:ardupilot:json_udp_step:JsonType', ...
        'Ожидалась строка JSON или символьный вектор.');
end

json_text = string(json_text);

if ~transport.is_open || isempty(transport.handle)
    diag.status = "прием не подтвержден";
    diag.status_message = "Средство обмена не открыто.";
    diag.rx_message = "Входящий двоичный пакет не получен.";
    diag.tx_message = string(transport.message);
    return;
end

[raw_bytes, transport, diag] = local_receive_bytes(transport, diag, cfg);

if ~isempty(raw_bytes)
    rx_out = uav.ardupilot.decode_sitl_output_packet(raw_bytes, cfg);
    diag.rx_received = true;
    diag.rx_bytes_count = numel(raw_bytes);
    diag.rx_valid = logical(rx_out.valid);
    diag.rx_message = string(rx_out.message);
else
    diag.rx_message = "Входящий двоичный пакет от ArduPilot не получен.";
end

if strlength(json_text) > 0
    [transport, diag] = local_send_json_text( ...
        transport, ...
        json_text, ...
        diag, ...
        cfg);
else
    diag.tx_kind = "none";
    diag.tx_message = "Строка JSON не была передана.";
end

diag.handshake_confirmed = logical(diag.rx_valid && diag.tx_ok);

if diag.handshake_confirmed
    diag.status = "ответная передача";
    diag.status_message = [ ...
        "Входящий двоичный пакет ArduPilot принят и разобран. " ...
        + "После этого выполнена ответная передача строки JSON."];
elseif diag.tx_ok
    diag.status = "исходящая пробная передача";
    diag.status_message = [ ...
        "Прием не подтвержден. Выполнена только исходящая пробная " ...
        + "передача строки JSON без принятого пакета ArduPilot."];
else
    diag.status = "прием не подтвержден";
    if diag.rx_received && ~diag.rx_valid
        diag.status_message = [ ...
            "Входящий двоичный пакет был получен, но не разобран. " ...
            + "Ответная передача не подтверждена."];
    else
        diag.status_message = ...
            "Входящий двоичный пакет ArduPilot не получен.";
    end
end
end

function transport = local_validate_transport(transport)
%LOCAL_VALIDATE_TRANSPORT Проверить структуру транспортного уровня.

if ~isstruct(transport) || ~isscalar(transport)
    error( ...
        'uav:ardupilot:json_udp_step:TransportType', ...
        'Ожидалась скалярная структура transport.');
end

required_fields = { ...
    'is_open', ...
    'method', ...
    'message', ...
    'remote_ip', ...
    'remote_port', ...
    'rx_count', ...
    'tx_count', ...
    'handle'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(transport, field_name)
        error( ...
            'uav:ardupilot:json_udp_step:MissingTransportField', ...
            'Ожидалось наличие поля transport.%s.', ...
            field_name);
    end
end
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить конфигурацию шага обмена.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:json_udp_step:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

required_fields = { ...
    'udp_remote_ip', ...
    'udp_remote_port', ...
    'udp_max_rx_bytes'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name)
        error( ...
            'uav:ardupilot:json_udp_step:MissingCfgField', ...
            'Ожидалось наличие поля cfg.%s.', ...
            field_name);
    end
end
end

function diag = local_make_empty_diag(cfg, transport)
%LOCAL_MAKE_EMPTY_DIAG Построить пустую диагностическую структуру.

diag = struct();
diag.status = "прием не подтвержден";
diag.status_message = "Обмен не выполнялся.";
diag.rx_received = false;
diag.rx_valid = false;
diag.rx_bytes_count = 0;
diag.rx_message = "";
diag.tx_attempted = false;
diag.tx_ok = false;
diag.tx_kind = "none";
diag.tx_message = "";
diag.handshake_confirmed = false;
diag.used_remote_ip = string(cfg.udp_remote_ip);
diag.used_remote_port = double(cfg.udp_remote_port);

if isfield(transport, 'remote_ip')
    diag.used_remote_ip = string(transport.remote_ip);
end

if isfield(transport, 'remote_port')
    diag.used_remote_port = double(transport.remote_port);
end
end

function [raw_bytes, transport, diag] = local_receive_bytes(transport, diag, cfg)
%LOCAL_RECEIVE_BYTES Принять один пакет байтов, если он доступен.

raw_bytes = uint8([]);

try
    switch string(transport.method)
        case "udpport"
            bytes_available = double(transport.handle.NumBytesAvailable);
            if bytes_available > 0
                bytes_to_read = min(bytes_available, double(cfg.udp_max_rx_bytes));
                raw_bytes = read(transport.handle, bytes_to_read, "uint8");
                raw_bytes = uint8(raw_bytes(:));
                transport.rx_count = transport.rx_count + 1;
            end
        case "udp"
            bytes_available = double(get(transport.handle, 'BytesAvailable'));
            if bytes_available > 0
                bytes_to_read = min(bytes_available, double(cfg.udp_max_rx_bytes));
                raw_bytes = fread(transport.handle, bytes_to_read, 'uint8');
                raw_bytes = uint8(raw_bytes(:));
                transport.rx_count = transport.rx_count + 1;
            end
        otherwise
            diag.rx_message = "Метод UDP не поддерживается.";
    end
catch rx_error
    raw_bytes = uint8([]);
    diag.rx_message = "Ошибка приема UDP: " + string(rx_error.message);
end
end

function [transport, diag] = local_send_json_text(transport, json_text, diag, cfg)
%LOCAL_SEND_JSON_TEXT Отправить строку JSON удаленному узлу.

diag.tx_attempted = true;
diag.tx_kind = "probe";

if diag.rx_valid
    diag.tx_kind = "reply";
end

warn_state = warning;
cleanup_obj = onCleanup(@() warning(warn_state));
warning('off', 'all');

bytes_to_send = reshape(uint8(char(json_text)), 1, []);

try
    switch string(transport.method)
        case "udpport"
            write( ...
                transport.handle, ...
                bytes_to_send, ...
                "uint8", ...
                char(cfg.udp_remote_ip), ...
                double(cfg.udp_remote_port));
        case "udp"
            fwrite(transport.handle, bytes_to_send, 'uint8');
        otherwise
            error( ...
                'uav:ardupilot:json_udp_step:UnsupportedMethod', ...
                'Неподдерживаемый метод UDP: %s.', ...
                transport.method);
    end

    transport.tx_count = transport.tx_count + 1;
    diag.tx_ok = true;

    if diag.rx_valid
        diag.tx_message = [ ...
            "Ответная передача строки JSON выполнена после " ...
            + "принятого двоичного пакета ArduPilot."];
    else
        diag.tx_message = [ ...
            "Исходящая пробная строка JSON была сформирована " ...
            + "и отправлена без принятого пакета ArduPilot."];
    end
catch tx_error
    diag.tx_ok = false;

    if diag.rx_valid
        diag.tx_message = [ ...
            "Не удалось выполнить ответную передачу строки JSON " ...
            + "после принятого пакета ArduPilot: " ...
            + string(tx_error.message)];
    else
        diag.tx_message = [ ...
            "Не удалось выполнить исходящую пробную передачу " ...
            + "строки JSON: " + string(tx_error.message)];
    end
end

clear cleanup_obj;
end
