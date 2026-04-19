function [transport, rx_out, diag] = json_udp_step(transport, json_text, cfg)
%JSON_UDP_STEP Выполнить один неблокирующий шаг обмена по UDP.
% Назначение:
%   Принимает все доступные входные UDP-датаграммы от `ArduPilot SITL`,
%   разбирает их, выбирает последний валидный двоичный пакет и после этого
%   выполняет передачу строки `JSON`. Если отправитель уже известен по
%   принятому пакету, ответная строка направляется именно на его адрес и
%   порт.
%
% Входы:
%   transport - структура транспортного уровня
%   json_text - строка `JSON` для передачи
%   cfg       - конфигурация средства сопряжения
%
% Выходы:
%   transport - обновленная структура транспортного уровня
%   rx_out    - результат разбора последнего валидного входного пакета
%   diag      - структура диагностических признаков шага обмена
%
% Единицы измерения:
%   объем принятого пакета задается в байтах
%
% Допущения:
%   Функция работает только с уже открытым средством обмена и не должна
%   зависать при отсутствии входящих датаграмм.

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

[transport, rx_out, diag] = local_receive_and_decode(transport, rx_out, diag, cfg);

if strlength(json_text) > 0
    [transport, diag] = local_send_json_text(transport, json_text, diag, cfg);
else
    diag.tx_kind = "none";
    diag.tx_message = "Строка JSON не была передана.";
end

diag.handshake_confirmed = logical(diag.tx_ok && diag.tx_kind == "reply");
diag.response_tx_count = double(diag.handshake_confirmed);

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
diag.rx_datagram_count = 0;
diag.rx_valid_count = 0;
diag.rx_invalid_count = 0;
diag.tx_attempted = false;
diag.tx_ok = false;
diag.tx_count = 0;
diag.response_tx_count = 0;
diag.tx_kind = "none";
diag.tx_message = "";
diag.handshake_confirmed = false;
diag.used_remote_ip = string(cfg.udp_remote_ip);
diag.used_remote_port = double(cfg.udp_remote_port);
diag.last_sender_address = "";
diag.last_sender_port = 0;
diag.last_magic = 0;
diag.last_frame_count = 0;

if isfield(transport, 'remote_ip')
    diag.used_remote_ip = string(transport.remote_ip);
end

if isfield(transport, 'remote_port')
    diag.used_remote_port = double(transport.remote_port);
end
end

function [transport, rx_out, diag] = local_receive_and_decode(transport, rx_out, diag, cfg)
%LOCAL_RECEIVE_AND_DECODE Принять все доступные датаграммы и выбрать последнюю валидную.

last_invalid = uav.ardupilot.decode_sitl_output_packet([], cfg);
last_invalid_bytes = 0;

try
    switch string(transport.method)
        case "udpport"
            while double(transport.handle.NumDatagramsAvailable) > 0
                datagram = read(transport.handle, 1, "uint8");
                if isempty(datagram)
                    break;
                end

                raw_bytes = uint8(datagram.Data(:));
                sender_address = string(datagram.SenderAddress);
                sender_port = double(datagram.SenderPort);

                transport.remote_ip = sender_address;
                transport.remote_port = sender_port;
                transport.rx_count = transport.rx_count + 1;

                diag.rx_datagram_count = diag.rx_datagram_count + 1;
                diag.last_sender_address = sender_address;
                diag.last_sender_port = sender_port;
                diag.used_remote_ip = sender_address;
                diag.used_remote_port = sender_port;

                decoded = uav.ardupilot.decode_sitl_output_packet(raw_bytes, cfg);
                diag.last_magic = double(decoded.magic);
                diag.last_frame_count = double(decoded.frame_count);

                if decoded.valid
                    rx_out = decoded;
                    diag.rx_received = true;
                    diag.rx_valid = true;
                    diag.rx_valid_count = diag.rx_valid_count + 1;
                    diag.rx_bytes_count = numel(raw_bytes);
                    diag.rx_message = string(decoded.message);
                    transport.has_valid_remote = true;
                else
                    last_invalid = decoded;
                    last_invalid_bytes = numel(raw_bytes);
                    diag.rx_invalid_count = diag.rx_invalid_count + 1;
                end
            end
        case "udp"
            bytes_available = double(get(transport.handle, 'BytesAvailable'));
            if bytes_available > 0
                bytes_to_read = min(bytes_available, double(cfg.udp_max_rx_bytes));
                raw_bytes = fread(transport.handle, bytes_to_read, 'uint8');
                raw_bytes = uint8(raw_bytes(:));

                transport.rx_count = transport.rx_count + 1;
                diag.rx_datagram_count = 1;

                decoded = uav.ardupilot.decode_sitl_output_packet(raw_bytes, cfg);
                diag.last_magic = double(decoded.magic);
                diag.last_frame_count = double(decoded.frame_count);

                if decoded.valid
                    rx_out = decoded;
                    diag.rx_received = true;
                    diag.rx_valid = true;
                    diag.rx_valid_count = 1;
                    diag.rx_bytes_count = numel(raw_bytes);
                    diag.rx_message = string(decoded.message);
                    transport.has_valid_remote = true;
                else
                    last_invalid = decoded;
                    last_invalid_bytes = numel(raw_bytes);
                    diag.rx_invalid_count = 1;
                end
            end
        otherwise
            diag.rx_message = "Метод UDP не поддерживается.";
    end
catch rx_error
    diag.rx_message = "Ошибка приема UDP: " + string(rx_error.message);
end

if ~diag.rx_valid && diag.rx_invalid_count > 0
    rx_out = last_invalid;
    diag.rx_received = true;
    diag.rx_bytes_count = last_invalid_bytes;
    diag.rx_message = string(last_invalid.message);
elseif ~diag.rx_received
    diag.rx_message = "Входящий двоичный пакет от ArduPilot не получен.";
end
end

function [transport, diag] = local_send_json_text(transport, json_text, diag, cfg)
%LOCAL_SEND_JSON_TEXT Отправить строку JSON удаленному узлу.

diag.tx_attempted = true;
diag.tx_kind = "probe";

target_ip = string(cfg.udp_remote_ip);
target_port = double(cfg.udp_remote_port);

has_valid_remote = false;
if isfield(transport, 'has_valid_remote')
    has_valid_remote = logical(transport.has_valid_remote);
end

if strlength(string(transport.remote_ip)) > 0 && double(transport.remote_port) > 0
    target_ip = string(transport.remote_ip);
    target_port = double(transport.remote_port);
end

if diag.rx_valid || has_valid_remote
    diag.tx_kind = "reply";
end

warn_state = warning;
cleanup_obj = onCleanup(@() warning(warn_state));
warning('off', 'all');

payload_text = char(json_text);
payload_bytes = uint8(payload_text);

try
    switch string(transport.method)
        case "udpport"
            write( ...
                transport.handle, ...
                payload_bytes, ...
                "uint8", ...
                char(target_ip), ...
                target_port);
        case "udp"
            fwrite(transport.handle, payload_bytes, 'uint8');
        otherwise
            error( ...
                'uav:ardupilot:json_udp_step:UnsupportedMethod', ...
                'Неподдерживаемый метод UDP: %s.', ...
                transport.method);
    end

    transport.tx_count = transport.tx_count + 1;
    diag.tx_ok = true;
    diag.tx_count = 1;
    diag.used_remote_ip = target_ip;
    diag.used_remote_port = target_port;

    if diag.tx_kind == "reply"
        diag.tx_message = append( ...
            "Ответная передача строки JSON выполнена после принятого двоичного пакета ArduPilot на адрес ", ...
            target_ip, ...
            ":", ...
            string(target_port), ...
            ".");
    else
        diag.tx_message = append( ...
            "Исходящая пробная строка JSON была сформирована и отправлена без принятого пакета ArduPilot на адрес ", ...
            target_ip, ...
            ":", ...
            string(target_port), ...
            ".");
    end
catch tx_error
    diag.tx_ok = false;

    if diag.tx_kind == "reply"
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
