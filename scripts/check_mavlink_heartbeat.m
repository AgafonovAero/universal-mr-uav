%% CHECK_MAVLINK_HEARTBEAT Проверить наличие MAVLink HEARTBEAT на дополнительном порту.
% Назначение:
%   Прослушивает дополнительный UDP-порт потока MAVLink, не вмешиваясь в
%   основной поток Mission Planner, и пытается обнаружить сообщение
%   HEARTBEAT протокола MAVLink v1 или v2.
%
% Входы:
%   none
%
% Выходы:
%   mavlink_heartbeat_check - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время ожидания задается в секундах
%
% Допущения:
%   ArduPilot SITL уже запущен и направляет дополнительный поток MAVLink
%   на cfg.mavlink_monitor_udp_port.

cfg = uav.ardupilot.default_json_config();
if evalin('base', 'exist(''ardupilot_mavlink_monitor_udp_port'', ''var'')')
    cfg.mavlink_monitor_udp_port = evalin('base', 'ardupilot_mavlink_monitor_udp_port');
end
availability = uav.ardupilot.json_udp_is_available();

result = struct();
result.heartbeat_detected = false;
result.protocol_version = "";
result.system_id = 0;
result.component_id = 0;
result.message = "";
result.elapsed_s = 0.0;
result.sender_address = "";
result.sender_port = 0;
result.port = cfg.mavlink_monitor_udp_port;

if ~availability.is_available || ~strcmp(string(availability.method), "udpport")
    result.message = "Средство UDP для прослушивания MAVLink недоступно.";
    assignin('base', 'mavlink_heartbeat_check', result);
    fprintf('Проверка MAVLink HEARTBEAT\n');
    fprintf('  HEARTBEAT обнаружен                 : нет\n');
    fprintf('  пояснение                           : %s\n', char(result.message));
    return;
end

listener = [];
try
    listener = udpport("byte", "IPV4", "LocalPort", double(cfg.mavlink_monitor_udp_port));
catch open_error
    result.message = "Не удалось открыть порт MAVLink: " + string(open_error.message);
    assignin('base', 'mavlink_heartbeat_check', result);
    fprintf('Проверка MAVLink HEARTBEAT\n');
    fprintf('  HEARTBEAT обнаружен                 : нет\n');
    fprintf('  пояснение                           : %s\n', char(result.message));
    return;
end

cleanup_obj = onCleanup(@() clear('listener')); %#ok<NASGU>

timeout_s = 8.0;
if evalin('base', 'exist(''ardupilot_mavlink_heartbeat_timeout_s'', ''var'')')
    timeout_s = double(evalin('base', 'ardupilot_mavlink_heartbeat_timeout_s'));
end
start_tic = tic;

while toc(start_tic) < timeout_s
    while listener.NumBytesAvailable > 0
        raw_bytes = read(listener, double(listener.NumBytesAvailable), "uint8");
        raw_bytes = uint8(raw_bytes(:));
        [is_detected, heartbeat_info] = local_find_heartbeat(raw_bytes);

        if is_detected
            result.heartbeat_detected = true;
            result.protocol_version = heartbeat_info.protocol_version;
            result.system_id = heartbeat_info.system_id;
            result.component_id = heartbeat_info.component_id;
            result.message = heartbeat_info.message;
            result.sender_address = "";
            result.sender_port = 0;
            result.elapsed_s = toc(start_tic);
            assignin('base', 'mavlink_heartbeat_check', result);

            fprintf('Проверка MAVLink HEARTBEAT\n');
            fprintf('  HEARTBEAT обнаружен                 : да\n');
            fprintf('  версия протокола                    : %s\n', char(result.protocol_version));
            fprintf('  system_id                           : %d\n', result.system_id);
            fprintf('  component_id                        : %d\n', result.component_id);
            fprintf('  sender address                      : %s\n', char(result.sender_address));
            fprintf('  sender port                         : %d\n', result.sender_port);
            fprintf('  время ожидания [s]                  : %.3f\n', result.elapsed_s);
            fprintf('  пояснение                           : %s\n', char(result.message));
            return;
        end
    end

    pause(0.05);
end

result.elapsed_s = toc(start_tic);
result.message = "Сообщение MAVLink HEARTBEAT на дополнительном порту не обнаружено.";

assignin('base', 'mavlink_heartbeat_check', result);

fprintf('Проверка MAVLink HEARTBEAT\n');
fprintf('  HEARTBEAT обнаружен                 : нет\n');
fprintf('  порт прослушивания                  : %d\n', result.port);
fprintf('  время ожидания [s]                  : %.3f\n', result.elapsed_s);
fprintf('  пояснение                           : %s\n', char(result.message));

function [is_detected, info] = local_find_heartbeat(raw_bytes)
%LOCAL_FIND_HEARTBEAT Найти HEARTBEAT внутри одной UDP-датаграммы.

is_detected = false;
info = struct( ...
    'protocol_version', "", ...
    'system_id', 0, ...
    'component_id', 0, ...
    'message', "");

if isempty(raw_bytes)
    return;
end

raw_bytes = uint8(raw_bytes(:));
idx = 1;
while idx <= numel(raw_bytes)
    magic_value = raw_bytes(idx);

    if magic_value == hex2dec('FE')
        if idx + 5 > numel(raw_bytes)
            return;
        end

        payload_len = double(raw_bytes(idx + 1));
        packet_len = 6 + payload_len + 2;
        if idx + packet_len - 1 > numel(raw_bytes)
            return;
        end

        msg_id = double(raw_bytes(idx + 5));
        if msg_id == 0
            info.protocol_version = "MAVLink v1";
            info.system_id = double(raw_bytes(idx + 3));
            info.component_id = double(raw_bytes(idx + 4));
            info.message = "Обнаружен пакет HEARTBEAT протокола MAVLink v1.";
            is_detected = true;
            return;
        end

        idx = idx + packet_len;
        continue;
    end

    if magic_value == hex2dec('FD')
        if idx + 9 > numel(raw_bytes)
            return;
        end

        payload_len = double(raw_bytes(idx + 1));
        incompat_flags = double(raw_bytes(idx + 2));
        packet_len = 10 + payload_len + 2;
        if bitand(incompat_flags, 1) ~= 0
            packet_len = packet_len + 13;
        end

        if idx + packet_len - 1 > numel(raw_bytes)
            return;
        end

        msg_id = double(raw_bytes(idx + 7)) ...
            + 256.0 * double(raw_bytes(idx + 8)) ...
            + 65536.0 * double(raw_bytes(idx + 9));
        if msg_id == 0
            info.protocol_version = "MAVLink v2";
            info.system_id = double(raw_bytes(idx + 5));
            info.component_id = double(raw_bytes(idx + 6));
            info.message = "Обнаружен пакет HEARTBEAT протокола MAVLink v2.";
            is_detected = true;
            return;
        end

        idx = idx + packet_len;
        continue;
    end

    idx = idx + 1;
end
end
