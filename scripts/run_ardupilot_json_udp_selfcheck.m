%% RUN_ARDUPILOT_JSON_UDP_SELFCHECK Выполнить самопроверку средства обмена.
% Назначение:
%   Проверяет кодирование пакета данных в `JSON`, разбор искусственно
%   сформированного двоичного пакета `ArduPilot`, преобразование ШИМ
%   в команды частоты вращения винтов и локальный обмен по `UDP`,
%   если средство `UDP` доступно в MATLAB.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_json_udp_selfcheck - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   длительность импульсов ШИМ - микросекунды, команды частоты вращения -
%   радианы в секунду
%
% Допущения:
%   Самопроверка не требует установленного `ArduPilot` и не подтверждает
%   штатный полет под управлением внешнего комплекса.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
availability = uav.ardupilot.json_udp_is_available();

state = uav.core.state_unpack(params.demo.initial_state_plant);
snapshot = local_snapshot_diag(state, params);
sensors = uav.sensors.sensors_step(state, snapshot, params);
estimator = uav.est.estimator_init(params, sensors);

packet = uav.ardupilot.pack_json_fdm_packet( ...
    state, ...
    sensors, ...
    estimator, ...
    0.0, ...
    params, ...
    cfg);

json_text = uav.ardupilot.encode_json_fdm_text(packet);
json_data = jsondecode(char(strip(json_text)));

synthetic_packet_16 = local_make_sitl_packet_16(cfg);
decoded_output = uav.ardupilot.decode_sitl_output_packet( ...
    synthetic_packet_16, ...
    cfg);
motor_cmd_radps = uav.ardupilot.pwm_to_motor_radps( ...
    decoded_output.motor_pwm_us, ...
    params, ...
    cfg);

[udp_exchange_ok, udp_exchange_message, udp_exchange_json, udp_exchange_rx] = ...
    local_udp_selfcheck_exchange(availability, cfg, json_text);

result = struct();
result.availability = availability;
result.packet = packet;
result.json_text = json_text;
result.json_has_timestamp = isfield(json_data, 'timestamp');
result.json_has_quaternion = isfield(json_data, 'quaternion');
result.decoded_output = decoded_output;
result.motor_cmd_radps = motor_cmd_radps;
result.udp_exchange_ok = logical(udp_exchange_ok);
result.udp_exchange_message = string(udp_exchange_message);
result.udp_exchange_json = string(udp_exchange_json);
result.udp_exchange_rx = udp_exchange_rx;

assignin('base', 'ardupilot_json_udp_selfcheck', result);

fprintf('Самопроверка средства обмена ArduPilot по JSON и UDP\n');
fprintf('  доступность UDP                        : %s\n', ...
    local_bool_text(availability.is_available));
fprintf('  используемый механизм                  : %s\n', ...
    char(string(availability.method)));
fprintf('  кодирование JSON выполнено             : %s\n', ...
    local_bool_text(result.json_has_timestamp && result.json_has_quaternion));
fprintf('  разбор двоичного пакета выполнен       : %s\n', ...
    local_bool_text(decoded_output.valid));
fprintf('  magic                                  : %u\n', decoded_output.magic);
fprintf('  frame_rate_hz                          : %.0f\n', ...
    decoded_output.frame_rate_hz);
fprintf('  frame_count                            : %.0f\n', ...
    decoded_output.frame_count);
fprintf('  первые четыре значения ШИМ [us]        : [%s]\n', ...
    local_format_vector(decoded_output.motor_pwm_us));
fprintf('  команды частоты вращения [rad/s]       : [%s]\n', ...
    local_format_vector(motor_cmd_radps));
fprintf('  локальный обмен UDP выполнен           : %s\n', ...
    local_bool_text(result.udp_exchange_ok));
fprintf('  сообщение транспортной проверки        : %s\n', ...
    char(result.udp_exchange_message));

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Построить детерминированный диагностический снимок.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function raw_bytes = local_make_sitl_packet_16(cfg)
%LOCAL_MAKE_SITL_PACKET_16 Построить искусственный пакет с 16 каналами.

pwm_us = uint16([1615 1615 1615 1615 zeros(1, 12)]);
raw_bytes = [ ...
    typecast(uint16(cfg.sitl_magic_16), 'uint8'), ...
    typecast(uint16(cfg.update_rate_hz), 'uint8'), ...
    typecast(uint32(7), 'uint8'), ...
    typecast(pwm_us, 'uint8')];
raw_bytes = uint8(raw_bytes(:));
end

function [is_ok, message, rx_json_text, rx_output] = local_udp_selfcheck_exchange( ...
        availability, cfg, json_text)
%LOCAL_UDP_SELFCHECK_EXCHANGE Выполнить локальный обмен по UDP.

is_ok = false;
message = "";
rx_json_text = "";
rx_output = struct();
rx_output.probe_received = false;
rx_output.probe_bytes = uint8([]);
rx_output.message = "";

if ~availability.is_available
    message = "Средство UDP недоступно; локальный обмен не выполнялся.";
    return;
end

if availability.method ~= "udpport"
    message = "Локальный обмен реализован только для механизма udpport.";
    return;
end

self_cfg = cfg;
[self_cfg.udp_local_port, self_cfg.udp_remote_port] = local_pick_selfcheck_ports();

aux_port = [];
model_port = [];
probe_bytes = uint8([1 3 5 7 9 11]);
timeout_s = max(double(self_cfg.udp_timeout_s), 0.2);

try
    aux_port = udpport( ...
        "byte", ...
        "IPV4", ...
        "LocalPort", double(self_cfg.udp_remote_port), ...
        "Timeout", timeout_s);
    model_port = udpport( ...
        "byte", ...
        "IPV4", ...
        "LocalPort", double(self_cfg.udp_local_port), ...
        "Timeout", timeout_s);

    pause(0.05);

    local_write_quietly( ...
        aux_port, ...
        reshape(probe_bytes, 1, []), ...
        self_cfg.udp_local_ip, ...
        self_cfg.udp_local_port);

    rx_probe = local_wait_for_udp_bytes( ...
        model_port, ...
        max(double(self_cfg.handshake_wait_timeout_s), 0.3));
    rx_output.probe_received = isequal(rx_probe(:), probe_bytes(:));
    rx_output.probe_bytes = rx_probe(:);
    rx_output.message = "Локальная проверочная датаграмма обработана.";

    if rx_output.probe_received
        local_write_quietly( ...
            model_port, ...
            reshape(uint8(char(json_text)), 1, []), ...
            self_cfg.udp_remote_ip, ...
            self_cfg.udp_remote_port);
        rx_json_bytes = local_wait_for_udp_bytes( ...
            aux_port, ...
            max(double(self_cfg.handshake_wait_timeout_s), 0.3));
        rx_json_text = string(char(rx_json_bytes(:).'));
    end

    is_ok = rx_output.probe_received && contains(rx_json_text, '"timestamp"');
    if is_ok
        message = "Локальный обмен по UDP выполнен успешно.";
    elseif rx_output.probe_received
        message = "Проверочная датаграмма принята, но строка JSON не получена на локальном приемнике.";
    else
        message = "Локальный прием проверочной датаграммы не подтвержден.";
    end
catch exchange_error
    message = "Ошибка локального обмена UDP: " + string(exchange_error.message);
end

if ~isempty(aux_port)
    clear aux_port;
end

if ~isempty(model_port)
    clear model_port;
end

pause(0.05);
end

function raw_bytes = local_wait_for_udp_bytes(port_handle, timeout_s)
%LOCAL_WAIT_FOR_UDP_BYTES Дождаться байтов на локальном приемнике UDP.

raw_bytes = uint8([]);
tic;

while toc < timeout_s
    bytes_available = double(port_handle.NumBytesAvailable);
    if bytes_available > 0
        raw_bytes = read(port_handle, bytes_available, "uint8");
        raw_bytes = uint8(raw_bytes(:));
        return;
    end

    pause(0.01);
end
end

function [local_port, remote_port] = local_pick_selfcheck_ports()
%LOCAL_PICK_SELFCHECK_PORTS Выбрать пару локальных портов для самопроверки.

timestamp_ms = posixtime(datetime("now")) * 1000.0;
base_port = 20000 + mod(floor(timestamp_ms), 20000);

local_port = double(base_port);
remote_port = double(base_port + 1);
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку числового вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end

function local_write_quietly(port_handle, bytes_to_send, remote_ip, remote_port)
%LOCAL_WRITE_QUIETLY Выполнить запись UDP без служебных предупреждений.

warn_state = warning;
cleanup_obj = onCleanup(@() warning(warn_state));
warning('off', 'all');

write( ...
    port_handle, ...
    reshape(uint8(bytes_to_send), 1, []), ...
    "uint8", ...
    char(remote_ip), ...
    double(remote_port));

clear cleanup_obj;
end
