%% RUN_ARDUPILOT_JSON_UDP_HANDSHAKE Выполнить единичную ответную передачу после первого пакета.
% Назначение:
%   Ожидает первый валидный двоичный пакет от уже запущенного `ArduPilot
%   SITL`, затем формирует одну строку `JSON` и выполняет ответную
%   передачу на адрес фактического отправителя.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_json_udp_handshake - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Допущения:
%   Сценарий не подменяет реальный пакет искусственными данными и не
%   подтверждает устойчивый обмен без фактического приема пакета.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
availability = uav.ardupilot.json_udp_is_available();
env_info = uav.ardupilot.inspect_sitl_environment(cfg);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);

transport = uav.ardupilot.json_udp_open(cfg);
transport_message_open = string(transport.message);

state = uav.core.state_unpack(params.demo.initial_state_plant);
snapshot = local_snapshot_diag(state, params);
sensors = uav.sensors.sensors_step(state, snapshot, params);
estimator = uav.est.estimator_init(params, sensors);
json_packet = uav.ardupilot.pack_json_fdm_packet( ...
    state, ...
    sensors, ...
    estimator, ...
    0.0, ...
    params, ...
    cfg);
json_text = uav.ardupilot.encode_json_fdm_text(json_packet);

wait_start = tic;
wait_diag = local_empty_exchange_diag(cfg);
rx_out = uav.ardupilot.decode_sitl_output_packet([], cfg);
packet_received = false;

while toc(wait_start) < cfg.udp_handshake_timeout_s
    [transport, rx_candidate, diag_candidate] = uav.ardupilot.json_udp_step( ...
        transport, ...
        "", ...
        cfg);
    wait_diag = diag_candidate;

    if rx_candidate.valid
        rx_out = rx_candidate;
        packet_received = true;
        break;
    end

    pause(cfg.udp_receive_pause_s);
end

probe_json_sent = false;
reply_json_sent = false;
exchange_status = string(wait_diag.status);
exchange_status_message = string(wait_diag.status_message);
reply_diag = wait_diag;

if packet_received
    [transport, ~, reply_diag] = uav.ardupilot.json_udp_step( ...
        transport, ...
        json_text, ...
        cfg);
    probe_json_sent = logical(reply_diag.tx_ok && reply_diag.tx_kind == "probe");
    reply_json_sent = logical(reply_diag.tx_ok && reply_diag.tx_kind == "reply");
    exchange_status = string(reply_diag.status);
    exchange_status_message = string(reply_diag.status_message);
end

transport = uav.ardupilot.json_udp_close(transport);

result = struct();
result.availability = availability;
result.env_info = env_info;
result.start_command = start_cmd;
result.transport_message_open = transport_message_open;
result.transport_message_close = string(transport.message);
result.packet_received = logical(packet_received);
result.probe_json_sent = logical(probe_json_sent);
result.reply_json_sent = logical(reply_json_sent);
result.rx_out = rx_out;
result.wait_diag = wait_diag;
result.reply_diag = reply_diag;
result.exchange_status = exchange_status;
result.exchange_status_message = exchange_status_message;

assignin('base', 'ardupilot_json_udp_handshake', result);

fprintf('Проверка обмена с ArduPilot по JSON и UDP\n');
fprintf('  готовность UDP                           : %s\n', ...
    local_bool_text(availability.is_available));
fprintf('  готовность локальной среды               : %s\n', ...
    local_bool_text(env_info.is_ready));
fprintf('  пакет от ArduPilot получен               : %s\n', ...
    local_bool_text(packet_received));

if packet_received
    fprintf('  magic                                    : %u\n', ...
        rx_out.magic);
    fprintf('  frame_rate_hz                            : %.0f\n', ...
        rx_out.frame_rate_hz);
    fprintf('  frame_count                              : %.0f\n', ...
        rx_out.frame_count);
    fprintf('  первые четыре значения ШИМ [us]          : [%s]\n', ...
        local_format_vector(rx_out.motor_pwm_us));
    fprintf('  ответная передача JSON выполнена         : %s\n', ...
        local_bool_text(reply_json_sent));
    fprintf('  статус обмена                            : %s\n', ...
        char(exchange_status));
    fprintf('  пояснение                                : %s\n', ...
        char(exchange_status_message));
else
    fprintf('  прием не подтвержден                     : да\n');
    fprintf('  исходящая пробная строка JSON отправлена : нет\n');
    fprintf('  ответная передача JSON выполнена         : нет\n');
    fprintf(['  ответ на принятый двоичный пакет ArduPilot не проверен, ' ...
        'так как входной пакет не был получен.\n']);
    fprintf('  команда запуска для отдельного прогона   : %s\n', ...
        char(start_cmd.command_text));
    fprintf('  пояснение                                : %s\n', ...
        char(exchange_status_message));
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

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Сформировать диагностический снимок состояния.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function diag = local_empty_exchange_diag(cfg)
%LOCAL_EMPTY_EXCHANGE_DIAG Построить пустую диагностику шага обмена.

diag = struct();
diag.status = "прием не подтвержден";
diag.status_message = "";
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
end
