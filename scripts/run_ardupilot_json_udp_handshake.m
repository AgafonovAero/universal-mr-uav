%% RUN_ARDUPILOT_JSON_UDP_HANDSHAKE Выполнить попытку обмена с ArduPilot.
% Назначение:
%   Выполняет короткую попытку обмена с уже запущенным `ArduPilot JSON SITL`
%   без установки внешнего программного комплекса и без заявлений о
%   готовности к устойчивому автоматическому полету.
%   Сценарий различает:
%   - отсутствие подтвержденного приема;
%   - исходящую пробную передачу строки JSON;
%   - ответную передачу после принятого пакета.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_json_udp_handshake - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   длительности импульсов ШИМ задаются в микросекундах
%
% Допущения:
%   При отсутствии ответа от `ArduPilot` сценарий завершается штатно и
%   честно фиксирует отсутствие принятого двоичного пакета.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
availability = uav.ardupilot.json_udp_is_available();
env_info = uav.ardupilot.inspect_sitl_environment(cfg);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = max( ...
    cfg.udp_handshake_timeout_s, ...
    cfg.handshake_max_steps / cfg.update_rate_hz);
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_udp(case_cfg);
diag_hist = log.exchange_diag;
status_values = arrayfun(@(item) string(item.status), diag_hist);

reply_indices = find([diag_hist.handshake_confirmed], 1, 'first');
probe_indices = find(status_values == "исходящая пробная передача", 1, 'first');
packet_indices = find([diag_hist.rx_valid], 1, 'first');

packet_received = ~isempty(packet_indices);
reply_json_sent = ~isempty(reply_indices);
probe_json_sent = ~isempty(probe_indices);

if packet_received
    rx_out = log.sitl_output(packet_indices);
    exchange_status = string(diag_hist(packet_indices).status);
    exchange_status_message = string(diag_hist(packet_indices).status_message);
else
    rx_out = uav.ardupilot.decode_sitl_output_packet([], cfg);
    exchange_status = string(diag_hist(end).status);
    exchange_status_message = string(diag_hist(end).status_message);
end

result = struct();
result.availability = availability;
result.env_info = env_info;
result.start_command = start_cmd;
result.log = log;
result.packet_received = logical(packet_received);
result.probe_json_sent = logical(probe_json_sent);
result.reply_json_sent = logical(reply_json_sent);
result.rx_out = rx_out;
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
    fprintf('  исходящая пробная строка JSON отправлена : %s\n', ...
        local_bool_text(probe_json_sent));
    fprintf('  ответная передача JSON выполнена         : нет\n');
    fprintf([ ...
        '  ответ на принятый двоичный пакет ArduPilot не проверен, ' ...
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
