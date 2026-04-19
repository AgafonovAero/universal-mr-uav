%% RUN_ARDUPILOT_JSON_UDP_HANDSHAKE Выполнить попытку обмена с ArduPilot.
% Назначение:
%   Выполняет короткую попытку обмена с уже запущенным `ArduPilot JSON SITL`
%   без установки стороннего программного комплекса и без заявления
%   готовности к полноценному автоматическому полету.
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
%   При отсутствии ответа от `ArduPilot` скрипт завершается штатно и
%   фиксирует отсутствие двоичного пакета в явном виде.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
availability = uav.ardupilot.json_udp_is_available();
env_info = uav.ardupilot.inspect_sitl_environment(cfg);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = max(cfg.handshake_wait_timeout_s, ...
    cfg.handshake_max_steps / cfg.update_rate_hz);
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_udp(case_cfg);
valid_indices = find([log.sitl_output.valid], 1, 'first');
packet_received = ~isempty(valid_indices);
json_sent = any(contains(log.exchange_status, "отправлена"));

if packet_received
    rx_out = log.sitl_output(valid_indices);
    first_pwm = rx_out.motor_pwm_us;
    exchange_status = log.exchange_status(valid_indices);
else
    rx_out = uav.ardupilot.decode_sitl_output_packet([], cfg);
    first_pwm = zeros(cfg.motor_count, 1);
    exchange_status = log.exchange_status(end);
end

result = struct();
result.availability = availability;
result.env_info = env_info;
result.start_command = start_cmd;
result.log = log;
result.packet_received = logical(packet_received);
result.json_sent = logical(json_sent);
result.rx_out = rx_out;
result.exchange_status = string(exchange_status);

assignin('base', 'ardupilot_json_udp_handshake', result);

fprintf('Проверка обмена с ArduPilot по JSON и UDP\n');
fprintf('  готовность UDP                         : %s\n', ...
    local_bool_text(availability.is_available));
fprintf('  готовность локальной среды             : %s\n', ...
    local_bool_text(env_info.is_ready));
fprintf('  пакет от ArduPilot получен             : %s\n', ...
    local_bool_text(packet_received));

if packet_received
    fprintf('  magic                                  : %u\n', rx_out.magic);
    fprintf('  frame_rate_hz                          : %.0f\n', rx_out.frame_rate_hz);
    fprintf('  frame_count                            : %.0f\n', rx_out.frame_count);
    fprintf('  первые четыре значения ШИМ [us]        : [%s]\n', ...
        local_format_vector(first_pwm));
    fprintf('  строка JSON отправлена                 : %s\n', ...
        local_bool_text(json_sent));
    fprintf('  статус обмена                          : %s\n', ...
        char(string(exchange_status)));
else
    fprintf('  среда не готова                        : %s\n', ...
        local_bool_text(~env_info.is_ready));
    fprintf('  пакет от ArduPilot не получен          : да\n');
    fprintf('  команда запуска для отдельного прогона : %s\n', ...
        char(start_cmd.command_text));
    fprintf('  пояснение                              : %s\n', ...
        char(start_cmd.message));
    fprintf('  статус обмена                          : %s\n', ...
        char(string(exchange_status)));
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
