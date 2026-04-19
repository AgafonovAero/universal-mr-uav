%% RUN_ARDUPILOT_WAIT_FOR_PACKET Ожидать первый пакет от ArduPilot SITL.
% Назначение:
%   Печатает диагностическую информацию о локальной среде, формирует
%   рекомендуемую команду запуска `ArduPilot SITL`, затем ожидает
%   входящий двоичный пакет в течение ограниченного времени.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_wait_for_packet - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   длительность импульсов ШИМ задается в микросекундах
%
% Допущения:
%   Отсутствие пакета от `ArduPilot` не считается аварией и должно
%   возвращать оператору команду запуска для отдельной ручной проверки.

cfg = uav.ardupilot.default_json_config();
env_info = uav.ardupilot.inspect_sitl_environment(cfg);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);
result = uav.ardupilot.wait_for_sitl_output_packet(cfg);

output = struct();
output.cfg = cfg;
output.env_info = env_info;
output.start_command = start_cmd;
output.wait_result = result;

assignin('base', 'ardupilot_wait_for_packet', output);

fprintf('Ожидание первого двоичного пакета от ArduPilot\n');
fprintf('  готовность локальной среды                : %s\n', ...
    local_bool_text(env_info.is_ready));
fprintf('  команда запуска                           : %s\n', ...
    char(start_cmd.command_text));
fprintf('  пакет получен                             : %s\n', ...
    local_bool_text(result.received));
fprintf('  тайм-аут ожидания [s]                     : %.3f\n', ...
    result.timeout_s);
fprintf('  фактическое время ожидания [s]            : %.3f\n', ...
    result.elapsed_s);

if result.received
    fprintf('  magic                                     : %u\n', ...
        result.sitl_output.magic);
    fprintf('  частота кадров [Hz]                       : %.0f\n', ...
        result.sitl_output.frame_rate_hz);
    fprintf('  счетчик кадров                            : %.0f\n', ...
        result.sitl_output.frame_count);
    fprintf('  первые четыре значения ШИМ [us]           : [%s]\n', ...
        local_format_vector(result.sitl_output.motor_pwm_us));
else
    fprintf('  реальный пакет ArduPilot не получен       : да\n');
    fprintf('  рекомендуемая команда запуска             : %s\n', ...
        char(start_cmd.command_text));
end

fprintf('  пояснение                                 : %s\n', ...
    char(result.message));

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
