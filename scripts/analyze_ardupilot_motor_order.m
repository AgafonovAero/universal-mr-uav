%% ANALYZE_ARDUPILOT_MOTOR_ORDER Оценить подтверждение порядка двигателей ArduPilot.
% Назначение:
%   Использует результаты наблюдения ШИМ и, при наличии, сравнения до/после
%   взведения для формирования инженерного заключения по порядку двигателей
%   и знакам групп рыскания.

params = uav.sim.make_deterministic_demo_params();
repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
if ~isfolder(logs_dir)
    mkdir(logs_dir);
end
if ~isfolder(reports_dir)
    mkdir(reports_dir);
end
source_data = local_load_best_source();
analysis = uav.ardupilot.analyze_motor_order_response( ...
    source_data.motor_pwm_us, ...
    source_data.motor_cmd_radps, ...
    params);

assignin('base', 'ardupilot_motor_order_analysis', analysis);

log_lines = strings(0, 1);
log_lines(end + 1, 1) = "Анализ порядка двигателей ArduPilot";
log_lines(end + 1, 1) = "  valid_sample_count                  : " + string(analysis.valid_sample_count);
log_lines(end + 1, 1) = "  sufficient_excitation               : " + local_bool_text(analysis.sufficient_excitation);
log_lines(end + 1, 1) = "  order_confirmed                     : " + local_bool_text(analysis.order_confirmed);
log_lines(end + 1, 1) = "  pwm_span_us                         : [" + local_format_vector(analysis.pwm_span_us) + "]";
log_lines(end + 1, 1) = "  motor_cmd_span_radps                : [" + local_format_vector(analysis.motor_cmd_span_radps) + "]";
log_lines(end + 1, 1) = "  yaw_pair_delta_us                   : [" + local_format_vector(analysis.yaw_pair_delta_us) + "]";
log_lines(end + 1, 1) = "  пояснение                           : " + string(analysis.message);

local_write_utf8_text(fullfile(logs_dir, 'task_16_motor_order_analysis.txt'), strjoin(log_lines, newline) + newline);
local_write_utf8_text(fullfile(reports_dir, 'task_16_motor_order_analysis_ru.md'), local_make_report(analysis));

fprintf('%s\n', char(strjoin(log_lines, newline)));

function source_data = local_load_best_source()
%LOCAL_LOAD_BEST_SOURCE Выбрать лучший источник данных для анализа.

repo_root = fileparts(fileparts(mfilename('fullpath')));
arm_response_path = fullfile(repo_root, 'artifacts', 'reports', 'task_16_arm_pwm_response.mat');
pwm_observation_path = fullfile(repo_root, 'artifacts', 'reports', 'task_16_pwm_observation.mat');

if isfile(arm_response_path)
    loaded = load(arm_response_path);
    if isfield(loaded, 'response') && isfield(loaded.response, 'after') ...
            && isfield(loaded.response.after, 'executed') && loaded.response.after.executed
        source_data = struct();
        source_data.motor_pwm_us = local_extract_pwm_matrix(loaded.response.after);
        source_data.motor_cmd_radps = loaded.response.after.motor_cmd_radps;
        return;
    end
end

loaded = load(pwm_observation_path);
source_data = struct();
source_data.motor_pwm_us = loaded.observation.motor_pwm_us;
source_data.motor_cmd_radps = loaded.observation.motor_cmd_radps;
end

function pwm_matrix = local_extract_pwm_matrix(result_struct)
%LOCAL_EXTRACT_PWM_MATRIX Извлечь матрицу ШИМ из результата прогона.

n_rows = numel(result_struct.sitl_output);
pwm_matrix = nan(n_rows, 4);
for idx = 1:n_rows
    if result_struct.sitl_output(idx).valid
        pwm_matrix(idx, :) = double(result_struct.sitl_output(idx).motor_pwm_us(:)).';
    end
end
end

function text_value = local_make_report(analysis)
%LOCAL_MAKE_REPORT Сформировать краткий отчет Markdown.

lines = strings(0, 1);
lines(end + 1, 1) = "# Анализ порядка двигателей ArduPilot";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "## Результат";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "- Достаточность возбуждения: " + local_bool_text(analysis.sufficient_excitation);
lines(end + 1, 1) = "- Подтверждение порядка двигателей: " + local_bool_text(analysis.order_confirmed);
lines(end + 1, 1) = "- Диапазон ШИМ по каналам [us]: [" + local_format_vector(analysis.pwm_span_us) + "]";
lines(end + 1, 1) = "- Диапазон команд частоты вращения [rad/s]: [" + local_format_vector(analysis.motor_cmd_span_radps) + "]";
lines(end + 1, 1) = "- Пояснение: " + string(analysis.message);
text_value = strjoin(lines, newline) + newline;
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить UTF-8 текст без BOM.

folder_path = fileparts(path_value);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:ardupilot:analyze_ardupilot_motor_order:OpenFile', ...
        'Не удалось открыть файл %s для записи.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать строку вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end
