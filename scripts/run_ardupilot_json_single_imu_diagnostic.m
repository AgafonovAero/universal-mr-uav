%% RUN_ARDUPILOT_JSON_SINGLE_IMU_DIAGNOSTIC Проверить arm в профиле одного ИНС.
% Назначение:
%   Выполняет диагностический прогон ArduPilot JSON + MATLAB-модель с
%   дополнительным файлом параметров, который оставляет активным только
%   первый виртуальный ИНС. Сценарий не является штатным режимом стенда и
%   используется только для локализации причины отказа "Arm: Accels inconsistent".
%
% Входы:
%   none
%
% Выходы:
%   task_19_single_imu_diagnostic - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   ArduPilot в WSL уже собран, а MATLAB запущен после bootstrap_project.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task19_single_imu_diagnostic.parm');
if ~isfile(parm_path)
    error('uav:task19:singleimu:MissingParm', ...
        'Не найден файл диагностических параметров: %s', ...
        parm_path);
end

cfg = uav.ardupilot.default_json_config();
cfg_override = struct( ...
    'json_accel_mode', string(cfg.json_accel_mode), ...
    'json_prearm_hold_enabled', true, ...
    'json_prearm_pwm_threshold_us', 1005.0);

mat_path = fullfile(reports_dir, 'task_19_single_imu_diagnostic.mat');
csv_path = fullfile(reports_dir, 'task_19_single_imu_diagnostic.csv');
log_path = fullfile(logs_dir, 'task_19_single_imu_diagnostic.txt');

baseline_mat_tmp = [tempname, '_task19_single_imu_baseline.mat'];
baseline_csv_tmp = [tempname, '_task19_single_imu_baseline.csv'];
arm_mat_tmp = [tempname, '_task19_single_imu_arm.mat'];
arm_csv_tmp = [tempname, '_task19_single_imu_arm.csv'];

local_assign_base('ardupilot_json_cfg_override', cfg_override);
local_assign_base('ardupilot_task15_extra_defaults_win_path', parm_path);
local_assign_base('ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, 'task_19_single_imu_baseline.txt'));
local_assign_base('ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, 'task_19_single_imu_wait.txt'));
local_assign_base('ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, 'task_19_single_imu_handshake.txt'));
local_assign_base('ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, 'task_19_single_imu_live_backend.txt'));
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
local_assign_base('ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
local_assign_base('ardupilot_arm_attempt_delay_s', 30.0);
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_json_cfg_override'', ' ...
    '''ardupilot_task15_extra_defaults_win_path'', ' ...
    '''ardupilot_task15_baseline_diag_log_path'', ' ...
    '''ardupilot_task15_baseline_wait_log_path'', ' ...
    '''ardupilot_task15_baseline_handshake_log_path'', ' ...
    '''ardupilot_task15_baseline_live_log_path'', ' ...
    '''ardupilot_task15_baseline_mat_report_path'', ' ...
    '''ardupilot_task15_baseline_csv_report_path'', ' ...
    '''ardupilot_arm_pwm_response_mat_path'', ' ...
    '''ardupilot_arm_pwm_response_csv_path'', ' ...
    '''ardupilot_arm_attempt_delay_s'');'])); %#ok<NASGU>
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp, arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m'));
baseline = evalin('base', 'task_17_task15_baseline_exchange');

response = struct();
if baseline.baseline_restored
    run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
    response = evalin('base', 'ardupilot_arm_pwm_response');
end

result = struct();
result.parm_path = string(parm_path);
result.cfg_override = cfg_override;
result.baseline = baseline;
result.arm_response = response;
result.arm_succeeded = false;
result.failure_reason = string(baseline.first_failure_reason);
result.ack_result = nan;
result.motor_pwm_us_range = [nan nan];
result.motor_cmd_radps_range = [nan nan];

if baseline.baseline_restored && ~isempty(fieldnames(response))
    result.arm_succeeded = logical(response.arm_attempt.arm_succeeded);
    result.failure_reason = string(response.arm_attempt.failure_reason);
    result.ack_result = double(response.arm_attempt.ack_result);
    result.motor_pwm_us_range = local_range(response.live_result.sitl_output, 'motor_pwm_us');
    result.motor_cmd_radps_range = local_numeric_range(response.live_result.motor_cmd_radps);
end

save(mat_path, 'result');
writetable(local_make_result_table(result), csv_path);
local_write_utf8_text(log_path, local_make_log_text(result));
assignin('base', 'task_19_single_imu_diagnostic', result);

fprintf('Диагностический профиль одного ИНС TASK-19\n');
fprintf('  baseline restored                      : %s\n', local_bool_text(baseline.baseline_restored));
fprintf('  arm succeeded                          : %s\n', local_bool_text(result.arm_succeeded));
fprintf('  ACK                                    : %.0f\n', result.ack_result);
fprintf('  failure reason                         : %s\n', char(result.failure_reason));

function local_assign_base(name, value)
%LOCAL_ASSIGN_BASE Записать переменную в base workspace.
assignin('base', name, value);
end

function local_cleanup_temp(file_list)
%LOCAL_CLEANUP_TEMP Удалить временные файлы.
for idx = 1:numel(file_list)
    if isfile(file_list{idx})
        delete(file_list{idx});
    end
end
end

function table_value = local_make_result_table(result)
%LOCAL_MAKE_RESULT_TABLE Сформировать CSV-таблицу диагностического прогона.

table_value = table( ...
    string(result.parm_path), ...
    double(result.baseline.metrics.valid_rx_count), ...
    double(result.baseline.metrics.response_tx_count), ...
    double(result.baseline.metrics.last_frame_count), ...
    logical(result.arm_succeeded), ...
    double(result.ack_result), ...
    string(result.failure_reason), ...
    double(result.motor_pwm_us_range(1)), ...
    double(result.motor_pwm_us_range(2)), ...
    double(result.motor_cmd_radps_range(1)), ...
    double(result.motor_cmd_radps_range(2)), ...
    'VariableNames', { ...
        'parm_path', ...
        'baseline_valid_rx_count', ...
        'baseline_response_tx_count', ...
        'baseline_last_frame_count', ...
        'arm_succeeded', ...
        'ack_result', ...
        'failure_reason', ...
        'motor_pwm_min_us', ...
        'motor_pwm_max_us', ...
        'motor_cmd_min_radps', ...
        'motor_cmd_max_radps'});
end

function text_value = local_make_log_text(result)
%LOCAL_MAKE_LOG_TEXT Сформировать текстовый журнал диагностического прогона.

lines = strings(0, 1);
lines(end + 1, 1) = "TASK-19: диагностический профиль одного виртуального ИНС";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Файл параметров: " + string(result.parm_path);
lines(end + 1, 1) = "baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "baseline last_frame_count: " + string(result.baseline.metrics.last_frame_count);
lines(end + 1, 1) = "arm выполнен: " + local_bool_text(result.arm_succeeded);
lines(end + 1, 1) = "ACK: " + string(result.ack_result);
lines(end + 1, 1) = "Причина: " + string(result.failure_reason);
lines(end + 1, 1) = "Диапазон motor_pwm_us: [" + local_format_range(result.motor_pwm_us_range) + "]";
lines(end + 1, 1) = "Диапазон motor_cmd_radps: [" + local_format_range(result.motor_cmd_radps_range) + "]";
text_value = strjoin(lines, newline) + newline;
end

function range_value = local_range(packet_array, field_name)
%LOCAL_RANGE Вычислить диапазон вектора из массива структур.

values = [];
for idx = 1:numel(packet_array)
    if isfield(packet_array(idx), field_name)
        item = double(packet_array(idx).(field_name));
        item = item(isfinite(item));
        values = [values; item(:)]; %#ok<AGROW>
    end
end

range_value = local_numeric_range(values);
end

function range_value = local_numeric_range(values)
%LOCAL_NUMERIC_RANGE Вычислить диапазон числового массива.

values = double(values(:));
values = values(isfinite(values));
if isempty(values)
    range_value = [nan nan];
else
    range_value = [min(values), max(values)];
end
end

function text_value = local_format_range(range_value)
%LOCAL_FORMAT_RANGE Сформировать компактную строку диапазона.

if numel(range_value) ~= 2 || any(~isfinite(range_value))
    text_value = "NaN NaN";
else
    text_value = sprintf('%.6f %.6f', range_value(1), range_value(2));
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function local_write_utf8_text(file_path, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

folder_path = fileparts(file_path);
if strlength(string(folder_path)) > 0 && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(file_path, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task19:singleimu:OpenFile', ...
        'Не удалось открыть файл %s для записи.', ...
        file_path);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end
