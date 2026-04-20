%% RUN_ARDUPILOT_ARM_RATE_EXPERIMENTS Выполнить серию опытов по взведению ArduPilot.
% Назначение:
%   Проводит три диагностических опыта TASK-17 с различной частотой
%   обмена `MATLAB` и различными файлами параметров `SCHED_LOOP_RATE`.
%   Для каждого опыта запускает `ArduPilot SITL`, выполняет попытку
%   взведения через существующий сценарий и сохраняет сводные показатели
%   обмена, отказа и диапазонов ШИМ.
%
% Входы:
%   none
%
% Выходы:
%   task_17_arm_rate_experiments - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время - секунды;
%   частота - герцы;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   Используется уже существующий стенд TASK-15/TASK-16 без изменения
%   математической модели движения и без изменения моделей `Simulink`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
arm_pwm_script = fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m');
summary_log_path = fullfile(logs_dir, 'task_17_arm_rate_experiments.txt');
csv_report_path = fullfile(reports_dir, 'task_17_arm_rate_experiments.csv');
mat_report_path = fullfile(reports_dir, 'task_17_arm_rate_experiments.mat');

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

cfg = uav.ardupilot.default_json_config();
host_ip = local_resolve_windows_host_ip(cfg);
cfg.udp_remote_ip = host_ip;
cfg.mavlink_udp_ip = host_ip;

cases = local_make_case_definitions(repo_root, logs_dir);
case_results = repmat(local_empty_case_result(), numel(cases), 1);

for case_index = 1:numel(cases)
    case_cfg = cases(case_index);
    local_stop_existing_sitl(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port);
    start_result = local_start_sitl(cfg, host_ip, case_cfg.extra_defaults_path, case_cfg.start_log_path);
    local_write_utf8_text(case_cfg.start_log_path, local_make_start_log(start_result, case_cfg));

    case_result = local_empty_case_result();
    case_result.case_id = string(case_cfg.case_id);
    case_result.case_title = string(case_cfg.case_title);
    case_result.extra_defaults_path = string(case_cfg.extra_defaults_path);
    case_result.update_rate_hz = double(case_cfg.update_rate_hz);
    case_result.launch_command = string(start_result.command_text);
    case_result.launch_status = double(start_result.status_code);
    case_result.launch_pid = double(start_result.arducopter_pid);
    case_result.process_started = logical(start_result.process_alive);
    case_result.process_alive_after_case = logical(start_result.process_alive);
    case_result.start_output = string(start_result.output_text);

    if start_result.process_alive
        [temp_mat_path, temp_csv_path] = local_make_temp_response_paths(case_cfg.case_id);

        assignin('base', 'ardupilot_live_backend_duration_s', 20.0);
        assignin('base', 'ardupilot_live_backend_update_rate_hz', case_cfg.update_rate_hz);
        assignin('base', 'ardupilot_arm_attempt_delay_s', 15.0);
        assignin('base', 'ardupilot_arm_pwm_response_mat_path', temp_mat_path);
        assignin('base', 'ardupilot_arm_pwm_response_csv_path', temp_csv_path);

        run(arm_pwm_script);

        response = evalin('base', 'ardupilot_arm_pwm_response');
        metrics = uav.ardupilot.summarize_live_backend_metrics(response.live_result);

        case_result.valid_rx_count = double(metrics.valid_rx_count);
        case_result.json_tx_count = double(metrics.json_tx_count);
        case_result.response_tx_count = double(metrics.response_tx_count);
        case_result.average_exchange_rate_hz = double(metrics.response_tx_rate_hz);
        case_result.p95_exchange_period_s = double(metrics.response_tx_period_p95_s);
        case_result.last_frame_count = double(metrics.last_frame_count);
        case_result.gyro_rate_hz = double(local_extract_gyro_rate_hz(response.arm_attempt.status_texts));
        case_result.ack_result = double(response.arm_attempt.ack_result);
        case_result.arm_succeeded = logical(response.arm_attempt.arm_succeeded);
        case_result.failure_reason = local_pick_failure_reason(response.arm_attempt);
        case_result.status_texts = string(response.arm_attempt.status_texts(:));
        case_result.last_exchange_status = string(metrics.last_exchange_status);
        case_result.motor_pwm_before_min_us = local_min_finite(response.before.last_pwm_us);
        case_result.motor_pwm_before_max_us = local_max_finite(response.before.last_pwm_us);
        case_result.motor_pwm_after_min_us = local_min_finite(response.after.last_pwm_us);
        case_result.motor_pwm_after_max_us = local_max_finite(response.after.last_pwm_us);
        case_result.motor_cmd_before_min_radps = local_min_finite(response.before.last_motor_cmd_radps);
        case_result.motor_cmd_before_max_radps = local_max_finite(response.before.last_motor_cmd_radps);
        case_result.motor_cmd_after_min_radps = local_min_finite(response.after.last_motor_cmd_radps);
        case_result.motor_cmd_after_max_radps = local_max_finite(response.after.last_motor_cmd_radps);

        evalin('base', [ ...
            'clear(' ...
            '''ardupilot_live_backend_duration_s'', ' ...
            '''ardupilot_live_backend_update_rate_hz'', ' ...
            '''ardupilot_arm_attempt_delay_s'', ' ...
            '''ardupilot_arm_pwm_response_mat_path'', ' ...
            '''ardupilot_arm_pwm_response_csv_path'');']);

        if isfile(temp_mat_path)
            delete(temp_mat_path);
        end
        if isfile(temp_csv_path)
            delete(temp_csv_path);
        end
    else
        case_result.failure_reason = "Не удалось подтвердить запуск ArduPilot SITL.";
    end

    case_result.process_alive_after_case = local_is_wsl_pid_alive(cfg.wsl_distro_name, case_result.launch_pid);
    case_results(case_index) = case_result;

    local_stop_existing_sitl(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port);
end

experiments = struct();
experiments.host_ip = string(host_ip);
experiments.cases = case_results;
experiments.best_case_id = local_pick_best_case(case_results);
experiments.arm_succeeded = any([case_results.arm_succeeded]);

experiments_table = local_make_experiments_table(case_results);
local_save_mat_file(mat_report_path, 'experiments', experiments);
local_write_table_csv(csv_report_path, experiments_table);
local_write_utf8_text(summary_log_path, local_make_summary_log(case_results));

assignin('base', 'task_17_arm_rate_experiments', experiments);

fprintf('Серия опытов TASK-17 по частоте обмена и взведению ArduPilot\n');
fprintf('  лучший опыт                          : %s\n', char(experiments.best_case_id));
fprintf('  взведение подтверждено               : %s\n', local_bool_text(experiments.arm_succeeded));

function cases = local_make_case_definitions(repo_root, logs_dir)
%LOCAL_MAKE_CASE_DEFINITIONS Определить набор опытов TASK-17.

cases = repmat(struct(), 3, 1);

cases(1).case_id = "A";
cases(1).case_title = "Базовая конфигурация TASK-16";
cases(1).extra_defaults_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task16_arducopter_loop_rate.parm');
cases(1).update_rate_hz = 120.0;
cases(1).start_log_path = fullfile(logs_dir, 'task_17_case_a_start.txt');

cases(2).case_id = "B";
cases(2).case_title = "Ускоренный обмен, SCHED_LOOP_RATE = 50";
cases(2).extra_defaults_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task17_loop_rate_50.parm');
cases(2).update_rate_hz = 160.0;
cases(2).start_log_path = fullfile(logs_dir, 'task_17_case_b_start.txt');

cases(3).case_id = "C";
cases(3).case_title = "Ускоренный обмен, SCHED_LOOP_RATE = 30";
cases(3).extra_defaults_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task17_loop_rate_30.parm');
cases(3).update_rate_hz = 160.0;
cases(3).start_log_path = fullfile(logs_dir, 'task_17_case_c_start.txt');
end

function case_result = local_empty_case_result()
%LOCAL_EMPTY_CASE_RESULT Построить пустую структуру результата одного опыта.

case_result = struct( ...
    'case_id', "", ...
    'case_title', "", ...
    'extra_defaults_path', "", ...
    'update_rate_hz', 0.0, ...
    'launch_command', "", ...
    'launch_status', 0, ...
    'launch_pid', 0, ...
    'process_started', false, ...
    'process_alive_after_case', false, ...
    'start_output', "", ...
    'valid_rx_count', 0, ...
    'json_tx_count', 0, ...
    'response_tx_count', 0, ...
    'average_exchange_rate_hz', 0.0, ...
    'p95_exchange_period_s', nan, ...
    'gyro_rate_hz', 0.0, ...
    'ack_result', nan, ...
    'arm_succeeded', false, ...
    'failure_reason', "", ...
    'status_texts', strings(0, 1), ...
    'last_frame_count', 0, ...
    'motor_pwm_before_min_us', nan, ...
    'motor_pwm_before_max_us', nan, ...
    'motor_pwm_after_min_us', nan, ...
    'motor_pwm_after_max_us', nan, ...
    'motor_cmd_before_min_radps', nan, ...
    'motor_cmd_before_max_radps', nan, ...
    'motor_cmd_after_min_radps', nan, ...
    'motor_cmd_after_max_radps', nan, ...
    'last_exchange_status', "");
end

function [temp_mat_path, temp_csv_path] = local_make_temp_response_paths(case_id)
%LOCAL_MAKE_TEMP_RESPONSE_PATHS Построить временные пути результата попытки взведения.

case_suffix = char(lower(string(case_id)));
temp_mat_path = [tempname, '_task17_case_', case_suffix, '.mat'];
temp_csv_path = [tempname, '_task17_case_', case_suffix, '.csv'];
end

function start_result = local_start_sitl(cfg, host_ip, extra_defaults_path, start_log_path)
%LOCAL_START_SITL Запустить ArduPilot SITL через существующий PowerShell-сценарий.

script_path = fullfile( ...
    fileparts(fileparts(mfilename('fullpath'))), ...
    'tools', ...
    'ardupilot', ...
    'windows', ...
    'Start-ArduPilotJsonSitl.ps1');

command_text = sprintf([ ...
    'powershell -ExecutionPolicy Bypass -File "%s" ' ...
    '-DistroName %s -Ip %s -MavlinkPort %d -SecondaryMavlinkPort %d ' ...
    '-NoConsole -NoMap -Execute -ExtraDefaultsParmPath "%s" -LogPath "%s"'], ...
    script_path, ...
    char(cfg.wsl_distro_name), ...
    char(host_ip), ...
    cfg.mavlink_udp_port, ...
    cfg.mavlink_monitor_udp_port, ...
    extra_defaults_path, ...
    start_log_path);

    [status_code, output_text] = system(command_text);
pause(6.0);

arducopter_pid = local_query_arducopter_pid(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port);
process_alive = arducopter_pid > 0 && local_is_wsl_pid_alive(cfg.wsl_distro_name, arducopter_pid);

start_result = struct();
start_result.command_text = string(command_text);
start_result.status_code = double(status_code);
start_result.output_text = string(output_text);
start_result.arducopter_pid = double(arducopter_pid);
start_result.process_alive = logical(process_alive);
end

function pid_value = local_query_arducopter_pid(distro_name, host_ip, mavlink_port)
%LOCAL_QUERY_ARDUCOPTER_PID Найти PID процесса arducopter внутри WSL.

pattern = sprintf( ...
    'build/sitl/bin/arducopter.*udpclient:%s:%d', ...
    char(host_ip), ...
    mavlink_port);
command_text = sprintf( ...
    'wsl -d %s -- bash -lc "pgrep -f ''%s'' | head -n 1"', ...
    char(distro_name), ...
    pattern);
[status_code, output_text] = system(command_text);

pid_value = 0;
if status_code == 0
    pid_candidate = str2double(strtrim(output_text));
    if isfinite(pid_candidate)
        pid_value = double(pid_candidate);
    end
end
end

function is_alive = local_is_wsl_pid_alive(distro_name, pid_value)
%LOCAL_IS_WSL_PID_ALIVE Проверить, существует ли PID внутри WSL.

if pid_value <= 0
    is_alive = false;
    return;
end

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "kill -0 %d >/dev/null 2>&1"', ...
    char(distro_name), ...
    round(pid_value));
[status_code, ~] = system(command_text);
is_alive = status_code == 0;
end

function local_stop_existing_sitl(distro_name, host_ip, mavlink_port)
%LOCAL_STOP_EXISTING_SITL Остановить процессы SITL текущего стенда.

pattern = sprintf( ...
    'build/sitl/bin/arducopter.*udpclient:%s:%d', ...
    char(host_ip), ...
    mavlink_port);
command_text = sprintf( ...
    'wsl -d %s -- bash -lc "pkill -f ''%s'' || true"', ...
    char(distro_name), ...
    pattern);
system(command_text);
pause(1.0);
end

function host_ip = local_resolve_windows_host_ip(cfg)
%LOCAL_RESOLVE_WINDOWS_HOST_IP Определить адрес Windows-хоста для WSL2.

host_ip = string(cfg.udp_remote_ip);
command_text = sprintf( ...
    'wsl -d %s -- bash -lc "ip -4 route list default | cut -d'' '' -f3 | head -n 1"', ...
    char(cfg.wsl_distro_name));
[status_code, output_text] = system(command_text);

if status_code == 0
    resolved = strtrim(output_text);
    if ~isempty(resolved)
        host_ip = string(resolved);
    end
end
end

function rate_hz = local_extract_gyro_rate_hz(status_texts)
%LOCAL_EXTRACT_GYRO_RATE_HZ Попытаться извлечь частоту гироскопа из STATUSTEXT.

rate_hz = 0.0;
for idx = 1:numel(status_texts)
    one_text = string(status_texts(idx));
    if contains(one_text, "Gyro")
        token = regexp(char(one_text), '(\d+(?:\.\d+)?)Hz', 'tokens', 'once');
        if ~isempty(token)
            rate_hz = str2double(token{1});
            return;
        end
    end
end
end

function reason_text = local_pick_failure_reason(arm_attempt)
%LOCAL_PICK_FAILURE_REASON Выбрать наиболее содержательную причину отказа.

reason_text = string(arm_attempt.failure_reason);
status_texts = string(arm_attempt.status_texts(:));

for idx = 1:numel(status_texts)
    if startsWith(strtrim(status_texts(idx)), "Arm:")
        reason_text = strtrim(status_texts(idx));
        return;
    end
end

if strlength(strtrim(reason_text)) == 0 && isfinite(double(arm_attempt.ack_result))
    reason_text = "Команда взведения отклонена с кодом " + string(double(arm_attempt.ack_result)) + ".";
end
end

function best_case_id = local_pick_best_case(case_results)
%LOCAL_PICK_BEST_CASE Выбрать лучший опыт по числу ответных передач.

response_counts = [case_results.response_tx_count];
valid_counts = [case_results.valid_rx_count];
[~, idx] = max(response_counts + 1.0e-3 * valid_counts);
best_case_id = string(case_results(idx).case_id);
end

function value = local_min_finite(vec_value)
%LOCAL_MIN_FINITE Вернуть минимум по конечным значениям.

finite_values = double(vec_value(:));
finite_values = finite_values(isfinite(finite_values));
if isempty(finite_values)
    value = nan;
else
    value = min(finite_values);
end
end

function value = local_max_finite(vec_value)
%LOCAL_MAX_FINITE Вернуть максимум по конечным значениям.

finite_values = double(vec_value(:));
finite_values = finite_values(isfinite(finite_values));
if isempty(finite_values)
    value = nan;
else
    value = max(finite_values);
end
end

function table_value = local_make_experiments_table(case_results)
%LOCAL_MAKE_EXPERIMENTS_TABLE Построить CSV-таблицу по опытам TASK-17.

table_value = table( ...
    string({case_results.case_id}).', ...
    string({case_results.case_title}).', ...
    string({case_results.extra_defaults_path}).', ...
    [case_results.update_rate_hz].', ...
    string({case_results.launch_command}).', ...
    [case_results.launch_pid].', ...
    logical([case_results.process_started].'), ...
    [case_results.valid_rx_count].', ...
    [case_results.json_tx_count].', ...
    [case_results.response_tx_count].', ...
    [case_results.average_exchange_rate_hz].', ...
    [case_results.p95_exchange_period_s].', ...
    [case_results.gyro_rate_hz].', ...
    [case_results.ack_result].', ...
    logical([case_results.arm_succeeded].'), ...
    string({case_results.failure_reason}).', ...
    logical([case_results.process_alive_after_case].'), ...
    [case_results.last_frame_count].', ...
    [case_results.motor_pwm_before_min_us].', ...
    [case_results.motor_pwm_before_max_us].', ...
    [case_results.motor_pwm_after_min_us].', ...
    [case_results.motor_pwm_after_max_us].', ...
    [case_results.motor_cmd_before_min_radps].', ...
    [case_results.motor_cmd_before_max_radps].', ...
    [case_results.motor_cmd_after_min_radps].', ...
    [case_results.motor_cmd_after_max_radps].', ...
    string({case_results.last_exchange_status}).', ...
    'VariableNames', { ...
        'case_id', ...
        'case_title', ...
        'extra_defaults_path', ...
        'update_rate_hz', ...
        'launch_command', ...
        'arducopter_pid', ...
        'process_started', ...
        'valid_rx_count', ...
        'json_tx_count', ...
        'response_tx_count', ...
        'average_exchange_rate_hz', ...
        'p95_exchange_period_s', ...
        'gyro_rate_hz', ...
        'ack_result', ...
        'arm_succeeded', ...
        'failure_reason', ...
        'process_alive_after_case', ...
        'last_frame_count', ...
        'motor_pwm_before_min_us', ...
        'motor_pwm_before_max_us', ...
        'motor_pwm_after_min_us', ...
        'motor_pwm_after_max_us', ...
        'motor_cmd_before_min_radps', ...
        'motor_cmd_before_max_radps', ...
        'motor_cmd_after_min_radps', ...
        'motor_cmd_after_max_radps', ...
        'last_exchange_status'});
end

function text_value = local_make_start_log(start_result, case_cfg)
%LOCAL_MAKE_START_LOG Построить текст журнала запуска одного опыта.

lines = strings(0, 1);
lines(end + 1, 1) = "Запуск ArduPilot SITL для опыта TASK-17";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "идентификатор опыта                : " + case_cfg.case_id;
lines(end + 1, 1) = "описание опыта                     : " + case_cfg.case_title;
lines(end + 1, 1) = "команда запуска                    : " + start_result.command_text;
lines(end + 1, 1) = "дополнительный файл параметров     : " + string(case_cfg.extra_defaults_path);
lines(end + 1, 1) = "update_rate_hz MATLAB              : " + sprintf('%.0f', case_cfg.update_rate_hz);
lines(end + 1, 1) = "статус запуска PowerShell          : " + sprintf('%d', start_result.status_code);
lines(end + 1, 1) = "PID arducopter                     : " + sprintf('%d', start_result.arducopter_pid);
lines(end + 1, 1) = "процесс arducopter жив             : " + local_bool_text(start_result.process_alive);
output_lines = splitlines(start_result.output_text);
for idx = 1:numel(output_lines)
    one_line = strtrim(output_lines(idx));
    if strlength(one_line) > 0
        lines(end + 1, 1) = "  " + one_line; %#ok<AGROW>
    end
end
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_make_summary_log(case_results)
%LOCAL_MAKE_SUMMARY_LOG Сформировать итоговый текстовый журнал опытов.

lines = strings(0, 1);
lines(end + 1, 1) = "Серия опытов TASK-17 по частоте обмена и взведению ArduPilot";
lines(end + 1, 1) = "============================================================";

for idx = 1:numel(case_results)
    case_result = case_results(idx);
    lines(end + 1, 1) = "Опыт " + case_result.case_id + ". " + case_result.case_title;
    lines(end + 1, 1) = "  команда запуска                   : " + case_result.launch_command;
    lines(end + 1, 1) = "  файл параметров                   : " + case_result.extra_defaults_path;
    lines(end + 1, 1) = "  update_rate_hz MATLAB             : " + sprintf('%.0f', case_result.update_rate_hz);
    lines(end + 1, 1) = "  запуск процесса                   : " + local_bool_text(case_result.process_started);
    lines(end + 1, 1) = "  PID arducopter                    : " + sprintf('%d', case_result.launch_pid);
    lines(end + 1, 1) = "  valid_rx_count                    : " + sprintf('%d', case_result.valid_rx_count);
    lines(end + 1, 1) = "  json_tx_count                     : " + sprintf('%d', case_result.json_tx_count);
    lines(end + 1, 1) = "  response_tx_count                 : " + sprintf('%d', case_result.response_tx_count);
    lines(end + 1, 1) = "  средняя частота обмена [Hz]       : " + sprintf('%.4f', case_result.average_exchange_rate_hz);
    lines(end + 1, 1) = "  p95 периода обмена [s]            : " + sprintf('%.6f', case_result.p95_exchange_period_s);
    lines(end + 1, 1) = "  gyro_rate_hz по STATUSTEXT        : " + sprintf('%.0f', case_result.gyro_rate_hz);
    if isfinite(case_result.ack_result)
        lines(end + 1, 1) = "  ack_result                        : " + sprintf('%.0f', case_result.ack_result);
    end
    lines(end + 1, 1) = "  взведение подтверждено            : " + local_bool_text(case_result.arm_succeeded);
    lines(end + 1, 1) = "  arducopter жив после опыта        : " + local_bool_text(case_result.process_alive_after_case);
    lines(end + 1, 1) = "  код завершения процесса           : ";
    lines(end + 1, 1) = "  last_frame_count                  : " + sprintf('%d', case_result.last_frame_count);
    lines(end + 1, 1) = "  motor_pwm до [us]                 : [" + sprintf('%.6f..%.6f', case_result.motor_pwm_before_min_us, case_result.motor_pwm_before_max_us) + ", " + sprintf('%.6f..%.6f', case_result.motor_pwm_before_min_us, case_result.motor_pwm_before_max_us) + ", " + sprintf('%.6f..%.6f', case_result.motor_pwm_before_min_us, case_result.motor_pwm_before_max_us) + ", " + sprintf('%.6f..%.6f', case_result.motor_pwm_before_min_us, case_result.motor_pwm_before_max_us) + "]";
    lines(end + 1, 1) = "  motor_pwm после [us]              : [" + sprintf('%.6f..%.6f', case_result.motor_pwm_after_min_us, case_result.motor_pwm_after_max_us) + ", " + sprintf('%.6f..%.6f', case_result.motor_pwm_after_min_us, case_result.motor_pwm_after_max_us) + ", " + sprintf('%.6f..%.6f', case_result.motor_pwm_after_min_us, case_result.motor_pwm_after_max_us) + ", " + sprintf('%.6f..%.6f', case_result.motor_pwm_after_min_us, case_result.motor_pwm_after_max_us) + "]";
    lines(end + 1, 1) = "  motor_cmd до [rad/s]              : [" + sprintf('%.6f..%.6f', case_result.motor_cmd_before_min_radps, case_result.motor_cmd_before_max_radps) + ", " + sprintf('%.6f..%.6f', case_result.motor_cmd_before_min_radps, case_result.motor_cmd_before_max_radps) + ", " + sprintf('%.6f..%.6f', case_result.motor_cmd_before_min_radps, case_result.motor_cmd_before_max_radps) + ", " + sprintf('%.6f..%.6f', case_result.motor_cmd_before_min_radps, case_result.motor_cmd_before_max_radps) + "]";
    lines(end + 1, 1) = "  motor_cmd после [rad/s]           : [" + sprintf('%.6f..%.6f', case_result.motor_cmd_after_min_radps, case_result.motor_cmd_after_max_radps) + ", " + sprintf('%.6f..%.6f', case_result.motor_cmd_after_min_radps, case_result.motor_cmd_after_max_radps) + ", " + sprintf('%.6f..%.6f', case_result.motor_cmd_after_min_radps, case_result.motor_cmd_after_max_radps) + ", " + sprintf('%.6f..%.6f', case_result.motor_cmd_after_min_radps, case_result.motor_cmd_after_max_radps) + "]";
    lines(end + 1, 1) = "  итоговый статус обмена            : " + case_result.last_exchange_status;
    lines(end + 1, 1) = "  причина                           : " + case_result.failure_reason;
    if ~isempty(case_result.status_texts)
        lines(end + 1, 1) = "  последние STATUSTEXT:";
        for text_idx = 1:numel(case_result.status_texts)
            lines(end + 1, 1) = "    " + strtrim(case_result.status_texts(text_idx)); %#ok<AGROW>
        end
    end
    lines(end + 1, 1) = "";
end

best_case_id = local_pick_best_case(case_results);
best_idx = find(string({case_results.case_id}) == best_case_id, 1, 'first');
lines(end + 1, 1) = "Лучший опыт: " + best_case_id;
lines(end + 1, 1) = "Взведение подтверждено: " + local_bool_text(any([case_results.arm_succeeded]));
if ~isempty(best_idx)
    lines(end + 1, 1) = "Лучшая конфигурация параметров: " + case_results(best_idx).extra_defaults_path;
end
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст UTF-8 без BOM.

folder_path = fileparts(path_value);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:task17:arm_experiments:OpenFile', ...
        'Не удалось открыть файл %s.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function local_save_mat_file(path_value, variable_name, variable_value)
%LOCAL_SAVE_MAT_FILE Сохранить MAT-файл через временный путь с проверкой.

folder_path = fileparts(path_value);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

temp_path = [tempname, '.mat'];
payload = struct();
payload.(variable_name) = variable_value;
save(temp_path, '-struct', 'payload', '-v7');

if ~isfile(temp_path) || dir(temp_path).bytes == 0
    error( ...
        'uav:task17:arm_experiments:EmptyMatFile', ...
        'Не удалось сформировать непустой MAT-файл %s.', ...
        path_value);
end

movefile(temp_path, path_value, 'f');

if ~isfile(path_value) || dir(path_value).bytes == 0
    error( ...
        'uav:task17:arm_experiments:MatFileMoveFailed', ...
        'После сохранения MAT-файл %s остался пустым.', ...
        path_value);
end
end

function local_write_table_csv(path_value, table_value)
%LOCAL_WRITE_TABLE_CSV Сохранить CSV-таблицу через временный путь с проверкой.

folder_path = fileparts(path_value);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

temp_path = [tempname, '.csv'];
writetable(table_value, temp_path, 'Encoding', 'UTF-8');

if ~isfile(temp_path) || dir(temp_path).bytes == 0
    error( ...
        'uav:task17:arm_experiments:EmptyCsvFile', ...
        'Не удалось сформировать непустой CSV-файл %s.', ...
        path_value);
end

movefile(temp_path, path_value, 'f');

if ~isfile(path_value) || dir(path_value).bytes == 0
    error( ...
        'uav:task17:arm_experiments:CsvFileMoveFailed', ...
        'После сохранения CSV-файл %s остался пустым.', ...
        path_value);
end
end
