%% MEASURE_ARDUPILOT_JSON_EXCHANGE_RATE Измерить частоту обмена ArduPilot JSON/UDP.
% Назначение:
%   Запускает `ArduPilot SITL` в наблюдаемой конфигурации TASK-17,
%   выполняет не менее 20 секунд обмена с математической моделью и
%   формирует измерение фактической частоты двоичных пакетов и ответных
%   строк `JSON`.
%
% Входы:
%   none
%
% Выходы:
%   task_17_exchange_rate_measurement - структура результата в базовом
%   рабочем пространстве MATLAB
%
% Единицы измерения:
%   время - секунды;
%   частота - герцы;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   Средство обмена TASK-15 уже работоспособно, а `ArduPilot` установлен
%   внутри `WSL`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
live_backend_script = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m');
start_log_path = fullfile(logs_dir, 'task_17_start_exchange_measurement.txt');
summary_log_path = fullfile(logs_dir, 'task_17_exchange_rate_measurement.txt');
csv_report_path = fullfile(reports_dir, 'task_17_exchange_rate_measurement.csv');
mat_report_path = fullfile(reports_dir, 'task_17_exchange_rate_measurement.mat');

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

cfg = uav.ardupilot.default_json_config();
cfg.update_rate_hz = 120.0;
host_ip = local_resolve_windows_host_ip(cfg);
cfg.udp_remote_ip = host_ip;
cfg.mavlink_udp_ip = host_ip;

extra_defaults_path = fullfile( ...
    repo_root, ...
    'tools', ...
    'ardupilot', ...
    'wsl', ...
    'task16_arducopter_loop_rate.parm');

local_stop_existing_sitl(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port);
start_result = local_start_sitl(cfg, host_ip, extra_defaults_path);
cleanup_obj = onCleanup(@() local_stop_existing_sitl(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port)); %#ok<NASGU>

local_write_utf8_text(start_log_path, local_make_start_log(start_result, extra_defaults_path, cfg));

if ~start_result.process_alive
    error( ...
        'uav:task17:measure_rate:SitlNotStarted', ...
        'Не удалось подтвердить запуск ArduPilot SITL. Подробности см. в %s.', ...
        start_log_path);
end

assignin('base', 'ardupilot_live_backend_duration_s', 20.0);
assignin('base', 'ardupilot_live_backend_update_rate_hz', cfg.update_rate_hz);
cleanup_duration = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
cleanup_rate = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_update_rate_hz'');')); %#ok<NASGU>

run(live_backend_script);
live_result = evalin('base', 'ardupilot_json_udp_live_backend');
metrics = uav.ardupilot.summarize_live_backend_metrics(live_result);

measurement = struct();
measurement.cfg = cfg;
measurement.host_ip = string(host_ip);
measurement.launch_command = string(start_result.command_text);
measurement.launch_status = double(start_result.status_code);
measurement.launch_pid = double(start_result.arducopter_pid);
measurement.process_alive = logical(start_result.process_alive);
measurement.start_output = string(start_result.output_text);
measurement.extra_defaults_path = string(extra_defaults_path);
measurement.metrics = metrics;
measurement.live_result = live_result;

save(mat_report_path, 'measurement');
writetable(local_make_measurement_table(measurement), csv_report_path);
local_write_utf8_text(summary_log_path, local_make_measurement_log(measurement));

assignin('base', 'task_17_exchange_rate_measurement', measurement);

fprintf('Измерение частоты обмена ArduPilot JSON/UDP\n');
fprintf('  valid_rx_count                    : %d\n', metrics.valid_rx_count);
fprintf('  json_tx_count                     : %d\n', metrics.json_tx_count);
fprintf('  response_tx_count                 : %d\n', metrics.response_tx_count);
fprintf('  средняя частота valid_rx [Hz]     : %.4f\n', metrics.valid_rx_rate_hz);
fprintf('  средняя частота response_tx [Hz]  : %.4f\n', metrics.response_tx_rate_hz);
fprintf('  95-процентиль response_tx [s]     : %.6f\n', metrics.response_tx_period_p95_s);
fprintf('  последний frame_count             : %d\n', metrics.last_frame_count);
fprintf('  процесс arducopter жив            : %s\n', local_bool_text(start_result.process_alive));

function start_result = local_start_sitl(cfg, host_ip, extra_defaults_path)
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
    '-NoConsole -NoMap -Execute -ExtraDefaultsParmPath "%s"'], ...
    script_path, ...
    char(cfg.wsl_distro_name), ...
    char(host_ip), ...
    cfg.mavlink_udp_port, ...
    cfg.mavlink_monitor_udp_port, ...
    extra_defaults_path);

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

function table_value = local_make_measurement_table(measurement)
%LOCAL_MAKE_MEASUREMENT_TABLE Сформировать табличный отчет измерения.

metrics = measurement.metrics;
table_value = table( ...
    string(measurement.host_ip), ...
    string(measurement.extra_defaults_path), ...
    measurement.launch_pid, ...
    metrics.valid_rx_count, ...
    metrics.json_tx_count, ...
    metrics.response_tx_count, ...
    metrics.invalid_rx_count, ...
    metrics.udp_rx_datagram_count, ...
    metrics.valid_rx_rate_hz, ...
    metrics.json_tx_rate_hz, ...
    metrics.response_tx_rate_hz, ...
    metrics.valid_rx_period_mean_s, ...
    metrics.valid_rx_period_p95_s, ...
    metrics.response_tx_period_mean_s, ...
    metrics.response_tx_period_p95_s, ...
    metrics.model_step_elapsed_mean_s, ...
    metrics.json_encode_elapsed_mean_s, ...
    metrics.udp_read_elapsed_mean_s, ...
    metrics.udp_write_elapsed_mean_s, ...
    metrics.frame_count_delta, ...
    metrics.last_frame_count, ...
    metrics.last_sender_address, ...
    metrics.last_sender_port, ...
    metrics.last_magic, ...
    string(local_format_vector(metrics.last_pwm_us)), ...
    string(local_format_vector(metrics.last_motor_cmd_radps)), ...
    metrics.quat_true_finite, ...
    metrics.quat_est_finite, ...
    metrics.processed_sample_count, ...
    metrics.last_exchange_status, ...
    'VariableNames', { ...
        'host_ip', ...
        'extra_defaults_path', ...
        'arducopter_pid', ...
        'valid_rx_count', ...
        'json_tx_count', ...
        'response_tx_count', ...
        'invalid_rx_count', ...
        'udp_rx_datagram_count', ...
        'valid_rx_rate_hz', ...
        'json_tx_rate_hz', ...
        'response_tx_rate_hz', ...
        'valid_rx_period_mean_s', ...
        'valid_rx_period_p95_s', ...
        'response_tx_period_mean_s', ...
        'response_tx_period_p95_s', ...
        'model_step_elapsed_mean_s', ...
        'json_encode_elapsed_mean_s', ...
        'udp_read_elapsed_mean_s', ...
        'udp_write_elapsed_mean_s', ...
        'frame_count_delta', ...
        'last_frame_count', ...
        'last_sender_address', ...
        'last_sender_port', ...
        'last_magic', ...
        'last_pwm_us', ...
        'last_motor_cmd_radps', ...
        'quat_true_finite', ...
        'quat_est_finite', ...
        'processed_sample_count', ...
        'last_exchange_status'});
end

function text_value = local_make_start_log(start_result, extra_defaults_path, cfg)
%LOCAL_MAKE_START_LOG Построить журнал запуска SITL.

lines = strings(0, 1);
lines(end + 1, 1) = "Запуск ArduPilot SITL для измерения частоты обмена TASK-17";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "команда запуска                   : " + start_result.command_text;
lines(end + 1, 1) = "дополнительный файл параметров    : " + string(extra_defaults_path);
lines(end + 1, 1) = "update_rate_hz MATLAB             : " + sprintf('%.0f', cfg.update_rate_hz);
lines(end + 1, 1) = "статус запуска PowerShell         : " + sprintf('%d', start_result.status_code);
lines(end + 1, 1) = "PID arducopter                    : " + sprintf('%d', start_result.arducopter_pid);
lines(end + 1, 1) = "процесс arducopter жив            : " + local_bool_text(start_result.process_alive);
lines(end + 1, 1) = "stdout/stderr средства запуска:";
output_lines = splitlines(start_result.output_text);
for idx = 1:numel(output_lines)
    one_line = strtrim(output_lines(idx));
    if strlength(one_line) > 0
        lines(end + 1, 1) = "  " + one_line; %#ok<AGROW>
    end
end
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_make_measurement_log(measurement)
%LOCAL_MAKE_MEASUREMENT_LOG Построить текстовый журнал измерения.

metrics = measurement.metrics;
lines = strings(0, 1);
lines(end + 1, 1) = "Измерение частоты обмена ArduPilot JSON/UDP";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Команда запуска ArduPilot           : " + measurement.launch_command;
lines(end + 1, 1) = "Дополнительный файл параметров      : " + measurement.extra_defaults_path;
lines(end + 1, 1) = "Целевой update_rate_hz MATLAB       : " + sprintf('%.0f', measurement.cfg.update_rate_hz);
lines(end + 1, 1) = "PID arducopter                      : " + sprintf('%d', measurement.launch_pid);
lines(end + 1, 1) = "Процесс arducopter жив              : " + local_bool_text(measurement.process_alive);
lines(end + 1, 1) = "valid_rx_count                      : " + sprintf('%d', metrics.valid_rx_count);
lines(end + 1, 1) = "json_tx_count                       : " + sprintf('%d', metrics.json_tx_count);
lines(end + 1, 1) = "response_tx_count                   : " + sprintf('%d', metrics.response_tx_count);
lines(end + 1, 1) = "invalid_rx_count                    : " + sprintf('%d', metrics.invalid_rx_count);
lines(end + 1, 1) = "udp_rx_datagram_count               : " + sprintf('%d', metrics.udp_rx_datagram_count);
lines(end + 1, 1) = "Средняя частота valid_rx [Hz]       : " + sprintf('%.4f', metrics.valid_rx_rate_hz);
lines(end + 1, 1) = "Средняя частота json_tx [Hz]        : " + sprintf('%.4f', metrics.json_tx_rate_hz);
lines(end + 1, 1) = "Средняя частота response_tx [Hz]    : " + sprintf('%.4f', metrics.response_tx_rate_hz);
lines(end + 1, 1) = "Средний период valid_rx [s]         : " + sprintf('%.6f', metrics.valid_rx_period_mean_s);
lines(end + 1, 1) = "95-процентиль valid_rx [s]          : " + sprintf('%.6f', metrics.valid_rx_period_p95_s);
lines(end + 1, 1) = "Средний период response_tx [s]      : " + sprintf('%.6f', metrics.response_tx_period_mean_s);
lines(end + 1, 1) = "95-процентиль response_tx [s]       : " + sprintf('%.6f', metrics.response_tx_period_p95_s);
lines(end + 1, 1) = "Средний период json_tx [s]          : " + sprintf('%.6f', metrics.json_tx_period_mean_s);
lines(end + 1, 1) = "95-процентиль json_tx [s]           : " + sprintf('%.6f', metrics.json_tx_period_p95_s);
lines(end + 1, 1) = "Среднее время шага модели [s]       : " + sprintf('%.8f', metrics.model_step_elapsed_mean_s);
lines(end + 1, 1) = "Среднее время кодирования JSON [s]  : " + sprintf('%.8f', metrics.json_encode_elapsed_mean_s);
lines(end + 1, 1) = "Среднее время чтения UDP [s]        : " + sprintf('%.8f', metrics.udp_read_elapsed_mean_s);
lines(end + 1, 1) = "Среднее время записи UDP [s]        : " + sprintf('%.8f', metrics.udp_write_elapsed_mean_s);
lines(end + 1, 1) = "Приращение frame_count              : " + sprintf('%d', metrics.frame_count_delta);
lines(end + 1, 1) = "Последний frame_count               : " + sprintf('%d', metrics.last_frame_count);
lines(end + 1, 1) = "Кватернион истинного состояния      : " + local_bool_text(metrics.quat_true_finite);
lines(end + 1, 1) = "Кватернион оценивания               : " + local_bool_text(metrics.quat_est_finite);
lines(end + 1, 1) = "Итоговый статус обмена              : " + metrics.last_exchange_status;
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_format_vector(vec_value)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку вектора.

text_value = sprintf('%.6f ', vec_value(:));
text_value = strtrim(text_value);
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
        'uav:task17:measure_rate:OpenFile', ...
        'Не удалось открыть файл %s.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end
