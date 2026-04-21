%% RUN_ARDUPILOT_TASK15_BASELINE_EXCHANGE Восстановить рабочий обмен TASK-15.
% Назначение:
%   Останавливает экземпляры `ArduPilot SITL`, запущенные текущим стендом,
%   затем поднимает `arducopter` прямой рабочей командой TASK-15,
%   выполняет:
%   - ожидание первого двоичного пакета;
%   - единичную ответную передачу `JSON`;
%   - 20-секундный расчетный обмен `MATLAB`-модели.
%   Сценарий сохраняет журналы, табличный отчет и MAT-файл результата.
%
% Входы:
%   none
%
% Выходы:
%   task_17_task15_baseline_exchange - структура результата в базовом
%   рабочем пространстве MATLAB
%
% Единицы измерения:
%   время - секунды;
%   частота - герцы;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   `Ubuntu` и `ArduPilot` уже установлены внутри `WSL`, а
%   `MATLAB` запущен из корня репозитория после `bootstrap_project`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

cfg = uav.ardupilot.default_json_config();
cfg.udp_remote_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
cfg.mavlink_udp_ip = cfg.udp_remote_ip;
cfg.mavlink_monitor_udp_port = 0;

diag_log_path = local_read_base_text_override( ...
    'ardupilot_task15_baseline_diag_log_path', ...
    fullfile(logs_dir, 'task_17_task15_baseline_exchange.txt'));
csv_report_path = local_read_base_text_override( ...
    'ardupilot_task15_baseline_csv_report_path', ...
    fullfile(reports_dir, 'task_17_task15_baseline_exchange.csv'));
mat_report_path = local_read_base_text_override( ...
    'ardupilot_task15_baseline_mat_report_path', ...
    fullfile(reports_dir, 'task_17_task15_baseline_exchange.mat'));
wait_log_path = local_read_base_text_override( ...
    'ardupilot_task15_baseline_wait_log_path', ...
    fullfile(logs_dir, 'task_17_task15_baseline_wait_for_packet.txt'));
handshake_log_path = local_read_base_text_override( ...
    'ardupilot_task15_baseline_handshake_log_path', ...
    fullfile(logs_dir, 'task_17_task15_baseline_handshake.txt'));
live_log_path = local_read_base_text_override( ...
    'ardupilot_task15_baseline_live_log_path', ...
    fullfile(logs_dir, 'task_17_task15_baseline_live_backend.txt'));
extra_defaults_win_path = local_read_base_text_override( ...
    'ardupilot_task15_extra_defaults_win_path', ...
    "");
extra_defaults_wsl_path = "";
if strlength(string(extra_defaults_win_path)) > 0
    extra_defaults_wsl_path = string(local_windows_to_wsl_path(cfg.wsl_distro_name, extra_defaults_win_path));
end

wsl_log_path = '/home/oaleg/task17_task15_baseline_arducopter.log';
wsl_pid_path = '/home/oaleg/task17_task15_baseline_arducopter.pid';
wsl_exit_path = '/home/oaleg/task17_task15_baseline_arducopter.exit';
wsl_wrapper_path = '/home/oaleg/task17_task15_baseline_arducopter.wrapper';

wait_script_path = fullfile(repo_root, 'scripts', 'run_ardupilot_wait_for_packet.m');
handshake_script_path = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_handshake.m');
live_script_path = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m');

evalin('base', ['clear(' ...
    '''ardupilot_live_backend_duration_s'', ' ...
    '''ardupilot_live_backend_update_rate_hz'', ' ...
    '''ardupilot_arm_attempt_delay_s'', ' ...
    '''ardupilot_wait_for_packet'', ' ...
    '''ardupilot_json_udp_handshake'', ' ...
    '''ardupilot_json_udp_live_backend'');']);

local_stop_existing_sitl(cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port);
local_remove_wsl_files(cfg.wsl_distro_name, sprintf( ...
    '%s %s %s %s', ...
    wsl_log_path, ...
    wsl_pid_path, ...
    wsl_exit_path, ...
    wsl_wrapper_path));

launch_info = local_build_launch_info(cfg, wsl_log_path, wsl_pid_path, wsl_exit_path, wsl_wrapper_path, extra_defaults_wsl_path);
launch_result = local_launch_ardupilot(cfg.wsl_distro_name, launch_info);

pause(3.0);

process_after_launch = local_read_process_state(cfg.wsl_distro_name, wsl_pid_path, wsl_exit_path);

wait_output_text = local_run_repo_script(wait_script_path);
local_write_utf8_text(wait_log_path, wait_output_text);
wait_result = evalin('base', 'ardupilot_wait_for_packet');

handshake_output_text = local_run_repo_script(handshake_script_path);
local_write_utf8_text(handshake_log_path, handshake_output_text);
handshake_result = evalin('base', 'ardupilot_json_udp_handshake');

process_after_handshake = local_read_process_state(cfg.wsl_distro_name, wsl_pid_path, wsl_exit_path);

run(live_script_path);
live_result = evalin('base', 'ardupilot_json_udp_live_backend');
metrics = uav.ardupilot.summarize_live_backend_metrics(live_result);

live_output_text = evalc('local_print_live_summary(live_result);');
local_write_utf8_text(live_log_path, live_output_text);

process_after_live = local_read_process_state(cfg.wsl_distro_name, wsl_pid_path, wsl_exit_path);
log_tail = local_read_log_tail(cfg.wsl_distro_name, wsl_log_path, 100);

baseline_restored = metrics.valid_rx_count > 50 ...
    && metrics.response_tx_count > 50 ...
    && metrics.json_tx_count > 50 ...
    && metrics.last_frame_count > 0 ...
    && process_after_live.is_alive;

first_failure_reason = local_pick_first_failure_reason( ...
    launch_result, ...
    process_after_launch, ...
    wait_result, ...
    handshake_result, ...
    metrics, ...
    process_after_live, ...
    log_tail);

result = struct();
result.cfg = cfg;
result.launch = launch_result;
result.launch_command = string(launch_info.command_text);
result.process_after_launch = process_after_launch;
result.process_after_handshake = process_after_handshake;
result.process_after_live = process_after_live;
result.wait_result = wait_result;
result.handshake_result = handshake_result;
result.live_result = live_result;
result.metrics = metrics;
result.log_tail = log_tail;
result.baseline_restored = logical(baseline_restored);
result.first_failure_reason = string(first_failure_reason);
result.diag_log_path = string(diag_log_path);
result.csv_report_path = string(csv_report_path);
result.mat_report_path = string(mat_report_path);
result.extra_defaults_win_path = string(extra_defaults_win_path);
result.extra_defaults_wsl_path = string(extra_defaults_wsl_path);

save(mat_report_path, 'result');
writetable(local_make_result_table(result), csv_report_path);
local_write_utf8_text(diag_log_path, local_make_diag_text(result));

assignin('base', 'task_17_task15_baseline_exchange', result);

fprintf('%s', local_make_console_summary(result));

function launch_info = local_build_launch_info(cfg, wsl_log_path, wsl_pid_path, wsl_exit_path, wsl_wrapper_path, extra_defaults_wsl_path)
%LOCAL_BUILD_LAUNCH_INFO Сформировать команду прямого запуска TASK-15.

defaults_argument = "../Tools/autotest/default_params/copter.parm";
if strlength(extra_defaults_wsl_path) > 0
    defaults_argument = defaults_argument + "," + extra_defaults_wsl_path;
end

command_text = sprintf([ ...
    'cd %s/ArduCopter && ' ...
    '../build/sitl/bin/arducopter ' ...
    '--model JSON:%s ' ...
    '--speedup 1 ' ...
    '--slave 0 ' ...
    '--serial0=udpclient:%s:%d ' ...
    '--defaults %s ' ...
    '--sim-address=%s ' ...
    '-I0'], ...
    char(cfg.ardupilot_root), ...
    char(cfg.udp_remote_ip), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_udp_port), ...
    char(defaults_argument), ...
    char(cfg.udp_remote_ip));

supervisor_script = sprintf([ ...
    'set -euo pipefail\n' ...
    'rm -f %s %s %s %s\n' ...
    'cd %s/ArduCopter\n' ...
    '(\n' ...
    '  ../build/sitl/bin/arducopter \\\n' ...
    '    --model JSON:%s \\\n' ...
    '    --speedup 1 \\\n' ...
    '    --slave 0 \\\n' ...
    '    --serial0=udpclient:%s:%d \\\n' ...
    '    --defaults %s \\\n' ...
    '    --sim-address=%s \\\n' ...
    '    -I0\n' ...
    ') >>%s 2>&1 &\n' ...
    'pid=$!\n' ...
    'echo $pid >%s\n' ...
    'wait $pid\n' ...
    'code=$?\n' ...
    'echo $code >%s\n'], ...
    wsl_pid_path, ...
    wsl_exit_path, ...
    wsl_log_path, ...
    wsl_wrapper_path, ...
    char(cfg.ardupilot_root), ...
    char(cfg.udp_remote_ip), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_udp_port), ...
    char(defaults_argument), ...
    char(cfg.udp_remote_ip), ...
    wsl_log_path, ...
    wsl_pid_path, ...
    wsl_exit_path);

launch_info = struct();
launch_info.command_text = string(command_text);
launch_info.supervisor_script = string(supervisor_script);
end

function launch_result = local_launch_ardupilot(distro_name, launch_info)
%LOCAL_LAUNCH_ARDUPILOT Запустить ArduPilot через временный PowerShell-файл.

tmp_ps1 = fullfile(tempdir, 'task17_launch_task15_baseline.ps1');
cleanup_obj = onCleanup(@() local_delete_if_exists(tmp_ps1)); %#ok<NASGU>

ps_lines = strings(0, 1);
ps_lines(end + 1, 1) = "$wslCommand = @'";
ps_lines = [ps_lines; splitlines(launch_info.supervisor_script)];
ps_lines(end + 1, 1) = "'@";
ps_lines(end + 1, 1) = "$p = Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d','" + string(distro_name) + "','--','bash','-lc',$wslCommand) -WindowStyle Hidden -PassThru";
ps_lines(end + 1, 1) = "Write-Output ('WSL_WRAPPER_PID={0}' -f $p.Id)";
local_write_utf8_text(tmp_ps1, strjoin(ps_lines, newline) + newline);

[status_code, output_text] = system(sprintf( ...
    'powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', ...
    tmp_ps1));

launch_result = struct();
launch_result.status_code = double(status_code);
launch_result.output_text = string(output_text);
launch_result.wrapper_pid = local_extract_wrapper_pid(output_text);
end

function wrapper_pid = local_extract_wrapper_pid(output_text)
%LOCAL_EXTRACT_WRAPPER_PID Выделить PID процесса wsl.exe.

token = regexp(char(output_text), 'WSL_WRAPPER_PID=(\d+)', 'tokens', 'once');
if isempty(token)
    wrapper_pid = 0;
else
    wrapper_pid = str2double(token{1});
end
end

function state = local_read_process_state(distro_name, wsl_pid_path, wsl_exit_path)
%LOCAL_READ_PROCESS_STATE Прочитать состояние процесса arducopter.

pid_text = strtrim(local_wsl_command(distro_name, sprintf('cat %s 2>/dev/null || true', wsl_pid_path)));
exit_text = strtrim(local_wsl_command(distro_name, sprintf('cat %s 2>/dev/null || true', wsl_exit_path)));

state = struct();
state.pid = 0;
state.is_alive = false;
state.ps_line = "";
state.exit_code_text = string(exit_text);

if strlength(pid_text) == 0
    return;
end

pid_value = str2double(pid_text);
if ~isfinite(pid_value)
    return;
end

state.pid = double(pid_value);
ps_line = strtrim(local_wsl_command(distro_name, sprintf( ...
    'ps -p %d -o pid=,ppid=,comm=,etimes=,args= 2>/dev/null || true', ...
    round(pid_value))));
state.ps_line = string(ps_line);
state.is_alive = strlength(state.ps_line) > 0;
end

function local_stop_existing_sitl(distro_name, host_ip, mavlink_port)
%LOCAL_STOP_EXISTING_SITL Остановить только экземпляры текущего стенда.

pattern = sprintf( ...
    'build/sitl/bin/arducopter.*JSON:%s.*udpclient:%s:%d', ...
    char(host_ip), ...
    char(host_ip), ...
    mavlink_port);
local_wsl_command(distro_name, sprintf('pkill -f ''%s'' >/dev/null 2>&1 || true', pattern)); %#ok<NASGU>
pause(1.0);
end

function local_remove_wsl_files(distro_name, files_expr)
%LOCAL_REMOVE_WSL_FILES Удалить временные файлы наблюдаемого запуска.

local_wsl_command(distro_name, "rm -f " + string(files_expr) + " >/dev/null 2>&1 || true"); %#ok<NASGU>
end

function value = local_read_base_text_override(var_name, default_value)
%LOCAL_READ_BASE_TEXT_OVERRIDE Прочитать текстовое переопределение из base workspace.

value = char(string(default_value));
if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    override_value = evalin('base', var_name);
    if ~isempty(override_value)
        value = char(string(override_value));
    end
end
end

function wsl_path = local_windows_to_wsl_path(distro_name, win_path)
%LOCAL_WINDOWS_TO_WSL_PATH Преобразовать путь Windows в путь WSL.

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "wslpath -a %s"', ...
    char(distro_name), ...
    local_shell_quote(win_path));
[status_code, raw_output] = system(command_text);
if status_code ~= 0
    error('uav:task17:baseline:WslPathFailed', ...
        'Не удалось преобразовать путь Windows в путь WSL: %s', ...
        win_path);
end

wsl_path = strtrim(raw_output);
end

function quoted_value = local_shell_quote(raw_value)
%LOCAL_SHELL_QUOTE Заключить строку в одинарные кавычки для bash.

quoted_value = ['''' strrep(char(raw_value), '''', '''"''"''') ''''];
end

function output_text = local_wsl_command(distro_name, bash_command)
%LOCAL_WSL_COMMAND Выполнить команду Bash внутри WSL.

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "%s"', ...
    char(distro_name), ...
    strrep(char(bash_command), '"', '\"'));
[~, output_text] = system(command_text);
output_text = string(output_text);
end

function host_ip = local_resolve_windows_host_ip(distro_name, fallback_ip)
%LOCAL_RESOLVE_WINDOWS_HOST_IP Определить адрес Windows-хоста для WSL2.

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "ip -4 route list default | cut -d'' '' -f3 | head -n 1"', ...
    char(distro_name));
[status_code, output_text] = system(command_text);

host_ip = string(fallback_ip);
if status_code == 0
    candidate = strtrim(string(output_text));
    if strlength(candidate) > 0
        host_ip = candidate;
    end
end
end

function output_text = local_run_repo_script(script_path)
%LOCAL_RUN_REPO_SCRIPT Выполнить сценарий MATLAB и вернуть текстовый вывод.

normalized_path = strrep(script_path, '\', '/');
escaped_path = strrep(normalized_path, '''', '''''');
output_text = string(evalc(sprintf('run(''%s'');', escaped_path)));
end

function log_tail = local_read_log_tail(distro_name, wsl_log_path, line_count)
%LOCAL_READ_LOG_TAIL Прочитать последние строки журнала ArduPilot.

tail_text = local_wsl_command(distro_name, sprintf( ...
    'tail -n %d %s 2>/dev/null || true', ...
    line_count, ...
    wsl_log_path));

if strlength(strtrim(tail_text)) == 0
    log_tail = strings(0, 1);
else
    log_tail = splitlines(strtrim(tail_text));
end
end

function reason = local_pick_first_failure_reason(launch_result, process_after_launch, wait_result, handshake_result, metrics, process_after_live, log_tail)
%LOCAL_PICK_FIRST_FAILURE_REASON Определить первую причину непрохождения baseline.

reason = "";

if launch_result.status_code ~= 0
    reason = "Команда запуска ArduPilot завершилась с ненулевым кодом PowerShell.";
    return;
end

if ~process_after_launch.is_alive
    reason = "После прямого запуска TASK-15 процесс arducopter не подтвердил состояние живого процесса.";
    return;
end

if ~logical(wait_result.wait_result.received)
    reason = "После запуска TASK-15 первый двоичный пакет от ArduPilot не был получен.";
    return;
end

if ~logical(handshake_result.reply_json_sent)
    reason = "После первого пакета не удалось подтвердить единичную ответную передачу JSON.";
    return;
end

if metrics.valid_rx_count <= 50
    reason = "20-секундный baseline не дал больше 50 валидных двоичных пакетов.";
    return;
end

if metrics.response_tx_count <= 50
    reason = "20-секундный baseline не дал больше 50 ответных передач JSON.";
    return;
end

if metrics.last_frame_count <= 0
    reason = "В baseline не увеличился frame_count.";
    return;
end

if ~process_after_live.is_alive
    if strlength(process_after_live.exit_code_text) > 0
        reason = "После 20 секунд arducopter завершился с кодом " + process_after_live.exit_code_text + ".";
    else
        reason = "После 20 секунд arducopter не сохранил состояние живого процесса.";
    end

    if ~isempty(log_tail)
        reason = reason + " Последняя строка журнала: " + string(log_tail(end));
    end
end
end

function table_value = local_make_result_table(result)
%LOCAL_MAKE_RESULT_TABLE Сформировать однострочный CSV-отчет baseline.

metrics = result.metrics;
table_value = table( ...
    string(result.cfg.udp_remote_ip), ...
    string(result.launch_command), ...
    double(result.process_after_launch.pid), ...
    logical(result.process_after_launch.is_alive), ...
    logical(result.process_after_live.is_alive), ...
    logical(result.wait_result.wait_result.received), ...
    double(result.wait_result.wait_result.sitl_output.magic), ...
    string(local_format_vector(result.wait_result.wait_result.sitl_output.motor_pwm_us)), ...
    logical(result.handshake_result.reply_json_sent), ...
    metrics.valid_rx_count, ...
    metrics.json_tx_count, ...
    metrics.response_tx_count, ...
    metrics.valid_rx_rate_hz, ...
    metrics.response_tx_rate_hz, ...
    metrics.valid_rx_period_mean_s, ...
    metrics.valid_rx_period_p95_s, ...
    metrics.last_frame_count, ...
    metrics.last_sender_address, ...
    metrics.last_sender_port, ...
    metrics.last_magic, ...
    string(local_format_vector(metrics.last_pwm_us)), ...
    string(local_format_vector(metrics.last_motor_cmd_radps)), ...
    metrics.quat_true_finite, ...
    metrics.quat_est_finite, ...
    string(metrics.last_exchange_status), ...
    logical(result.baseline_restored), ...
    string(result.first_failure_reason), ...
    'VariableNames', { ...
        'host_ip', ...
        'launch_command', ...
        'arducopter_pid', ...
        'process_alive_after_launch', ...
        'process_alive_after_20s', ...
        'first_packet_received', ...
        'first_packet_magic', ...
        'first_packet_pwm_us', ...
        'reply_json_sent', ...
        'valid_rx_count', ...
        'json_tx_count', ...
        'response_tx_count', ...
        'valid_rx_rate_hz', ...
        'response_tx_rate_hz', ...
        'valid_rx_period_mean_s', ...
        'valid_rx_period_p95_s', ...
        'last_frame_count', ...
        'last_sender_address', ...
        'last_sender_port', ...
        'last_magic', ...
        'last_pwm_us', ...
        'last_motor_cmd_radps', ...
        'quat_true_finite', ...
        'quat_est_finite', ...
        'last_exchange_status', ...
        'baseline_restored', ...
        'first_failure_reason'});
end

function text_value = local_make_diag_text(result)
%LOCAL_MAKE_DIAG_TEXT Сформировать подробный текстовый журнал baseline.

metrics = result.metrics;
lines = strings(0, 1);
lines(end + 1, 1) = "Восстановление рабочего 20-секундного обмена TASK-15";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Адрес Windows-хоста для WSL2       : " + string(result.cfg.udp_remote_ip);
lines(end + 1, 1) = "Команда запуска ArduPilot SITL     : " + string(result.launch_command);
lines(end + 1, 1) = "PID процесса arducopter            : " + string(result.process_after_launch.pid);
lines(end + 1, 1) = "Процесс жив после запуска          : " + local_bool_text(result.process_after_launch.is_alive);
lines(end + 1, 1) = "Процесс жив после handshake        : " + local_bool_text(result.process_after_handshake.is_alive);
lines(end + 1, 1) = "Процесс жив после 20 секунд        : " + local_bool_text(result.process_after_live.is_alive);
lines(end + 1, 1) = "Код завершения процесса            : " + string(result.process_after_live.exit_code_text);
lines(end + 1, 1) = "Первый пакет получен               : " + local_bool_text(result.wait_result.wait_result.received);
lines(end + 1, 1) = "Magic первого пакета               : " + string(result.wait_result.wait_result.sitl_output.magic);
lines(end + 1, 1) = "Первые четыре значения ШИМ [us]    : [" + local_format_vector(result.wait_result.wait_result.sitl_output.motor_pwm_us) + "]";
lines(end + 1, 1) = "Единичная ответная передача JSON   : " + local_bool_text(result.handshake_result.reply_json_sent);
lines(end + 1, 1) = "valid_rx_count                     : " + string(metrics.valid_rx_count);
lines(end + 1, 1) = "json_tx_count                      : " + string(metrics.json_tx_count);
lines(end + 1, 1) = "response_tx_count                  : " + string(metrics.response_tx_count);
lines(end + 1, 1) = "Средняя частота valid_rx [Hz]      : " + string(metrics.valid_rx_rate_hz);
lines(end + 1, 1) = "Средняя частота response_tx [Hz]   : " + string(metrics.response_tx_rate_hz);
lines(end + 1, 1) = "Средний период valid_rx [s]        : " + string(metrics.valid_rx_period_mean_s);
lines(end + 1, 1) = "95-процентиль valid_rx [s]         : " + string(metrics.valid_rx_period_p95_s);
lines(end + 1, 1) = "Последний frame_count              : " + string(metrics.last_frame_count);
lines(end + 1, 1) = "Последний sender address           : " + string(metrics.last_sender_address);
lines(end + 1, 1) = "Последний sender port              : " + string(metrics.last_sender_port);
lines(end + 1, 1) = "Последний magic                    : " + string(metrics.last_magic);
lines(end + 1, 1) = "Последние ШИМ [us]                 : [" + local_format_vector(metrics.last_pwm_us) + "]";
lines(end + 1, 1) = "Последние motor_cmd [rad/s]        : [" + local_format_vector(metrics.last_motor_cmd_radps) + "]";
lines(end + 1, 1) = "Норма истинного кватерниона конечна: " + local_bool_text(metrics.quat_true_finite);
lines(end + 1, 1) = "Норма оцененного кватерниона конечна: " + local_bool_text(metrics.quat_est_finite);
lines(end + 1, 1) = "Итоговый статус обмена             : " + string(metrics.last_exchange_status);
lines(end + 1, 1) = "Baseline TASK-15 восстановлен      : " + local_bool_text(result.baseline_restored);
lines(end + 1, 1) = "Первая причина отказа              : " + string(result.first_failure_reason);

if ~isempty(result.log_tail)
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "Последние строки журнала ArduPilot:";
    lines = [lines; string(result.log_tail(:))];
end

text_value = strjoin(lines, newline) + newline;
end

function text_value = local_make_console_summary(result)
%LOCAL_MAKE_CONSOLE_SUMMARY Сформировать краткую сводку baseline для консоли.

metrics = result.metrics;
lines = strings(0, 1);
lines(end + 1, 1) = "Восстановление рабочего обмена TASK-15";
lines(end + 1, 1) = "  baseline восстановлен            : " + local_bool_text(result.baseline_restored);
lines(end + 1, 1) = "  valid_rx_count                   : " + string(metrics.valid_rx_count);
lines(end + 1, 1) = "  json_tx_count                    : " + string(metrics.json_tx_count);
lines(end + 1, 1) = "  response_tx_count                : " + string(metrics.response_tx_count);
lines(end + 1, 1) = "  last_frame_count                 : " + string(metrics.last_frame_count);
lines(end + 1, 1) = "  процесс arducopter жив           : " + local_bool_text(result.process_after_live.is_alive);
if ~result.baseline_restored
    lines(end + 1, 1) = "  первая причина                   : " + string(result.first_failure_reason);
end
text_value = strjoin(lines, newline) + newline;
end

function local_print_live_summary(live_result)
%LOCAL_PRINT_LIVE_SUMMARY Напечатать краткий текст результата длительного обмена.

fprintf('20-секундный обмен MATLAB-модели с ArduPilot\n');
fprintf('  первый пакет получен                : %s\n', local_bool_text(live_result.first_packet_received));
fprintf('  принято UDP-датаграмм               : %d\n', live_result.rx_datagram_count);
fprintf('  принято валидных двоичных пакетов   : %d\n', live_result.valid_rx_count);
fprintf('  выполнено ответных передач          : %d\n', live_result.response_tx_count);
fprintf('  отправлено строк JSON               : %d\n', live_result.json_tx_count);
fprintf('  последний frame_count               : %d\n', live_result.last_frame_count);
fprintf('  итоговый статус                     : %s\n', char(live_result.last_exchange_status));
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку числового вектора.

vec = double(vec(:));
text_value = strtrim(sprintf('%.6f ', vec));
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

directory_path = fileparts(file_path);
if strlength(string(directory_path)) > 0 && ~isfolder(directory_path)
    mkdir(directory_path);
end

fid = fopen(file_path, 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:task17:baseline:OpenFile', ...
        'Не удалось открыть файл %s для записи.', ...
        file_path);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
text_value = string(text_value);
text_value(ismissing(text_value)) = "";
if numel(text_value) > 1
    text_value = strjoin(text_value, newline);
end
fprintf(fid, '%s', char(text_value));
end

function local_delete_if_exists(file_path)
%LOCAL_DELETE_IF_EXISTS Удалить временный файл, если он существует.

if isfile(file_path)
    delete(file_path);
end
end
