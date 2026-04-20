%% DIAGNOSE_ARDUPILOT_JSON_SITL_CRASH Диагностика управляемого запуска ArduPilot SITL.
% Назначение:
%   Останавливает предыдущий диагностический экземпляр `arducopter`,
%   запускает новый экземпляр в наблюдаемом режиме с сохранением stdout/stderr
%   в журнал, затем выполняет:
%   - ожидание первого двоичного пакета;
%   - единичную ответную передачу `JSON`;
%   - 20-секундный расчетный обмен MATLAB-модели с `ArduPilot`.
%   После этого сценарий сохраняет:
%   - PID процесса `arducopter`;
%   - код завершения, если процесс завершился;
%   - последние строки журнала `ArduPilot`;
%   - первые пять фактически отправленных строк `JSON`;
%   - итоговые счетчики приема и передачи.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_sitl_diagnostics - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время - секунды;
%   порты - целые числа;
%   длины пакетов - байты
%
% Допущения:
%   Сценарий предназначен только для локальной диагностики стенда на Windows
%   и WSL. Искусственный пакет `ArduPilot` не используется.

repo_root = fileparts(fileparts(mfilename('fullpath')));

cfg = uav.ardupilot.default_json_config();
cfg.udp_remote_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
cfg.mavlink_udp_ip = cfg.udp_remote_ip;
cfg.extra_defaults_path = local_resolve_task16_defaults_path(repo_root);

diag_log_path = fullfile(repo_root, 'artifacts', 'logs', 'task_15_sitl_foreground_diagnostics.txt');
json_frames_log_path = fullfile(repo_root, 'artifacts', 'logs', 'task_15_first_json_frames.txt');
wait_log_path = fullfile(repo_root, 'artifacts', 'logs', 'task_15_wait_for_packet_real.txt');
handshake_log_path = fullfile(repo_root, 'artifacts', 'logs', 'task_15_handshake_real.txt');
live_log_path = fullfile(repo_root, 'artifacts', 'logs', 'task_15_live_backend_20s_after_diagnostics.txt');
wait_script_path = fullfile(repo_root, 'scripts', 'run_ardupilot_wait_for_packet.m');
handshake_script_path = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_handshake.m');
live_script_path = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m');

wsl_diag_log = '/home/oaleg/task15_diag_arducopter.log';
wsl_pid_file = '/home/oaleg/task15_diag_arducopter.pid';
wsl_exit_file = '/home/oaleg/task15_diag_arducopter.exit';
wsl_wrapper_file = '/home/oaleg/task15_diag_arducopter.wrapper';
legacy_wsl_pid_file = '/home/oaleg/task15_diag.pid';
legacy_wsl_exit_file = '/home/oaleg/task15_diag.exit';
legacy_wsl_wrapper_file = '/home/oaleg/task15_diag.wrapper';

local_stop_previous_diag_process(cfg.wsl_distro_name, wsl_pid_file, wsl_exit_file, wsl_wrapper_file);
local_stop_previous_diag_process(cfg.wsl_distro_name, legacy_wsl_pid_file, legacy_wsl_exit_file, legacy_wsl_wrapper_file);

launch_info = local_build_launch_info(cfg, wsl_diag_log, wsl_pid_file, wsl_exit_file, wsl_wrapper_file);
launch_result = local_launch_ardupilot(cfg.wsl_distro_name, launch_info);

pause(3.0);

proc_after_launch = local_read_process_state(cfg.wsl_distro_name, wsl_pid_file, wsl_exit_file);

wait_output_text = local_run_repo_script(wait_script_path);
local_write_utf8_text(wait_log_path, wait_output_text);
wait_result = evalin('base', 'ardupilot_wait_for_packet');

handshake_output_text = local_run_repo_script(handshake_script_path);
local_write_utf8_text(handshake_log_path, handshake_output_text);
handshake_result = evalin('base', 'ardupilot_json_udp_handshake');

proc_after_handshake = local_read_process_state(cfg.wsl_distro_name, wsl_pid_file, wsl_exit_file);

live_output_text = local_run_repo_script(live_script_path);
local_write_utf8_text(live_log_path, live_output_text);
live_result = evalin('base', 'ardupilot_json_udp_live_backend');

proc_after_live = local_read_process_state(cfg.wsl_distro_name, wsl_pid_file, wsl_exit_file);

log_tail = local_read_log_tail(cfg.wsl_distro_name, wsl_diag_log, 100);
pattern_hits = local_collect_pattern_hits(log_tail);

[json_frame_log_text, json_frame_diag] = local_build_json_frame_log(live_result);
local_write_utf8_text(json_frames_log_path, json_frame_log_text);

result = struct();
result.cfg = cfg;
result.launch = launch_result;
result.process_after_launch = proc_after_launch;
result.process_after_handshake = proc_after_handshake;
result.process_after_live = proc_after_live;
result.wait_result = wait_result;
result.handshake_result = handshake_result;
result.live_result = live_result;
result.json_frame_diag = json_frame_diag;
result.log_tail = log_tail;
result.pattern_hits = pattern_hits;

assignin('base', 'ardupilot_sitl_diagnostics', result);

diag_lines = strings(0, 1);
diag_lines(end + 1, 1) = "Диагностика управляемого запуска ArduPilot SITL";
diag_lines(end + 1, 1) = "============================================================";
diag_lines(end + 1, 1) = "Windows IP для WSL2              : " + string(cfg.udp_remote_ip);
diag_lines(end + 1, 1) = "Команда запуска ArduPilot SITL   : " + string(launch_info.command_text);
diag_lines(end + 1, 1) = "PID оболочки wsl.exe             : " + string(launch_result.wrapper_pid);
diag_lines(end + 1, 1) = "PID процесса arducopter          : " + string(proc_after_launch.pid);
diag_lines(end + 1, 1) = "Процесс жив после запуска        : " + local_bool_text(proc_after_launch.is_alive);
diag_lines(end + 1, 1) = "Процесс жив после handshake      : " + local_bool_text(proc_after_handshake.is_alive);
diag_lines(end + 1, 1) = "Процесс жив после 20 секунд      : " + local_bool_text(proc_after_live.is_alive);
diag_lines(end + 1, 1) = "Код завершения процесса          : " + string(proc_after_live.exit_code_text);
diag_lines(end + 1, 1) = "Первый пакет получен             : " + local_bool_text(wait_result.wait_result.received);
diag_lines(end + 1, 1) = "Magic первого пакета             : " + string(wait_result.wait_result.sitl_output.magic);
diag_lines(end + 1, 1) = "Единичная ответная передача      : " + local_bool_text(handshake_result.reply_json_sent);
diag_lines(end + 1, 1) = "valid_rx_count за 20 секунд      : " + string(live_result.valid_rx_count);
diag_lines(end + 1, 1) = "json_tx_count за 20 секунд       : " + string(live_result.json_tx_count);
diag_lines(end + 1, 1) = "response_tx_count за 20 секунд   : " + string(live_result.response_tx_count);
diag_lines(end + 1, 1) = "last_frame_count                 : " + string(live_result.last_frame_count);
diag_lines(end + 1, 1) = "last_exchange_status             : " + string(live_result.last_exchange_status);
diag_lines(end + 1, 1) = "NaN/Inf в первых JSON-кадрах     : " + local_bool_text(json_frame_diag.has_nan_or_inf);
diag_lines(end + 1, 1) = "timestamp монотонен              : " + local_bool_text(json_frame_diag.is_timestamp_monotonic);
diag_lines(end + 1, 1) = "Признаки missing field           : " + local_bool_text(pattern_hits.missing_field);
diag_lines(end + 1, 1) = "Признаки invalid JSON            : " + local_bool_text(pattern_hits.invalid_json);
diag_lines(end + 1, 1) = "Признаки panic/assert            : " + local_bool_text(pattern_hits.fatal_error);
diag_lines(end + 1, 1) = "";
diag_lines(end + 1, 1) = "Последние строки журнала ArduPilot:";
diag_lines = [diag_lines; log_tail(:)];

diag_text = strjoin(diag_lines, newline) + newline;
local_write_utf8_text(diag_log_path, diag_text);

fprintf('%s', diag_text);

function cfg_ip = local_resolve_windows_host_ip(distro_name, fallback_ip)
%LOCAL_RESOLVE_WINDOWS_HOST_IP Определить адрес Windows-хоста для WSL2.

status_command = sprintf( ...
    'wsl -d %s -- bash -lc "ip -4 route list default | cut -d'' '' -f3 | head -n 1"', ...
    char(distro_name));

[status_code, raw_output] = system(status_command);
raw_output = strtrim(string(raw_output));

if status_code == 0 && strlength(raw_output) > 0
    cfg_ip = raw_output;
    return;
end

cfg_ip = string(fallback_ip);
end

function defaults_path_wsl = local_resolve_task16_defaults_path(repo_root)
%LOCAL_RESOLVE_TASK16_DEFAULTS_PATH Определить путь к дополнительному файлу параметров TASK-16.

defaults_path_win = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task16_arducopter_loop_rate.parm');

if ~isfile(defaults_path_win)
    defaults_path_wsl = "";
    return;
end

defaults_path_win = string(java.io.File(char(defaults_path_win)).getCanonicalPath());
drive_letter = lower(extractBefore(defaults_path_win, ":\"));
tail_value = extractAfter(defaults_path_win, ":\");
tail_value = replace(tail_value, "\", "/");

if startsWith(tail_value, "/")
    tail_value = extractAfter(tail_value, 1);
end

defaults_path_wsl = "/mnt/" + drive_letter + "/" + tail_value;
end

function local_stop_previous_diag_process(distro_name, pid_file, exit_file, wrapper_file)
%LOCAL_STOP_PREVIOUS_DIAG_PROCESS Остановить предыдущий диагностический процесс.

bash_command = sprintf([ ...
    'if [ -f %s ]; then ' ...
    'pid=$(cat %s 2>/dev/null || true); ' ...
    'if [ -n "$pid" ] && ps -p "$pid" >/dev/null 2>&1; then ' ...
    'kill -TERM "$pid" >/dev/null 2>&1 || true; sleep 2; ' ...
    'if ps -p "$pid" >/dev/null 2>&1; then kill -KILL "$pid" >/dev/null 2>&1 || true; fi; ' ...
    'fi; fi; rm -f %s %s %s'], ...
    pid_file, ...
    pid_file, ...
    pid_file, ...
    exit_file, ...
    wrapper_file);

local_wsl_command(distro_name, bash_command); %#ok<NASGU>
end

function launch_info = local_build_launch_info(cfg, wsl_log, wsl_pid, wsl_exit, wsl_wrapper)
%LOCAL_BUILD_LAUNCH_INFO Сформировать команду запуска под наблюдением.

defaults_arg = "../Tools/autotest/default_params/copter.parm";
if isfield(cfg, 'extra_defaults_path') && strlength(cfg.extra_defaults_path) > 0
    defaults_arg = defaults_arg + "," + cfg.extra_defaults_path;
end

arducopter_command = sprintf([ ...
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
    char(defaults_arg), ...
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
    wsl_pid, ...
    wsl_exit, ...
    wsl_log, ...
    wsl_wrapper, ...
    char(cfg.ardupilot_root), ...
    char(cfg.udp_remote_ip), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_udp_port), ...
    char(defaults_arg), ...
    char(cfg.udp_remote_ip), ...
    wsl_log, ...
    wsl_pid, ...
    wsl_exit);

launch_info = struct();
launch_info.command_text = string(arducopter_command);
launch_info.supervisor_script = string(supervisor_script);
launch_info.wsl_pid_file = string(wsl_pid);
launch_info.wsl_exit_file = string(wsl_exit);
launch_info.wsl_log_file = string(wsl_log);
launch_info.wsl_wrapper_file = string(wsl_wrapper);
end

function launch_result = local_launch_ardupilot(distro_name, launch_info)
%LOCAL_LAUNCH_ARDUPILOT Запустить ArduPilot SITL через временный PowerShell-файл.

tmp_ps1 = fullfile(tempdir, 'task15_launch_ardupilot_diag.ps1');

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
launch_result.status_code = status_code;
launch_result.output_text = string(output_text);
launch_result.wrapper_pid = local_extract_wrapper_pid(output_text);
end

function wrapper_pid = local_extract_wrapper_pid(output_text)
%LOCAL_EXTRACT_WRAPPER_PID Выделить PID процесса wsl.exe из вывода PowerShell.

token = regexp(char(output_text), 'WSL_WRAPPER_PID=(\d+)', 'tokens', 'once');
if isempty(token)
    wrapper_pid = 0;
else
    wrapper_pid = str2double(token{1});
end
end

function state = local_read_process_state(distro_name, pid_file, exit_file)
%LOCAL_READ_PROCESS_STATE Прочитать состояние процесса arducopter в WSL.

pid_text = local_wsl_command(distro_name, sprintf('cat %s 2>/dev/null || true', pid_file));
pid_text = strtrim(pid_text);

state = struct();
state.pid = 0;
state.is_alive = false;
state.ps_line = "";
state.exit_code_text = "";

if strlength(pid_text) == 0
    exit_text = local_wsl_command(distro_name, sprintf('cat %s 2>/dev/null || true', exit_file));
    state.exit_code_text = strtrim(string(exit_text));
    return;
end

state.pid = str2double(pid_text);

ps_command = sprintf( ...
    'ps -p %d -o pid=,ppid=,comm=,etimes=,args= 2>/dev/null || true', ...
    state.pid);
ps_text = strtrim(local_wsl_command(distro_name, ps_command));
state.ps_line = string(ps_text);
state.is_alive = strlength(state.ps_line) > 0;

exit_text = local_wsl_command(distro_name, sprintf('cat %s 2>/dev/null || true', exit_file));
exit_text = strtrim(string(exit_text));
if strlength(exit_text) == 0
    state.exit_code_text = "не зафиксирован";
else
    state.exit_code_text = exit_text;
end
end

function text_value = local_read_log_tail(distro_name, log_path, line_count)
%LOCAL_READ_LOG_TAIL Прочитать хвост журнала ArduPilot из WSL.

tail_text = local_wsl_command( ...
    distro_name, ...
    sprintf('tail -n %d %s 2>/dev/null || true', line_count, log_path));

tail_lines = splitlines(string(tail_text));
tail_lines = tail_lines(~cellfun(@isempty, cellstr(tail_lines)));
text_value = tail_lines;
end

function pattern_hits = local_collect_pattern_hits(log_tail)
%LOCAL_COLLECT_PATTERN_HITS Проверить наличие ключевых диагностических признаков.

full_text = lower(join(string(log_tail), newline));

pattern_hits = struct();
pattern_hits.missing_field = contains(full_text, "missing field") || contains(full_text, "did not contain all mandatory fields");
pattern_hits.invalid_json = contains(full_text, "invalid json");
pattern_hits.connection_timeout = contains(full_text, "timeout");
pattern_hits.packet_received = contains(full_text, "json received");
pattern_hits.fatal_error = ...
    contains(full_text, "panic") || ...
    contains(full_text, "segmentation fault") || ...
    contains(full_text, "assert");
pattern_hits.ekf = contains(full_text, "ekf");
pattern_hits.scheduler = contains(full_text, "scheduler");
pattern_hits.lockstep = contains(full_text, "lockstep");
pattern_hits.time_sync = contains(full_text, "time_sync");
end

function [log_text, frame_diag] = local_build_json_frame_log(live_result)
%LOCAL_BUILD_JSON_FRAME_LOG Сохранить первые фактически отправленные JSON-кадры.

json_series = string(live_result.json_text(:));
json_series = json_series(strlength(json_series) > 0);

frame_count = min(5, numel(json_series));
frames = json_series(1:frame_count);

timestamps = nan(frame_count, 1);
frame_diag = struct();
frame_diag.frame_count = frame_count;
frame_diag.has_nan_or_inf = false;
frame_diag.has_required_fields = true;
frame_diag.is_timestamp_monotonic = true;
frame_diag.last_quaternion_norm = nan;

lines = strings(0, 1);
lines(end + 1, 1) = "Первые фактически отправленные строки JSON";
lines(end + 1, 1) = "============================================================";

for idx = 1:frame_count
    raw_frame = frames(idx);
    trimmed_frame = strip(raw_frame, 'both', newline);
    lines(end + 1, 1) = "Кадр " + string(idx);
    lines(end + 1, 1) = "------------------------------------------------------------";
    lines(end + 1, 1) = "Заканчивается переводом строки : " + local_bool_text(endsWith(raw_frame, newline));
    lines(end + 1, 1) = "Содержит NaN/Inf в строке       : " + local_bool_text(contains(raw_frame, "NaN") || contains(raw_frame, "Inf"));
    lines(end + 1, 1) = trimmed_frame;
    lines(end + 1, 1) = "";

    frame_diag.has_nan_or_inf = frame_diag.has_nan_or_inf || contains(raw_frame, "NaN") || contains(raw_frame, "Inf");

    decoded = jsondecode(char(trimmed_frame));
    local_assert_required_fields(decoded);
    timestamps(idx) = double(decoded.timestamp);
    frame_diag.last_quaternion_norm = norm(double(decoded.quaternion(:)));
end

if frame_count > 1
    frame_diag.is_timestamp_monotonic = all(diff(timestamps) >= 0);
end

log_text = strjoin(lines, newline) + newline;
end

function local_assert_required_fields(decoded)
%LOCAL_ASSERT_REQUIRED_FIELDS Проверить обязательные поля JSON-кадра.

required_top = {'timestamp', 'imu', 'position', 'velocity', 'quaternion'};
for idx = 1:numel(required_top)
    if ~isfield(decoded, required_top{idx})
        error( ...
            'uav:ardupilot:diagnose_ardupilot_json_sitl_crash:MissingField', ...
            'В JSON-кадре отсутствует поле %s.', ...
            required_top{idx});
    end
end

if ~isfield(decoded.imu, 'gyro') || ~isfield(decoded.imu, 'accel_body')
    error( ...
        'uav:ardupilot:diagnose_ardupilot_json_sitl_crash:MissingImuField', ...
        'В JSON-кадре отсутствуют обязательные поля imu.gyro или imu.accel_body.');
end

numeric_vectors = { ...
    double(decoded.imu.gyro(:)); ...
    double(decoded.imu.accel_body(:)); ...
    double(decoded.position(:)); ...
    double(decoded.velocity(:)); ...
    double(decoded.quaternion(:))};

for idx = 1:numel(numeric_vectors)
    vec_value = numeric_vectors{idx};
    if any(~isfinite(vec_value))
        error( ...
            'uav:ardupilot:diagnose_ardupilot_json_sitl_crash:NonFiniteJson', ...
            'В JSON-кадре обнаружено нечисловое или бесконечное значение.');
    end
end
end

function output_text = local_wsl_command(distro_name, bash_command)
%LOCAL_WSL_COMMAND Выполнить команду bash в выбранном дистрибутиве WSL.

[~, raw_output] = system(sprintf( ...
    'wsl -d %s -- bash -lc "%s"', ...
    char(distro_name), ...
    strrep(char(bash_command), '"', '\"')));
output_text = string(raw_output);
end

function output_text = local_run_repo_script(script_abs_path)
%LOCAL_RUN_REPO_SCRIPT Выполнить MATLAB-сценарий по абсолютному пути.

script_abs_path = strrep(script_abs_path, '\', '/');
output_text = evalc(sprintf('run(''%s'');', script_abs_path));
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

folder_path = fileparts(path_value);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:ardupilot:diagnose_ardupilot_json_sitl_crash:LogOpen', ...
        'Не удалось открыть файл %s.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(string(text_value)));
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
