%% RUN_ARDUPILOT_INTERNAL_SITL_DEMO Запустить штатный ArduPilot SITL и демонстрацию взлета.
% Назначение:
%   Выполняет практический прогон штатного `ArduPilot SITL` без внешней
%   MATLAB-модели: поднимает `Mission Planner`, запускает внутреннюю
%   модель движения `ArduPilot`, затем через `pymavlink` выполняет
%   последовательность `GUIDED -> arm -> takeoff 5 m`.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_internal_sitl_demo - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   высота задается в метрах;
%   время задается в секундах
%
% Допущения:
%   `ArduPilot` уже установлен внутри `WSL`, а `Mission Planner` доступен
%   в среде Windows.

repo_root = fileparts(fileparts(mfilename('fullpath')));
log_start = fullfile(repo_root, 'artifacts', 'logs', 'task_18_internal_sitl_start.txt');
log_takeoff = fullfile(repo_root, 'artifacts', 'logs', 'task_18_internal_sitl_takeoff.txt');
log_mp = fullfile(repo_root, 'artifacts', 'logs', 'task_18_mission_planner_start.txt');
report_mat = fullfile(repo_root, 'artifacts', 'reports', 'task_18_internal_sitl_demo.mat');
report_csv = fullfile(repo_root, 'artifacts', 'reports', 'task_18_internal_sitl_demo.csv');
fig_alt = fullfile(repo_root, 'artifacts', 'figures', 'task_18_internal_altitude.png');
fig_mode = fullfile(repo_root, 'artifacts', 'figures', 'task_18_internal_mode_arm_state.png');
fig_mav = fullfile(repo_root, 'artifacts', 'figures', 'task_18_internal_mavlink_status.png');

local_prepare_parent(log_start);
local_prepare_parent(log_takeoff);
local_prepare_parent(log_mp);
local_prepare_parent(report_mat);
local_prepare_parent(report_csv);
local_prepare_parent(fig_alt);

cfg = uav.ardupilot.default_json_config();
host_ip = local_detect_windows_host_ip();
cfg.mavlink_udp_ip = host_ip;
cfg.mavlink_udp_port = 14550;
cfg.mavlink_monitor_udp_port = 14552;

local_cleanup_previous_internal_stand(repo_root);
mp_status = local_launch_mission_planner();
local_write_utf8_text(log_mp, mp_status + newline);

sequence_cfg = struct();
sequence_cfg.wsl_distro_name = cfg.wsl_distro_name;
sequence_cfg.connection_candidates = ["tcp:127.0.0.1:5760"; "tcp:127.0.0.1:5770"];
sequence_cfg.mode_name = "GUIDED";
sequence_cfg.wait_before_command_s = 60.0;
sequence_cfg.heartbeat_timeout_s = 20.0;
sequence_cfg.arm_timeout_s = 20.0;
sequence_cfg.monitor_duration_s = 25.0;
sequence_cfg.takeoff_alt_m = 5.0;
sequence_cfg.sample_period_s = 0.5;

command_info = uav.ardupilot.make_pymavlink_sequence_command(sequence_cfg);
cleanup_python = onCleanup(@() local_delete_if_exists(command_info.python_script_path_win)); %#ok<NASGU>
cleanup_json = onCleanup(@() local_delete_if_exists(command_info.output_json_path_win)); %#ok<NASGU>

ardupilot_log_win = fullfile(repo_root, 'artifacts', 'logs', 'task_18_internal_sitl_console.txt');
local_delete_if_exists(ardupilot_log_win);

[start_status, start_output, orchestration_info] = local_run_internal_orchestration( ...
    cfg, ...
    host_ip, ...
    ardupilot_log_win, ...
    command_info.python_script_path_wsl);

if isfile(char(command_info.output_json_path_win))
    seq_result = uav.ardupilot.read_pymavlink_sequence_result(command_info.output_json_path_win);
else
    seq_result = struct();
    seq_result.failure_reason = "Файл результата pymavlink не сформирован.";
    seq_result.telemetry_table = table();
    seq_result.status_texts = strings(0, 1);
    seq_result.arm_succeeded = false;
    seq_result.takeoff_requested = false;
    seq_result.takeoff_ack_result = nan;
    seq_result.arm_ack_result = nan;
    seq_result.max_relative_alt_m = nan;
    seq_result.height_changed = false;
    seq_result.heartbeat_received = false;
    seq_result.heartbeat_count = 0;
    seq_result.global_position_count = 0;
end

telemetry_table = local_make_internal_csv_table(seq_result);
if isempty(telemetry_table)
    telemetry_table = table( ...
        nan, false, "", nan, nan, nan, nan, ...
        'VariableNames', { ...
        'time_s', 'armed', 'mode', 'relative_alt_m', ...
        'heartbeat_count', 'global_position_count', ...
        'status_text_count'});
end

writetable(telemetry_table, report_csv);

start_lines = strings(0, 1);
start_lines(end + 1, 1) = "Запуск штатного ArduPilot SITL через WSL";
start_lines(end + 1, 1) = "Windows-хост для WSL2: " + host_ip;
start_lines(end + 1, 1) = "Канал Mission Planner: UDP " + string(cfg.mavlink_udp_port);
start_lines(end + 1, 1) = "Дополнительный канал MAVLink: UDP " + string(cfg.mavlink_monitor_udp_port);
start_lines(end + 1, 1) = "Код завершения сценария WSL: " + string(start_status);
start_lines(end + 1, 1) = "PID arducopter внутри WSL: " + orchestration_info.ardupilot_pid;
start_lines(end + 1, 1) = "ArduPilot оставался жив до завершения прогона: " ...
    + local_bool_text(orchestration_info.ardupilot_alive_before_stop);
start_lines(end + 1, 1) = "Внутренний HEARTBEAT по tcp:5760: " ...
    + local_bool_text(local_get_logical(seq_result, 'heartbeat_received'));
start_lines(end + 1, 1) = "Дополнительный UDP HEARTBEAT на порту 14552: не подтвержден отдельным слушателем.";
start_lines(end + 1, 1) = "";
start_lines(end + 1, 1) = "Вывод orchestration-сценария:";
start_lines = [start_lines; splitlines(string(start_output))]; %#ok<AGROW>
local_write_utf8_text(log_start, strjoin(start_lines, newline) + newline);

takeoff_lines = strings(0, 1);
takeoff_lines(end + 1, 1) = "Проверка arm/takeoff штатного ArduPilot SITL";
takeoff_lines(end + 1, 1) = "Команда pymavlink:";
takeoff_lines(end + 1, 1) = string(command_info.command_text);
takeoff_lines(end + 1, 1) = "";
takeoff_lines(end + 1, 1) = local_format_sequence_result(seq_result);
local_write_utf8_text(log_takeoff, strjoin(takeoff_lines, newline) + newline);

result = struct();
result.host_ip = host_ip;
result.mission_planner_status = mp_status;
result.start_status_code = double(start_status);
result.start_output = string(start_output);
result.orchestration_info = orchestration_info;
result.sequence_result = seq_result;
result.telemetry_table = telemetry_table;
result.sitl_started = local_get_logical(seq_result, 'heartbeat_received');
result.mission_planner_started = contains(mp_status, "Mission Planner запущен: да");
result.heartbeat_confirmed = local_get_logical(seq_result, 'heartbeat_received');
result.arm_succeeded = local_get_logical(seq_result, 'arm_succeeded');
result.takeoff_succeeded = local_get_logical(seq_result, 'height_changed');
result.max_relative_alt_m = local_get_numeric(seq_result, 'max_relative_alt_m');
result.ardupilot_alive_before_stop = orchestration_info.ardupilot_alive_before_stop;
result.ardupilot_pid = orchestration_info.ardupilot_pid;

save(report_mat, 'result');
assignin('base', 'ardupilot_internal_sitl_demo', result);

local_plot_internal_figures(telemetry_table, fig_alt, fig_mode, fig_mav);

fprintf('Штатный стенд ArduPilot SITL с внутренней моделью\n');
fprintf('  ArduPilot SITL запущен                  : %s\n', local_bool_text(result.sitl_started));
fprintf('  Mission Planner запущен                 : %s\n', local_bool_text(result.mission_planner_started));
fprintf('  HEARTBEAT по tcp:5760 подтвержден       : %s\n', local_bool_text(result.heartbeat_confirmed));
fprintf('  взведение выполнено                     : %s\n', local_bool_text(result.arm_succeeded));
fprintf('  команда взлета 5 м принята/высота растет: %s\n', local_bool_text(result.takeoff_succeeded));
fprintf('  максимальная относительная высота [m]   : %.3f\n', result.max_relative_alt_m);

function [status_code, output_text, info] = local_run_internal_orchestration(cfg, host_ip, ardupilot_log_win, python_script_wsl)
%LOCAL_RUN_INTERNAL_ORCHESTRATION Выполнить внутренний прогон в одном WSL-сеансе.

ardupilot_log_wsl = local_windows_to_wsl_path(ardupilot_log_win);
temp_bash_win = string(tempname) + ".sh";
temp_bash_wsl = local_windows_to_wsl_path(temp_bash_win);
bash_lines = strings(0, 1);
bash_lines(end + 1, 1) = "set -euo pipefail";
bash_lines(end + 1, 1) = "cd ~/src/ardupilot/ArduCopter";
bash_lines(end + 1, 1) = "rm -f " + local_bash_single_quote(ardupilot_log_wsl);
bash_lines(end + 1, 1) = "../build/sitl/bin/arducopter --model + --speedup 1 --defaults ../Tools/autotest/default_params/copter.parm --serial0 tcp:0 --serial1 udpclient:" ...
    + host_ip + ":" + string(cfg.mavlink_udp_port) ...
    + " --serial2 udpclient:" + host_ip + ":" + string(cfg.mavlink_monitor_udp_port) ...
    + " -I0 > " + local_bash_single_quote(ardupilot_log_wsl) + " 2>&1 &";
bash_lines(end + 1, 1) = "sleep 2";
bash_lines(end + 1, 1) = "ARDUPID=$(pgrep -n arducopter || true)";
bash_lines(end + 1, 1) = "echo arducopter_pid=${ARDUPID}";
bash_lines(end + 1, 1) = "if [ -z ""${ARDUPID}"" ]; then echo arducopter_start_failed=yes; tail -n 120 " ...
    + local_bash_single_quote(ardupilot_log_wsl) + " || true; exit 1; fi";
bash_lines(end + 1, 1) = "python3 " + local_bash_single_quote(python_script_wsl);
bash_lines(end + 1, 1) = "PY_STATUS=$?";
bash_lines(end + 1, 1) = "if ps -p ""${ARDUPID}"" >/dev/null 2>&1; then echo arducopter_alive_before_stop=yes; else echo arducopter_alive_before_stop=no; fi";
bash_lines(end + 1, 1) = "kill ""${ARDUPID}"" >/dev/null 2>&1 || true";
bash_lines(end + 1, 1) = "wait ""${ARDUPID}"" >/dev/null 2>&1 || true";
bash_lines(end + 1, 1) = "echo ---ARDUPILOT_LOG_TAIL---";
bash_lines(end + 1, 1) = "tail -n 120 " + local_bash_single_quote(ardupilot_log_wsl) + " || true";
bash_lines(end + 1, 1) = "exit ${PY_STATUS}";

bash_payload = strjoin(bash_lines, newline);
local_write_utf8_text(temp_bash_win, bash_payload + newline);
cleanup_bash = onCleanup(@() local_delete_if_exists(temp_bash_win)); %#ok<NASGU>
wsl_result = uav.setup.run_wsl_command( ...
    "bash " + temp_bash_wsl, ...
    'DistroName', cfg.wsl_distro_name, ...
    'WorkDir', pwd());
status_code = wsl_result.status_code;
output_text = string(strtrim(wsl_result.output_text));

info = struct();
info.ardupilot_pid = local_extract_marker(output_text, "arducopter_pid=");
info.ardupilot_alive_before_stop = contains(output_text, "arducopter_alive_before_stop=yes");
end

function mp_status = local_launch_mission_planner()
%LOCAL_LAUNCH_MISSION_PLANNER Запустить или переиспользовать Mission Planner.

process_command = ['powershell -NoProfile -NonInteractive -Command "' ...
    '$mp = Get-Process -Name MissionPlanner -ErrorAction SilentlyContinue | Select-Object -First 1; ' ...
    'if ($null -eq $mp) { ' ...
    '$candidates = @(' ...
    '''C:\Program Files (x86)\Mission Planner\MissionPlanner.exe'', ' ...
    '''C:\Program Files\Mission Planner\MissionPlanner.exe'', ' ...
    '''$env:LOCALAPPDATA\Mission Planner\MissionPlanner.exe''); ' ...
    '$path = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1; ' ...
    'if ($path) { $mp = Start-Process -FilePath $path -PassThru; Start-Sleep -Seconds 3 } }; ' ...
    '$udp = Get-NetUDPEndpoint -ErrorAction SilentlyContinue | Where-Object { $_.LocalPort -eq 14550 } | Select-Object -First 1; ' ...
    'Write-Output (' ...
    '''Mission Planner запущен: '' + $(if ($mp) { ''да'' } else { ''нет'' })); ' ...
    'if ($mp) { Write-Output (''PID: '' + $mp.Id) }; ' ...
    'Write-Output (''Порт UDP 14550 занят: '' + $(if ($udp) { ''да'' } else { ''нет'' })); ' ...
    'if ($udp) { Write-Output (''PID владельца порта 14550: '' + $udp.OwningProcess) }"'];
[~, output_text] = system(process_command);
mp_status = string(strtrim(output_text));
end

function local_cleanup_previous_internal_stand(repo_root)
%LOCAL_CLEANUP_PREVIOUS_INTERNAL_STAND Остановить остатки прошлых прогонов.

cleanup_command = "pkill arducopter >/dev/null 2>&1 || true; exit 0";
uav.setup.run_wsl_command( ...
    cleanup_command, ...
    'DistroName', 'Ubuntu', ...
    'WorkDir', repo_root);
pause(2.0);
end

function host_ip = local_detect_windows_host_ip()
%LOCAL_DETECT_WINDOWS_HOST_IP Определить адрес Windows-хоста для WSL2.

command_text = sprintf([ ...
    'powershell -NoProfile -NonInteractive -Command ' ...
    '"(Get-NetIPAddress -AddressFamily IPv4 | ' ...
    'Where-Object { $_.InterfaceAlias -like ''*WSL*'' -or $_.InterfaceAlias -like ''*vEthernet*'' } | ' ...
    'Select-Object -First 1 -ExpandProperty IPAddress)"']);
[status_code, output_text] = system(command_text);

if status_code == 0
    host_ip = string(strtrim(output_text));
else
    host_ip = "";
end

if strlength(host_ip) == 0
    host_ip = "172.19.208.1";
end
end

function text_value = local_format_sequence_result(seq_result)
%LOCAL_FORMAT_SEQUENCE_RESULT Сформировать текстовый блок результата pymavlink.

lines = strings(0, 1);
lines(end + 1, 1) = "HEARTBEAT получен: " + local_bool_text(local_get_logical(seq_result, 'heartbeat_received'));
lines(end + 1, 1) = "взведение выполнено: " + local_bool_text(local_get_logical(seq_result, 'arm_succeeded'));
lines(end + 1, 1) = "причина: " + string(local_get_text(seq_result, 'failure_reason'));
lines(end + 1, 1) = "arm ACK: " + string(local_get_numeric(seq_result, 'arm_ack_result'));
lines(end + 1, 1) = "takeoff ACK: " + string(local_get_numeric(seq_result, 'takeoff_ack_result'));
lines(end + 1, 1) = "максимальная относительная высота [m]: " + string(local_get_numeric(seq_result, 'max_relative_alt_m'));

if isfield(seq_result, 'status_texts') && ~isempty(seq_result.status_texts)
    lines(end + 1, 1) = "Последние STATUSTEXT:";
    lines = [lines; "  - " + seq_result.status_texts(:)]; %#ok<AGROW>
end

text_value = strjoin(lines, newline);
end

function telemetry_table = local_make_internal_csv_table(seq_result)
%LOCAL_MAKE_INTERNAL_CSV_TABLE Подготовить CSV-таблицу внутреннего SITL.

if isfield(seq_result, 'telemetry_table')
    telemetry_table = seq_result.telemetry_table;
else
    telemetry_table = table();
end

if isempty(telemetry_table)
    return;
end

rename_pairs = {
    't_s', 'time_s';
    'armed', 'armed';
    'mode', 'mode';
    'relative_alt_m', 'relative_alt_m';
    'heartbeat_count', 'heartbeat_count';
    'global_position_count', 'global_position_count';
    'status_text_count', 'status_text_count'};

for idx = 1:size(rename_pairs, 1)
    old_name = rename_pairs{idx, 1};
    new_name = rename_pairs{idx, 2};
    if any(strcmp(telemetry_table.Properties.VariableNames, old_name)) ...
            && ~strcmp(old_name, new_name)
        telemetry_table.Properties.VariableNames{old_name} = new_name;
    end
end
end

function local_plot_internal_figures(telemetry_table, fig_alt, fig_mode, fig_mav)
%LOCAL_PLOT_INTERNAL_FIGURES Построить графики внутреннего режима.

if isempty(telemetry_table)
    return;
end

local_save_figure(fig_alt, @() local_plot_altitude(telemetry_table));
local_save_figure(fig_mode, @() local_plot_mode_arm(telemetry_table));
local_save_figure(fig_mav, @() local_plot_mav_counts(telemetry_table));
end

function local_plot_altitude(telemetry_table)
%LOCAL_PLOT_ALTITUDE Построить график высоты.

plot(telemetry_table.time_s, telemetry_table.relative_alt_m, 'LineWidth', 1.5);
grid on;
xlabel('t, c');
ylabel('Относительная высота, м');
title('Штатный SITL: изменение высоты');
end

function local_plot_mode_arm(telemetry_table)
%LOCAL_PLOT_MODE_ARM Построить график режима и признака взведения.

tiledlayout(2, 1);

nexttile;
stairs(telemetry_table.time_s, double(telemetry_table.armed), 'LineWidth', 1.5);
grid on;
ylabel('Взведение');
title('Штатный SITL: режим взведения');

nexttile;
[~, ~, mode_index] = unique(telemetry_table.mode, 'stable');
stairs(telemetry_table.time_s, mode_index, 'LineWidth', 1.5);
grid on;
xlabel('t, c');
ylabel('Индекс режима');
title('Штатный SITL: смена режима');
end

function local_plot_mav_counts(telemetry_table)
%LOCAL_PLOT_MAV_COUNTS Построить график счетчиков сообщений MAVLink.

plot(telemetry_table.time_s, telemetry_table.heartbeat_count, 'LineWidth', 1.5);
hold on;
plot(telemetry_table.time_s, telemetry_table.global_position_count, 'LineWidth', 1.5);
plot(telemetry_table.time_s, telemetry_table.status_text_count, 'LineWidth', 1.5);
hold off;
grid on;
xlabel('t, c');
ylabel('Счетчик сообщений');
title('Штатный SITL: накопление сообщений MAVLink');
legend({'HEARTBEAT', 'GLOBAL_POSITION_INT', 'STATUSTEXT'}, 'Location', 'best');
end

function local_save_figure(path_value, plotter)
%LOCAL_SAVE_FIGURE Построить и сохранить рисунок.

fig = figure('Visible', 'off');
cleanup_obj = onCleanup(@() close(fig)); %#ok<NASGU>
plotter();
exportgraphics(fig, path_value, 'Resolution', 150);
end

function local_prepare_parent(path_value)
%LOCAL_PREPARE_PARENT Создать родительский каталог файла.

folder_path = fileparts(path_value);
if ~isfolder(folder_path)
    mkdir(folder_path);
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

local_prepare_parent(path_value);
text_value = string(text_value);
text_value(ismissing(text_value)) = "";
if numel(text_value) > 1
    text_value = strjoin(text_value, newline);
end
fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task18:internal_demo:OpenLog', ...
        'Не удалось открыть файл %s.', path_value);
end
cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function local_delete_if_exists(path_value)
%LOCAL_DELETE_IF_EXISTS Удалить временный файл при наличии.

if isfile(path_value)
    delete(path_value);
end
end

function tf = local_get_logical(data_struct, field_name)
%LOCAL_GET_LOGICAL Безопасно прочитать логическое поле структуры.

if isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    tf = logical(data_struct.(field_name));
else
    tf = false;
end
end

function value = local_get_numeric(data_struct, field_name)
%LOCAL_GET_NUMERIC Безопасно прочитать числовое поле структуры.

if isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    value = double(data_struct.(field_name));
else
    value = nan;
end
end

function text_value = local_get_text(data_struct, field_name)
%LOCAL_GET_TEXT Безопасно прочитать текстовое поле структуры.

if isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    text_value = string(data_struct.(field_name));
else
    text_value = "";
end
end

function value = local_extract_marker(output_text, marker)
%LOCAL_EXTRACT_MARKER Извлечь значение из строки вида marker=value.

lines = splitlines(string(output_text));
value = "";
for idx = 1:numel(lines)
    line_value = strtrim(lines(idx));
    if startsWith(line_value, marker)
        value = extractAfter(line_value, strlength(marker));
        return;
    end
end
end

function quoted = local_bash_single_quote(text_value)
%LOCAL_BASH_SINGLE_QUOTE Заключить текст в одиночные кавычки для bash.

text_value = string(text_value);
text_value = replace(text_value, "'", "'""'""'");
quoted = "'" + text_value + "'";
end

function wsl_path = local_windows_to_wsl_path(path_value)
%LOCAL_WINDOWS_TO_WSL_PATH Преобразовать путь Windows в путь WSL.

path_value = string(path_value);
full_path = string(java.io.File(char(path_value)).getCanonicalPath());
drive_letter = lower(extractBefore(full_path, ":\"));
tail_value = extractAfter(full_path, ":\");
tail_value = replace(tail_value, "\", "/");

if startsWith(tail_value, "/")
    tail_value = extractAfter(tail_value, 1);
end

wsl_path = "/mnt/" + drive_letter + "/" + tail_value;
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end
