%% COMPARE_INTERNAL_SITL_ACCEL_CALIBRATION_PARAMS Сравнить параметры калибровки внутреннего SITL и JSON-режима.
% Назначение:
%   Запускает штатный ArduPilot SITL без внешней модели, подтверждает
%   возможность взведения и считывает параметры калибровки акселерометра.
%   Затем сравнивает их с параметрами JSON-режима из TASK-21.
%
% Входы:
%   none
%
% Выходы:
%   task_21_internal_vs_json_accel_calibration - структура результата в base workspace
%
% Единицы измерения:
%   параметры - в единицах ArduPilot;
%   время - секунды.
%
% Допущения:
%   Сценарий `diagnose_ardupilot_accel_calibration_params.m` уже выполнялся
%   или его CSV-результат существует в `artifacts/reports`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

json_csv_path = fullfile(reports_dir, 'task_21_accel_calibration_params.csv');
log_path = fullfile(logs_dir, 'task_21_compare_internal_sitl_accel_calibration_params.txt');
csv_path = fullfile(reports_dir, 'task_21_internal_vs_json_accel_calibration.csv');
mat_path = fullfile(reports_dir, 'task_21_internal_vs_json_accel_calibration.mat');
internal_console_log = fullfile(logs_dir, 'task_21_internal_sitl_console.txt');

if ~isfile(json_csv_path)
    error('uav:task21:compareInternal:MissingJsonDiag', ...
        'Не найден результат диагностики JSON-режима: %s', json_csv_path);
end

json_table = readtable(json_csv_path, 'TextType', 'string');
cfg = uav.ardupilot.default_json_config();

local_stop_existing_internal(cfg);
[start_info, start_result] = local_start_internal_sitl(cfg, internal_console_log);
pause(8.0);

param_result = local_fetch_internal_params(cfg, string(json_table.param_name));
sequence_result = uav.ardupilot.try_arm_internal_sitl(cfg);
process_state = local_read_process_state(cfg.wsl_distro_name, start_info.pid_path, start_info.exit_path);
log_tail = local_read_log_tail(cfg, start_info.log_path);

local_stop_existing_internal(cfg);

internal_table = local_build_internal_table(param_result);
compare_table = local_build_compare_table(json_table, internal_table);

result = struct();
result.start_info = start_info;
result.start_result = start_result;
result.internal_params = internal_table;
result.compare_table = compare_table;
result.param_result = param_result;
result.sequence_result = sequence_result;
result.process_state = process_state;
result.log_tail = log_tail;
result.internal_arm_succeeded = logical(local_get_field(sequence_result, 'arm_succeeded', false));
result.internal_arm_failure_reason = string(local_get_field(sequence_result, 'failure_reason', ""));

writetable(compare_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_21_internal_vs_json_accel_calibration', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('TASK-21: сравнение параметров внутреннего SITL и JSON-режима\n');
fprintf('  internal arm succeeded                 : %s\n', local_bool_text(result.internal_arm_succeeded));
fprintf('  fetched internal params                : %d\n', height(internal_table));
fprintf('  differing rows                         : %d\n', sum(compare_table.is_different));

function [start_info, start_result] = local_start_internal_sitl(cfg, log_path)
pid_path = '/home/oaleg/task21_internal_sitl.pid';
exit_path = '/home/oaleg/task21_internal_sitl.exit';
log_path_wsl = uav.ardupilot.windows_to_wsl_path(log_path);
tmp_ps1 = [tempname, '_task21_internal_start.ps1'];
cleanup_tmp = onCleanup(@() local_delete_if_exists(tmp_ps1)); %#ok<NASGU>

supervisor_lines = strings(0, 1);
supervisor_lines(end + 1, 1) = "set -euo pipefail";
supervisor_lines(end + 1, 1) = "rm -f " + pid_path + " " + exit_path + " " + log_path_wsl;
supervisor_lines(end + 1, 1) = "cd " + string(cfg.ardupilot_root) + "/ArduCopter";
supervisor_lines(end + 1, 1) = "(";
supervisor_lines(end + 1, 1) = "  ../build/sitl/bin/arducopter --model + --speedup 1 --defaults ../Tools/autotest/default_params/copter.parm --serial0 tcp:0 -I0";
supervisor_lines(end + 1, 1) = ") >>" + log_path_wsl + " 2>&1 &";
supervisor_lines(end + 1, 1) = "pid=$!";
supervisor_lines(end + 1, 1) = "echo $pid >" + pid_path;
supervisor_lines(end + 1, 1) = "wait $pid";
supervisor_lines(end + 1, 1) = "code=$?";
supervisor_lines(end + 1, 1) = "echo $code >" + exit_path;

ps_lines = strings(0, 1);
ps_lines(end + 1, 1) = "$wslCommand = @'";
ps_lines = [ps_lines; supervisor_lines]; %#ok<AGROW>
ps_lines(end + 1, 1) = "'@";
ps_lines(end + 1, 1) = "$p = Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d','" + string(cfg.wsl_distro_name) + "','--','bash','-lc',$wslCommand) -WindowStyle Hidden -PassThru";
ps_lines(end + 1, 1) = "Write-Output ('WSL_WRAPPER_PID={0}' -f $p.Id)";

uav.ardupilot.write_utf8_text_file(tmp_ps1, strjoin(ps_lines, newline) + newline);
[status_code, output_text] = system(sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', tmp_ps1));

start_info = struct();
start_info.pid_path = string(pid_path);
start_info.exit_path = string(exit_path);
start_info.log_path = string(log_path_wsl);
start_info.command_text = "../build/sitl/bin/arducopter --model + --speedup 1 --defaults ../Tools/autotest/default_params/copter.parm --serial0 tcp:0 -I0";
start_result = struct('status_code', double(status_code), 'output_text', string(output_text));
end

function sequence_result = local_try_internal_arm(cfg)
seq_cfg = struct();
seq_cfg.wsl_distro_name = cfg.wsl_distro_name;
seq_cfg.connection_candidates = ["tcp:127.0.0.1:5760"; "tcp:127.0.0.1:5762"; "tcp:127.0.0.1:5763"];
seq_cfg.mode_name = "GUIDED";
seq_cfg.wait_before_command_s = 5.0;
seq_cfg.heartbeat_timeout_s = 15.0;
seq_cfg.arm_timeout_s = 15.0;
seq_cfg.monitor_duration_s = 10.0;
seq_cfg.takeoff_alt_m = nan;
seq_cfg.sample_period_s = 0.5;

command_info = uav.ardupilot.make_pymavlink_sequence_command(seq_cfg);
cleanup_py = onCleanup(@() local_delete_if_exists(command_info.python_script_path_win)); %#ok<NASGU>
cleanup_json = onCleanup(@() local_delete_if_exists(command_info.output_json_path_win)); %#ok<NASGU>
[status_code, output_text] = system(command_info.command_text); %#ok<ASGLU>
if isfile(command_info.output_json_path_win)
    sequence_result = uav.ardupilot.read_pymavlink_sequence_result(command_info.output_json_path_win);
else
    sequence_result = struct();
    sequence_result.arm_succeeded = false;
    sequence_result.failure_reason = "Не сформирован JSON-результат pymavlink для внутреннего SITL.";
    sequence_result.raw_output = string(output_text);
    sequence_result.status_code = double(status_code);
end
end

function state = local_read_process_state(distro_name, pid_path, exit_path)
pid_text = strtrim(local_run_wsl(distro_name, "cat " + string(pid_path) + " 2>/dev/null || true"));
exit_text = strtrim(local_run_wsl(distro_name, "cat " + string(exit_path) + " 2>/dev/null || true"));
state = struct('pid', 0, 'is_alive', false, 'exit_code_text', string(exit_text), 'ps_line', "");
if strlength(pid_text) == 0
    return;
end
pid_value = str2double(pid_text);
if ~isfinite(pid_value)
    return;
end
state.pid = double(pid_value);
ps_line = strtrim(local_run_wsl(distro_name, sprintf('ps -p %d -o pid=,ppid=,comm=,etimes=,args= 2>/dev/null || true', round(pid_value))));
state.ps_line = string(ps_line);
state.is_alive = strlength(state.ps_line) > 0;
end

function text_value = local_read_log_tail(cfg, log_path_wsl)
text_value = string(local_run_wsl(cfg.wsl_distro_name, "tail -n 80 " + string(log_path_wsl) + " 2>/dev/null || true"));
end

function table_value = local_build_internal_table(param_result)
param_names = string(fieldnames(param_result.param_values));
rows = repmat(struct('param_name', "", 'internal_value', nan), numel(param_names), 1);
for idx = 1:numel(param_names)
    rows(idx).param_name = param_names(idx);
    value = param_result.param_values.(char(param_names(idx)));
    if ~isempty(value)
        rows(idx).internal_value = double(value);
    end
end
table_value = struct2table(rows);
table_value.param_name = string(table_value.param_name);
mask = startsWith(table_value.param_name, "INS_ACC") ...
    | startsWith(table_value.param_name, "INS_USE") ...
    | startsWith(table_value.param_name, "INS_ENABLE") ...
    | startsWith(table_value.param_name, "AHRS_") ...
    | startsWith(table_value.param_name, "ARMING_") ...
    | startsWith(table_value.param_name, "LOG_");
table_value = table_value(mask, :);
table_value = sortrows(table_value, 'param_name');
end

function table_value = local_build_compare_table(json_table, internal_table)
all_names = union(string(json_table.param_name), string(internal_table.param_name), 'stable');
rows = repmat(struct( ...
    'param_name', "", ...
    'json_value', nan, ...
    'internal_value', nan, ...
    'delta_value', nan, ...
    'is_different', false), numel(all_names), 1);
for idx = 1:numel(all_names)
    name = all_names(idx);
    rows(idx).param_name = name;
    json_idx = find(string(json_table.param_name) == name, 1, 'first');
    if ~isempty(json_idx)
        rows(idx).json_value = double(json_table.param_value(json_idx));
    end
    int_idx = find(string(internal_table.param_name) == name, 1, 'first');
    if ~isempty(int_idx)
        rows(idx).internal_value = double(internal_table.internal_value(int_idx));
    end
    if isfinite(rows(idx).json_value) && isfinite(rows(idx).internal_value)
        rows(idx).delta_value = rows(idx).json_value - rows(idx).internal_value;
        rows(idx).is_different = abs(rows(idx).delta_value) > 1.0e-9;
    else
        rows(idx).is_different = xor(isfinite(rows(idx).json_value), isfinite(rows(idx).internal_value));
    end
end
table_value = sortrows(struct2table(rows), 'param_name');
end

function local_stop_existing_internal(cfg)
local_run_wsl(cfg.wsl_distro_name, ...
    "ps -eo pid=,args= | grep ""[a]rducopter"" | awk ""{print \$1}"" | xargs -r kill -TERM >/dev/null 2>&1 || true");
pause(1.0);
end

function param_result = local_fetch_internal_params(cfg, param_names)
candidate_connections = ["tcp:127.0.0.1:5760"; "tcp:127.0.0.1:5762"; "tcp:127.0.0.1:5763"];
param_result = struct('heartbeat_received', false, 'failure_reason', "Не выполнено", 'param_values', struct(), 'param_names', strings(0,1));
for idx = 1:numel(candidate_connections)
    trial = uav.ardupilot.fetch_mavlink_params(cfg, candidate_connections(idx), 'ParamNames', param_names, 'Timeout_s', 15.0);
    if trial.heartbeat_received && isempty(trial.failure_reason) && ~isempty(trial.param_names)
        param_result = trial;
        return;
    end
    param_result = trial;
end
end

function output_text = local_run_wsl(distro_name, bash_text)
command = sprintf('wsl.exe -d %s -- bash -lc ''%s''', char(distro_name), char(string(bash_text)));
[~, output_text] = system(command);
end

function value = local_get_field(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name)
    value = data.(field_name);
else
    value = default_value;
end
end

function local_delete_if_exists(path_text)
path_text = char(string(path_text));
if isfile(path_text)
    delete(path_text);
end
end

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-21: сравнение параметров калибровки внутреннего SITL и JSON-режима";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Команда внутреннего SITL: " + result.start_info.command_text;
lines(end + 1, 1) = "Internal arm succeeded: " + local_bool_text(result.internal_arm_succeeded);
lines(end + 1, 1) = "Internal arm failure reason: " + string(result.internal_arm_failure_reason);
lines(end + 1, 1) = "HEARTBEAT по tcp:5760: " + local_bool_text(result.param_result.heartbeat_received);
lines(end + 1, 1) = "Процесс arducopter жив до остановки: " + local_bool_text(result.process_state.is_alive);
lines(end + 1, 1) = "Различающихся параметров: " + string(sum(result.compare_table.is_different));
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Различающиеся строки:";
diff_rows = result.compare_table(result.compare_table.is_different, :);
if isempty(diff_rows)
    lines(end + 1, 1) = "  различий не найдено";
else
    for idx = 1:height(diff_rows)
        lines(end + 1, 1) = "  " + diff_rows.param_name(idx) + ...
            " json=" + string(diff_rows.json_value(idx)) + ...
            " internal=" + string(diff_rows.internal_value(idx)) + ...
            " delta=" + string(diff_rows.delta_value(idx));
    end
end
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Последние строки консоли внутреннего SITL:";
lines = [lines; splitlines(result.log_tail)]; %#ok<AGROW>
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
