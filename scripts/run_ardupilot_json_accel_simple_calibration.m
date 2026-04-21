%% RUN_ARDUPILOT_JSON_ACCEL_SIMPLE_CALIBRATION Выполнить простую калибровку акселерометра в JSON-режиме.
% Назначение:
%   Восстанавливает устойчивый обмен `ArduPilot JSON + MATLAB-модель`,
%   затем по приоритету пробует штатные механизмы калибровки акселерометра:
%   `MAVProxy accelcalsimple`, `MAVProxy ground`, прямую MAVLink-команду
%   простой калибровки и, при необходимости, принудительное подтверждение
%   существующей калибровки после перезагрузки параметров.
%
% Входы:
%   none
%
% Выходы:
%   task_21_json_accel_simple_calibration - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   счетчики обмена - безразмерные.
%
% Допущения:
%   Профиль TASK-20 уже устраняет `Arm: Accels inconsistent`, а обмен
%   `JSON/UDP` остается устойчивым.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

log_path = fullfile(logs_dir, 'task_21_json_accel_simple_calibration.txt');
csv_path = fullfile(reports_dir, 'task_21_json_accel_simple_calibration.csv');
mat_path = fullfile(reports_dir, 'task_21_json_accel_simple_calibration.mat');
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');

if ~isfile(parm_path)
    error('uav:task21:simpleCal:MissingParm', ...
        'Не найден профиль TASK-20: %s', parm_path);
end

cfg = uav.ardupilot.default_json_config();
baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_21_simplecal");

if ~baseline.baseline_restored
    uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
    error('uav:task21:simpleCal:BaselineFailed', ...
        'Не удалось восстановить устойчивый обмен перед калибровкой. Причина: %s', ...
        char(string(baseline.first_failure_reason)));
end

attempt_defs = [ ...
    struct('method_name', "mavproxy_accelcalsimple", 'kind', "mavproxy", 'command_text', "accelcalsimple"), ...
    struct('method_name', "mavproxy_ground", 'kind', "mavproxy", 'command_text', "ground"), ...
    struct('method_name', "mavlink_simple", 'kind', "mavlink", 'command_text', "prefight_calibration_p5_4"), ...
    struct('method_name', "mavlink_force_save", 'kind', "mavlink", 'command_text', "prefight_calibration_p5_76")];

attempt_rows = repmat(local_empty_attempt_row(), numel(attempt_defs), 1);
selected_idx = 0;

for idx = 1:numel(attempt_defs)
    attempt_rows(idx) = local_run_attempt(cfg, repo_root, attempt_defs(idx));
    if attempt_rows(idx).helper_completed && (attempt_rows(idx).ack_result == 0 || contains(lower(attempt_rows(idx).output_excerpt), "calibr"))
        selected_idx = idx;
        break;
    end
end

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
pause(2.0);

post_fetch = struct();
if selected_idx > 0
    post_baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_21_simplecal_post");
    if post_baseline.baseline_restored
        post_fetch = uav.ardupilot.fetch_mavlink_params(cfg, "tcp:127.0.0.1:5763", 'FetchAll', true, 'Timeout_s', 25.0);
        uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
    end
else
    post_baseline = struct();
end

result = struct();
result.parm_path = string(parm_path);
result.baseline = baseline;
result.attempt_table = struct2table(attempt_rows);
result.selected_method = "";
result.post_calibration_baseline = post_baseline;
result.post_calibration_fetch = post_fetch;
result.simple_calibration_performed = false;
result.force_save_used = false;

if selected_idx > 0
    result.selected_method = attempt_rows(selected_idx).method_name;
    result.simple_calibration_performed = any(result.selected_method == ["mavproxy_accelcalsimple", "mavlink_simple"]);
    result.force_save_used = result.selected_method == "mavlink_force_save";
end

writetable(result.attempt_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_21_json_accel_simple_calibration', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('TASK-21: штатная простая калибровка акселерометра\n');
fprintf('  baseline valid_rx_count                : %d\n', baseline.metrics.valid_rx_count);
fprintf('  baseline response_tx_count             : %d\n', baseline.metrics.response_tx_count);
fprintf('  selected method                        : %s\n', char(local_empty_as_none(result.selected_method)));
fprintf('  simple calibration performed           : %s\n', local_bool_text(result.simple_calibration_performed));
fprintf('  force save used                        : %s\n', local_bool_text(result.force_save_used));

function baseline = local_run_baseline(repo_root, logs_dir, parm_path, tag)
cfg_local = uav.ardupilot.default_json_config();
tag_char = char(tag);
baseline_mat_tmp = [tempname, '_', tag_char, '_baseline.mat'];
baseline_csv_tmp = [tempname, '_', tag_char, '_baseline.csv'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp})); %#ok<NASGU>

local_assign_base('ardupilot_task15_extra_defaults_win_path', parm_path);
local_assign_base('ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, [tag_char, '_baseline.txt']));
local_assign_base('ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, [tag_char, '_wait.txt']));
local_assign_base('ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, [tag_char, '_handshake.txt']));
local_assign_base('ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, [tag_char, '_live.txt']));
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_live_backend_duration_s', 25.0);
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_task15_extra_defaults_win_path'', ' ...
    '''ardupilot_task15_baseline_diag_log_path'', ' ...
    '''ardupilot_task15_baseline_wait_log_path'', ' ...
    '''ardupilot_task15_baseline_handshake_log_path'', ' ...
    '''ardupilot_task15_baseline_live_log_path'', ' ...
    '''ardupilot_task15_baseline_mat_report_path'', ' ...
    '''ardupilot_task15_baseline_csv_report_path'', ' ...
    '''ardupilot_live_backend_duration_s'');'])); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m'));
baseline = evalin('base', 'task_17_task15_baseline_exchange');

if ~baseline.baseline_restored
    uav.ardupilot.stop_existing_sitl(cfg_local.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg_local.mavlink_udp_port);
end
end

function row = local_run_attempt(cfg, repo_root, attempt_def)
helper = local_prepare_helper(cfg, attempt_def);
cleanup_files = onCleanup(@() local_cleanup_temp({ ...
    helper.python_script_path_win, ...
    helper.launcher_script_path_win, ...
    helper.output_json_path_win, ...
    helper.output_log_path_win})); %#ok<NASGU>

system(helper.launch_command); %#ok<NASGU>
local_assign_base('ardupilot_live_backend_duration_s', 30.0);
cleanup_base = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
run(fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m'));
live_result = evalin('base', 'ardupilot_json_udp_live_backend');
metrics = uav.ardupilot.summarize_live_backend_metrics(live_result);
helper_result = local_read_helper_result(helper.output_json_path_win, helper.output_log_path_win);

row = local_empty_attempt_row();
row.method_name = attempt_def.method_name;
row.command_text = attempt_def.command_text;
row.valid_rx_count = double(metrics.valid_rx_count);
row.response_tx_count = double(metrics.response_tx_count);
row.last_frame_count = double(metrics.last_frame_count);
row.helper_completed = logical(helper_result.helper_completed);
row.ack_result = double(helper_result.ack_result);
row.failure_reason = string(helper_result.failure_reason);
row.output_excerpt = string(helper_result.output_excerpt);
end

function helper = local_prepare_helper(cfg, attempt_def)
method_char = char(attempt_def.method_name);
python_script_path_win = [tempname, '_', method_char, '.py'];
launcher_script_path_win = [tempname, '_', method_char, '.ps1'];
output_json_path_win = [tempname, '_', method_char, '.json'];
output_log_path_win = [tempname, '_', method_char, '.log'];

python_script_path_wsl = uav.ardupilot.windows_to_wsl_path(python_script_path_win);
output_json_path_wsl = uav.ardupilot.windows_to_wsl_path(output_json_path_win);
output_log_path_wsl = uav.ardupilot.windows_to_wsl_path(output_log_path_win);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);
method_name = string(attempt_def.method_name);

if attempt_def.kind == "mavproxy"
    mavproxy_cmd = "/home/oaleg/venv-ardupilot/bin/mavproxy.py";
    if attempt_def.command_text == "accelcalsimple"
        bash_payload = "timeout 40s " + mavproxy_cmd + " --master=tcp:127.0.0.1:5763 --non-interactive --cmd=" + ...
            local_bash_quote("accelcalsimple; exit") + " >" + output_log_path_wsl + " 2>&1";
    else
        bash_payload = "timeout 40s " + mavproxy_cmd + " --master=tcp:127.0.0.1:5763 --non-interactive --cmd=" + ...
            local_bash_quote("ground; exit") + " >" + output_log_path_wsl + " 2>&1";
    end

    py_lines = strings(0, 1);
    py_lines(end + 1, 1) = "import json";
    py_lines(end + 1, 1) = "from pathlib import Path";
    py_lines(end + 1, 1) = "result = {";
    py_lines(end + 1, 1) = "    'helper_completed': True,";
    py_lines(end + 1, 1) = "    'ack_result': None,";
    py_lines(end + 1, 1) = "    'failure_reason': '',";
    py_lines(end + 1, 1) = "    'output_excerpt': ''";
    py_lines(end + 1, 1) = "}";
    py_lines(end + 1, 1) = "Path(" + local_py(output_json_path_wsl) + ").write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')";
    uav.ardupilot.write_utf8_text_file(python_script_path_win, strjoin(py_lines, newline) + newline);
    launcher_lines = strings(0, 1);
    launcher_lines(end + 1, 1) = "$ArgumentList = @('-d','" + string(cfg.wsl_distro_name) + "','--','bash','-lc','sleep 5; " + bash_payload + "; " + python_command_wsl + " " + python_script_path_wsl + "')"; 
    launcher_lines(end + 1, 1) = "Start-Process -WindowStyle Hidden -FilePath 'wsl.exe' -ArgumentList $ArgumentList | Out-Null";
    uav.ardupilot.write_utf8_text_file(launcher_script_path_win, strjoin(launcher_lines, newline) + newline);
else
    param5_value = 4.0;
    if method_name == "mavlink_force_save"
        param5_value = 76.0;
    end
    py_lines = strings(0, 1);
    py_lines(end + 1, 1) = "from pymavlink import mavutil";
    py_lines(end + 1, 1) = "import json";
    py_lines(end + 1, 1) = "import time";
    py_lines(end + 1, 1) = "from pathlib import Path";
    py_lines(end + 1, 1) = "result = {'helper_completed': True, 'ack_result': None, 'failure_reason': '', 'output_excerpt': ''}";
    py_lines(end + 1, 1) = "status_texts = []";
    py_lines(end + 1, 1) = "try:";
    py_lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=245, source_component=190)";
    py_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
    py_lines(end + 1, 1) = "    if hb is None:";
    py_lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT по tcp:5763 перед калибровкой.')";
    py_lines(end + 1, 1) = "    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, " + sprintf('%.0f', param5_value) + ", 0, 0)";
    py_lines(end + 1, 1) = "    deadline = time.time() + 25.0";
    py_lines(end + 1, 1) = "    while time.time() < deadline:";
    py_lines(end + 1, 1) = "        msg = master.recv_match(blocking=True, timeout=1)";
    py_lines(end + 1, 1) = "        if msg is None:";
    py_lines(end + 1, 1) = "            continue";
    py_lines(end + 1, 1) = "        mtype = msg.get_type()";
    py_lines(end + 1, 1) = "        if mtype == 'COMMAND_ACK' and int(msg.command) == int(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION):";
    py_lines(end + 1, 1) = "            result['ack_result'] = int(msg.result)";
    py_lines(end + 1, 1) = "        elif mtype == 'STATUSTEXT':";
    py_lines(end + 1, 1) = "            text = str(getattr(msg, 'text', '')).strip()";
    py_lines(end + 1, 1) = "            if text:";
    py_lines(end + 1, 1) = "                status_texts.append(text)";
    py_lines(end + 1, 1) = "    if status_texts:";
    py_lines(end + 1, 1) = "        result['output_excerpt'] = ' | '.join(status_texts[-8:])";
    py_lines(end + 1, 1) = "    if result['ack_result'] not in (0, None) and not result['failure_reason']:";
    py_lines(end + 1, 1) = "        result['failure_reason'] = 'Команда калибровки отклонена с кодом {}'.format(result['ack_result'])";
    py_lines(end + 1, 1) = "except Exception as exc:";
    py_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
    py_lines(end + 1, 1) = "Path(" + local_py(output_json_path_wsl) + ").write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')";
    uav.ardupilot.write_utf8_text_file(python_script_path_win, strjoin(py_lines, newline) + newline);
    launcher_lines = strings(0, 1);
    launcher_lines(end + 1, 1) = "$ArgumentList = @('-d','" + string(cfg.wsl_distro_name) + "','--','bash','-lc','sleep 5; " + python_command_wsl + " " + python_script_path_wsl + " >" + output_log_path_wsl + " 2>&1')";
    launcher_lines(end + 1, 1) = "Start-Process -WindowStyle Hidden -FilePath 'wsl.exe' -ArgumentList $ArgumentList | Out-Null";
    uav.ardupilot.write_utf8_text_file(launcher_script_path_win, strjoin(launcher_lines, newline) + newline);
end

helper = struct();
helper.python_script_path_win = string(python_script_path_win);
helper.launcher_script_path_win = string(launcher_script_path_win);
helper.output_json_path_win = string(output_json_path_win);
helper.output_log_path_win = string(output_log_path_win);
helper.launch_command = sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', char(launcher_script_path_win));
end

function helper_result = local_read_helper_result(json_path_win, log_path_win)
deadline = tic;
while toc(deadline) < 35.0
    if isfile(json_path_win)
        break;
    end
    pause(0.2);
end

helper_result = struct('helper_completed', false, 'ack_result', nan, 'failure_reason', "", 'output_excerpt', "");
if isfile(json_path_win)
    decoded = jsondecode(fileread(json_path_win));
    helper_result.helper_completed = logical(local_get_field(decoded, 'helper_completed', false));
    helper_result.ack_result = double(local_get_field(decoded, 'ack_result', nan));
    helper_result.failure_reason = string(local_get_field(decoded, 'failure_reason', ""));
    helper_result.output_excerpt = string(local_get_field(decoded, 'output_excerpt', ""));
    if strlength(helper_result.output_excerpt) == 0 && isfile(log_path_win)
        helper_result.output_excerpt = local_tail_excerpt(fileread(log_path_win));
    end
elseif isfile(log_path_win)
    helper_result.failure_reason = "Не сформирован JSON-результат калибровки.";
    helper_result.output_excerpt = local_tail_excerpt(fileread(log_path_win));
end
end

function row = local_empty_attempt_row()
row = struct( ...
    'method_name', "", ...
    'command_text', "", ...
    'valid_rx_count', 0, ...
    'response_tx_count', 0, ...
    'last_frame_count', 0, ...
    'helper_completed', false, ...
    'ack_result', nan, ...
    'failure_reason', "", ...
    'output_excerpt', "");
end

function value = local_get_field(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name)
    value = data.(field_name);
else
    value = default_value;
end
end

function local_assign_base(name, value)
assignin('base', name, value);
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    if isfile(file_list{idx})
        delete(file_list{idx});
    end
end
end

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-21: штатная простая калибровка акселерометра";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль TASK-20: " + result.parm_path;
lines(end + 1, 1) = "Baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "Baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "Выбранный метод: " + local_empty_as_none(result.selected_method);
lines(end + 1, 1) = "Простая калибровка выполнена: " + local_bool_text(result.simple_calibration_performed);
lines(end + 1, 1) = "Force-save применен: " + local_bool_text(result.force_save_used);
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Попытки:";
for idx = 1:height(result.attempt_table)
    row = result.attempt_table(idx, :);
    lines(end + 1, 1) = "  " + row.method_name + ...
        " valid_rx=" + string(row.valid_rx_count) + ...
        " response_tx=" + string(row.response_tx_count) + ...
        " ack=" + string(row.ack_result) + ...
        " completed=" + local_bool_text(row.helper_completed) + ...
        " reason=" + string(row.failure_reason);
    if strlength(string(row.output_excerpt)) > 0
        lines(end + 1, 1) = "    excerpt: " + string(row.output_excerpt);
    end
end
if isstruct(result.post_calibration_baseline) && ~isempty(fieldnames(result.post_calibration_baseline))
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "Post-restart valid_rx_count: " + string(result.post_calibration_baseline.metrics.valid_rx_count);
    lines(end + 1, 1) = "Post-restart response_tx_count: " + string(result.post_calibration_baseline.metrics.response_tx_count);
end
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_empty_as_none(value)
value = string(value);
if strlength(value) == 0
    text_value = "нет";
else
    text_value = value;
end
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function literal = local_py(text_value)
literal = "'" + replace(string(text_value), "'", "\\'") + "'";
end

function literal = local_bash_quote(text_value)
text_value = string(text_value);
text_value = replace(text_value, "'", "'""'""'");
literal = "'" + text_value + "'";
end

function text_value = local_tail_excerpt(raw_text)
lines = splitlines(string(raw_text));
lines = strtrim(lines);
lines(lines == "") = [];
if isempty(lines)
    text_value = "";
else
    lines = lines(max(1, numel(lines) - 7):end);
    text_value = strjoin(lines, " | ");
end
end
