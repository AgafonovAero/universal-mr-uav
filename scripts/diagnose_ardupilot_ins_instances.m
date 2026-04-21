%% DIAGNOSE_ARDUPILOT_INS_INSTANCES Проверить виртуальные ИНС ArduPilot в JSON-режиме.
% Назначение:
%   На базе устойчивого JSON-обмена TASK-15 подключается к MAVLink,
%   запрашивает сообщения IMU и параметры INS_USE*, выполняет попытку arm
%   и фиксирует, есть ли признаки рассогласования нескольких виртуальных ИНС.
%
% Входы:
%   none
%
% Выходы:
%   task_19_ins_instances - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   значения IMU сохраняются в тех единицах, в которых они приходят по MAVLink
%
% Допущения:
%   В WSL доступен Python с pymavlink.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_19_ins_instances.txt');
csv_path = fullfile(reports_dir, 'task_19_ins_instances.csv');
mat_path = fullfile(reports_dir, 'task_19_ins_instances.mat');

cfg = uav.ardupilot.default_json_config();
cfg_override = local_pick_cfg_override(cfg, repo_root);

baseline_diag_log = fullfile(logs_dir, 'task_19_ins_instances_baseline.txt');
baseline_wait_log = fullfile(logs_dir, 'task_19_ins_instances_wait.txt');
baseline_handshake_log = fullfile(logs_dir, 'task_19_ins_instances_handshake.txt');
baseline_live_log = fullfile(logs_dir, 'task_19_ins_instances_live_backend.txt');
baseline_mat_tmp = [tempname, '_task19_ins_baseline.mat'];
baseline_csv_tmp = [tempname, '_task19_ins_baseline.csv'];

python_script_win = fullfile(logs_dir, 'task_19_ins_instances_helper.py');
python_result_win = fullfile(reports_dir, 'task_19_ins_instances_raw.json');
python_launcher_win = fullfile(logs_dir, 'task_19_ins_instances_helper_launcher.ps1');

cleanup_temp = onCleanup(@() local_cleanup_temp_files({ ...
    baseline_mat_tmp, baseline_csv_tmp, python_launcher_win})); %#ok<NASGU>

local_assign_base('ardupilot_json_cfg_override', cfg_override);
local_assign_base('ardupilot_task15_baseline_diag_log_path', baseline_diag_log);
local_assign_base('ardupilot_task15_baseline_wait_log_path', baseline_wait_log);
local_assign_base('ardupilot_task15_baseline_handshake_log_path', baseline_handshake_log);
local_assign_base('ardupilot_task15_baseline_live_log_path', baseline_live_log);
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_json_cfg_override'', ' ...
    '''ardupilot_task15_baseline_diag_log_path'', ' ...
    '''ardupilot_task15_baseline_wait_log_path'', ' ...
    '''ardupilot_task15_baseline_handshake_log_path'', ' ...
    '''ardupilot_task15_baseline_live_log_path'', ' ...
    '''ardupilot_task15_baseline_mat_report_path'', ' ...
    '''ardupilot_task15_baseline_csv_report_path'');'])); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m'));
baseline = evalin('base', 'task_17_task15_baseline_exchange');

if ~baseline.baseline_restored
    result = struct();
    result.cfg_override = cfg_override;
    result.baseline = baseline;
    result.failure_reason = string(baseline.first_failure_reason);
    save(mat_path, 'result');
    writetable(table(string(baseline.first_failure_reason), ...
        'VariableNames', {'failure_reason'}), csv_path);
    local_write_utf8_text(log_path, ...
        "Диагностика ИНС TASK-19 не выполнена: базовый JSON-обмен TASK-15 не восстановлен." + newline ...
        + "Причина: " + string(baseline.first_failure_reason) + newline);
    assignin('base', 'task_19_ins_instances', result);
    error('uav:task19:insinstances:BaselineFailed', ...
        'Базовый обмен TASK-15 не восстановлен: %s', ...
        char(string(baseline.first_failure_reason)));
end

python_cfg = struct();
python_cfg.wsl_distro_name = string(cfg.wsl_distro_name);
python_cfg.connection_string = "tcp:127.0.0.1:5763";
python_cfg.param_names = ["INS_USE"; "INS_USE2"; "INS_USE3"; "INS_ENABLE_MASK"];
python_cfg.monitor_duration_s = 20.0;
python_cfg.wait_before_arm_s = 5.0;
python_cfg.python_script_path = string(python_script_win);
python_cfg.output_json_path = string(python_result_win);
python_cfg.launcher_script_path = string(python_launcher_win);
python_cfg.python_command_wsl = local_resolve_wsl_python(cfg);

command_info = local_make_python_command(python_cfg);
local_delete_if_exists(char(python_result_win));
[status_code, output_text] = system(command_info.command_text); %#ok<ASGLU>
python_result = local_read_python_result(string(python_result_win), status_code, output_text);

summary = local_make_summary_struct(baseline, python_result, cfg_override);
summary.command_text = string(command_info.command_text);
summary.python_output_text = string(output_text);
save(mat_path, 'summary');
writetable(local_make_summary_table(summary), csv_path);
local_write_utf8_text(log_path, local_make_log_text(summary));
assignin('base', 'task_19_ins_instances', summary);
local_stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);

fprintf('Диагностика виртуальных ИНС TASK-19\n');
fprintf('  baseline restored                      : %s\n', local_bool_text(baseline.baseline_restored));
fprintf('  heartbeat received                     : %s\n', local_bool_text(summary.heartbeat_received));
fprintf('  INS_USE                               : %.0f\n', summary.INS_USE);
fprintf('  INS_USE2                              : %.0f\n', summary.INS_USE2);
fprintf('  INS_USE3                              : %.0f\n', summary.INS_USE3);
fprintf('  arm ACK                               : %.0f\n', summary.arm_ack_result);
fprintf('  failure reason                         : %s\n', char(summary.failure_reason));

function cfg_override = local_pick_cfg_override(cfg, repo_root)
%LOCAL_PICK_CFG_OVERRIDE Выбрать рабочее диагностическое переопределение JSON.

cfg_override = struct( ...
    'json_accel_mode', string(cfg.json_accel_mode), ...
    'json_prearm_hold_enabled', true, ...
    'json_prearm_pwm_threshold_us', 1005.0);

mat_path = fullfile(repo_root, 'artifacts', 'reports', 'task_19_accel_convention_experiments.mat');
if isfile(mat_path)
    loaded = load(mat_path);
    if isfield(loaded, 'summary') && isfield(loaded.summary, 'selected_mode')
        cfg_override.json_accel_mode = string(loaded.summary.selected_mode);
    end
end
end

function command_info = local_make_python_command(python_cfg)
%LOCAL_MAKE_PYTHON_COMMAND Сформировать Python-команду диагностики ИНС.

python_script_wsl = local_windows_to_wsl_path(python_cfg.python_script_path);
output_json_wsl = local_windows_to_wsl_path(python_cfg.output_json_path);
param_list = join("    " + local_python_literal(python_cfg.param_names) + ",", newline);

python_lines = strings(0, 1);
python_lines(end + 1, 1) = "from pymavlink import mavutil";
python_lines(end + 1, 1) = "import json";
python_lines(end + 1, 1) = "import math";
python_lines(end + 1, 1) = "import time";
python_lines(end + 1, 1) = "from pathlib import Path";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "OUTPUT_PATH = Path(" + local_python_literal(output_json_wsl) + ")";
python_lines(end + 1, 1) = "CONNECTION = " + local_python_literal(python_cfg.connection_string);
python_lines(end + 1, 1) = "PARAM_NAMES = [";
python_lines(end + 1, 1) = param_list;
python_lines(end + 1, 1) = "]";
python_lines(end + 1, 1) = "MONITOR_DURATION_S = " + sprintf('%.6f', double(python_cfg.monitor_duration_s));
python_lines(end + 1, 1) = "WAIT_BEFORE_ARM_S = " + sprintf('%.6f', double(python_cfg.wait_before_arm_s));
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "result = {";
python_lines(end + 1, 1) = "  'heartbeat_received': False,";
python_lines(end + 1, 1) = "  'connection_string': str(CONNECTION),";
python_lines(end + 1, 1) = "  'failure_reason': '',";
python_lines(end + 1, 1) = "  'system_id': 0,";
python_lines(end + 1, 1) = "  'component_id': 0,";
python_lines(end + 1, 1) = "  'param_values': {},";
python_lines(end + 1, 1) = "  'message_counts': {},";
python_lines(end + 1, 1) = "  'latest_messages': {},";
python_lines(end + 1, 1) = "  'status_texts': [],";
python_lines(end + 1, 1) = "  'command_acks': [],";
python_lines(end + 1, 1) = "  'arm_ack_result': None,";
python_lines(end + 1, 1) = "  'arm_succeeded': False,";
python_lines(end + 1, 1) = "  'elapsed_s': 0.0";
python_lines(end + 1, 1) = "}";
python_lines(end + 1, 1) = "interesting = {'RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3', 'HIGHRES_IMU', 'ATTITUDE', 'LOCAL_POSITION_NED', 'STATUSTEXT', 'COMMAND_ACK', 'HEARTBEAT'}";
python_lines(end + 1, 1) = "start_t = time.time()";
python_lines(end + 1, 1) = "master = None";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def safe_float(value):";
python_lines(end + 1, 1) = "    try:";
python_lines(end + 1, 1) = "        value = float(value)";
python_lines(end + 1, 1) = "    except Exception:";
python_lines(end + 1, 1) = "        return None";
python_lines(end + 1, 1) = "    return value if math.isfinite(value) else None";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def struct_from_message(msg):";
python_lines(end + 1, 1) = "    out = {}";
python_lines(end + 1, 1) = "    for field in getattr(msg, '_fieldnames', []):";
python_lines(end + 1, 1) = "        value = getattr(msg, field, None)";
python_lines(end + 1, 1) = "        if isinstance(value, (int, bool)):";
python_lines(end + 1, 1) = "            out[field] = int(value)";
python_lines(end + 1, 1) = "        elif isinstance(value, float):";
python_lines(end + 1, 1) = "            out[field] = safe_float(value)";
python_lines(end + 1, 1) = "        else:";
python_lines(end + 1, 1) = "            out[field] = str(value)";
python_lines(end + 1, 1) = "    return out";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "try:";
python_lines(end + 1, 1) = "    master = mavutil.mavlink_connection(CONNECTION, source_system=245, source_component=190)";
python_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
python_lines(end + 1, 1) = "    if hb is None:";
python_lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT по tcp:5763.')";
python_lines(end + 1, 1) = "    result['heartbeat_received'] = True";
python_lines(end + 1, 1) = "    result['system_id'] = int(hb.get_srcSystem())";
python_lines(end + 1, 1) = "    result['component_id'] = int(hb.get_srcComponent())";
python_lines(end + 1, 1) = "    result['message_counts']['HEARTBEAT'] = 1";
python_lines(end + 1, 1) = "    result['latest_messages']['HEARTBEAT'] = struct_from_message(hb)";
python_lines(end + 1, 1) = "    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)";
python_lines(end + 1, 1) = "    for param_name in PARAM_NAMES:";
python_lines(end + 1, 1) = "        try:";
python_lines(end + 1, 1) = "            master.param_fetch_one(param_name)";
python_lines(end + 1, 1) = "        except Exception:";
python_lines(end + 1, 1) = "            pass";
python_lines(end + 1, 1) = "    arm_sent = False";
python_lines(end + 1, 1) = "    deadline = time.time() + MONITOR_DURATION_S";
python_lines(end + 1, 1) = "    while time.time() < deadline:";
python_lines(end + 1, 1) = "        msg = master.recv_match(blocking=True, timeout=1)";
python_lines(end + 1, 1) = "        if msg is None:";
python_lines(end + 1, 1) = "            continue";
python_lines(end + 1, 1) = "        msg_type = msg.get_type()";
python_lines(end + 1, 1) = "        if msg_type == 'BAD_DATA':";
python_lines(end + 1, 1) = "            continue";
python_lines(end + 1, 1) = "        result['message_counts'][msg_type] = int(result['message_counts'].get(msg_type, 0)) + 1";
python_lines(end + 1, 1) = "        if msg_type in interesting:";
python_lines(end + 1, 1) = "            result['latest_messages'][msg_type] = struct_from_message(msg)";
python_lines(end + 1, 1) = "        if msg_type == 'PARAM_VALUE':";
python_lines(end + 1, 1) = "            pname = str(getattr(msg, 'param_id', '')).rstrip('\\x00')";
python_lines(end + 1, 1) = "            if pname:";
python_lines(end + 1, 1) = "                result['param_values'][pname] = safe_float(getattr(msg, 'param_value', None))";
python_lines(end + 1, 1) = "        elif msg_type == 'STATUSTEXT':";
python_lines(end + 1, 1) = "            result['status_texts'].append(str(getattr(msg, 'text', '')).strip())";
python_lines(end + 1, 1) = "        elif msg_type == 'COMMAND_ACK':";
python_lines(end + 1, 1) = "            ack = {'command': int(getattr(msg, 'command', -1)), 'result': int(getattr(msg, 'result', -1))}";
python_lines(end + 1, 1) = "            result['command_acks'].append(ack)";
python_lines(end + 1, 1) = "            if ack['command'] == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):";
python_lines(end + 1, 1) = "                result['arm_ack_result'] = ack['result']";
python_lines(end + 1, 1) = "        elif msg_type == 'HEARTBEAT':";
python_lines(end + 1, 1) = "            armed = bool(int(getattr(msg, 'base_mode', 0)) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))";
python_lines(end + 1, 1) = "            if armed:";
python_lines(end + 1, 1) = "                result['arm_succeeded'] = True";
python_lines(end + 1, 1) = "        if (not arm_sent) and (time.time() - start_t) >= WAIT_BEFORE_ARM_S:";
python_lines(end + 1, 1) = "            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)";
python_lines(end + 1, 1) = "            arm_sent = True";
python_lines(end + 1, 1) = "    if not result['arm_succeeded']:";
python_lines(end + 1, 1) = "        arm_msgs = [text for text in result['status_texts'] if str(text).startswith('Arm:')]";
python_lines(end + 1, 1) = "        if arm_msgs:";
python_lines(end + 1, 1) = "            result['failure_reason'] = arm_msgs[-1]";
python_lines(end + 1, 1) = "        elif result['arm_ack_result'] is not None:";
python_lines(end + 1, 1) = "            result['failure_reason'] = 'Команда arm отклонена с кодом {}.'.format(result['arm_ack_result'])";
python_lines(end + 1, 1) = "        else:";
python_lines(end + 1, 1) = "            result['failure_reason'] = 'Arm не подтвержден, COMMAND_ACK не получен.'";
python_lines(end + 1, 1) = "except Exception as exc:";
python_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
python_lines(end + 1, 1) = "result['elapsed_s'] = safe_float(time.time() - start_t)";
python_lines(end + 1, 1) = "OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)";
python_lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";

local_write_utf8_text(python_cfg.python_script_path, strjoin(python_lines, newline) + newline);

launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$ArgumentList = @(";
launcher_lines(end + 1, 1) = "    '-d',";
launcher_lines(end + 1, 1) = "    '" + string(python_cfg.wsl_distro_name) + "',";
launcher_lines(end + 1, 1) = "    '--',";
launcher_lines(end + 1, 1) = "    'bash',";
launcher_lines(end + 1, 1) = "    '--noprofile',";
launcher_lines(end + 1, 1) = "    '--norc',";
launcher_lines(end + 1, 1) = "    '-lc',";
launcher_lines(end + 1, 1) = "    '" + string(python_cfg.python_command_wsl) + " " + python_script_wsl + "'";
launcher_lines(end + 1, 1) = ")";
launcher_lines(end + 1, 1) = "& wsl.exe @ArgumentList";
launcher_lines(end + 1, 1) = "exit $LASTEXITCODE";
local_write_utf8_text(python_cfg.launcher_script_path, strjoin(launcher_lines, newline) + newline);

command_info = struct();
command_info.command_text = sprintf( ...
    'powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', ...
    char(python_cfg.launcher_script_path));
end

function python_command_wsl = local_resolve_wsl_python(cfg)
%LOCAL_RESOLVE_WSL_PYTHON Определить интерпретатор Python с pymavlink внутри WSL.

preferred_command = "/home/oaleg/venv-ardupilot/bin/python3";
probe_command = sprintf( ...
    'wsl -d %s -- bash -lc ''if [ -x %s ]; then printf %s; else command -v python3; fi''', ...
    char(cfg.wsl_distro_name), ...
    preferred_command, ...
    preferred_command);
[status_code, raw_output] = system(probe_command);
raw_output = strtrim(string(raw_output));

if status_code == 0 && strlength(raw_output) > 0
    python_command_wsl = raw_output;
else
    python_command_wsl = "python3";
end
end

function result = local_read_python_result(json_path, status_code, output_text)
%LOCAL_READ_PYTHON_RESULT Прочитать JSON-результат Python-сценария.

result = struct();
result.status_code = double(status_code);
result.output_text = string(output_text);
result.heartbeat_received = false;
result.connection_string = "";
result.failure_reason = "";
result.system_id = 0;
result.component_id = 0;
result.param_values = struct();
result.message_counts = struct();
result.latest_messages = struct();
result.status_texts = strings(0, 1);
result.command_acks = struct('command', {}, 'result', {});
result.arm_ack_result = nan;
result.arm_succeeded = false;
result.elapsed_s = nan;

deadline = tic;
while ~isfile(char(json_path)) && toc(deadline) < 5.0
    pause(0.2);
end

if ~isfile(char(json_path))
    result.failure_reason = "Python-результат диагностики ИНС не был сохранен. system() output: " + string(output_text);
    return;
end

parsed = jsondecode(fileread(char(json_path)));
result = local_overlay_struct(result, parsed);

if isfield(parsed, 'status_texts') && ~isempty(parsed.status_texts)
    result.status_texts = string(parsed.status_texts(:));
end

function local_delete_if_exists(file_path)
%LOCAL_DELETE_IF_EXISTS Удалить файл, если он существует.

if isfile(file_path)
    delete(file_path);
end
end

if isfield(parsed, 'command_acks') && ~isempty(parsed.command_acks)
    result.command_acks = parsed.command_acks;
end

if isfield(parsed, 'arm_ack_result') && ~isempty(parsed.arm_ack_result)
    result.arm_ack_result = double(parsed.arm_ack_result);
else
    result.arm_ack_result = nan;
end
end

function summary = local_make_summary_struct(baseline, python_result, cfg_override)
%LOCAL_MAKE_SUMMARY_STRUCT Сформировать итоговую структуру диагностики ИНС.

summary = struct();
summary.cfg_override = cfg_override;
summary.baseline = baseline;
summary.baseline_valid_rx_count = double(baseline.metrics.valid_rx_count);
summary.baseline_response_tx_count = double(baseline.metrics.response_tx_count);
summary.baseline_json_tx_count = double(baseline.metrics.json_tx_count);
summary.baseline_last_frame_count = double(baseline.metrics.last_frame_count);
summary.heartbeat_received = logical(local_struct_scalar(python_result, 'heartbeat_received', false));
summary.connection_string = string(local_struct_string(python_result, 'connection_string', ""));
summary.failure_reason = string(local_struct_string(python_result, 'failure_reason', ""));
summary.system_id = double(local_struct_scalar(python_result, 'system_id', 0));
summary.component_id = double(local_struct_scalar(python_result, 'component_id', 0));
summary.arm_succeeded = logical(local_struct_scalar(python_result, 'arm_succeeded', false));
summary.arm_ack_result = double(local_struct_scalar(python_result, 'arm_ack_result', nan));
summary.elapsed_s = double(local_struct_scalar(python_result, 'elapsed_s', nan));
summary.status_texts = strings(0, 1);
if isfield(python_result, 'status_texts') && ~isempty(python_result.status_texts)
    summary.status_texts = string(python_result.status_texts(:));
end

summary.message_counts = local_struct_field_or_default(python_result, 'message_counts', struct());
summary.latest_messages = local_struct_field_or_default(python_result, 'latest_messages', struct());
summary.param_values = local_struct_field_or_default(python_result, 'param_values', struct());

summary.INS_USE = local_param_value(summary.param_values, 'INS_USE');
summary.INS_USE2 = local_param_value(summary.param_values, 'INS_USE2');
summary.INS_USE3 = local_param_value(summary.param_values, 'INS_USE3');
summary.INS_ENABLE_MASK = local_param_value(summary.param_values, 'INS_ENABLE_MASK');

summary.scaled_imu = local_message_vector(summary.latest_messages, 'SCALED_IMU', {'xacc', 'yacc', 'zacc'});
summary.scaled_imu2 = local_message_vector(summary.latest_messages, 'SCALED_IMU2', {'xacc', 'yacc', 'zacc'});
summary.scaled_imu3 = local_message_vector(summary.latest_messages, 'SCALED_IMU3', {'xacc', 'yacc', 'zacc'});
summary.raw_imu = local_message_vector(summary.latest_messages, 'RAW_IMU', {'xacc', 'yacc', 'zacc'});
summary.highres_imu = local_message_vector(summary.latest_messages, 'HIGHRES_IMU', {'xacc', 'yacc', 'zacc'});
summary.scaled_imu12_diff_norm = local_diff_norm(summary.scaled_imu, summary.scaled_imu2);
summary.scaled_imu13_diff_norm = local_diff_norm(summary.scaled_imu, summary.scaled_imu3);
summary.multiple_imu_streams_present = local_message_count(summary.message_counts, 'SCALED_IMU2') > 0 ...
    || local_message_count(summary.message_counts, 'SCALED_IMU3') > 0;
end

function data_table = local_make_summary_table(summary)
%LOCAL_MAKE_SUMMARY_TABLE Построить CSV-таблицу итогов диагностики ИНС.

data_table = table( ...
    string(summary.cfg_override.json_accel_mode), ...
    summary.baseline_valid_rx_count, ...
    summary.baseline_response_tx_count, ...
    summary.baseline_json_tx_count, ...
    summary.baseline_last_frame_count, ...
    summary.heartbeat_received, ...
    summary.system_id, ...
    summary.component_id, ...
    summary.INS_USE, ...
    summary.INS_USE2, ...
    summary.INS_USE3, ...
    summary.INS_ENABLE_MASK, ...
    local_message_count(summary.message_counts, 'RAW_IMU'), ...
    local_message_count(summary.message_counts, 'SCALED_IMU'), ...
    local_message_count(summary.message_counts, 'SCALED_IMU2'), ...
    local_message_count(summary.message_counts, 'SCALED_IMU3'), ...
    local_message_count(summary.message_counts, 'HIGHRES_IMU'), ...
    local_message_count(summary.message_counts, 'ATTITUDE'), ...
    local_message_count(summary.message_counts, 'LOCAL_POSITION_NED'), ...
    local_message_count(summary.message_counts, 'STATUSTEXT'), ...
    local_message_count(summary.message_counts, 'COMMAND_ACK'), ...
    summary.arm_ack_result, ...
    summary.arm_succeeded, ...
    summary.scaled_imu(1), summary.scaled_imu(2), summary.scaled_imu(3), ...
    summary.scaled_imu2(1), summary.scaled_imu2(2), summary.scaled_imu2(3), ...
    summary.scaled_imu3(1), summary.scaled_imu3(2), summary.scaled_imu3(3), ...
    summary.scaled_imu12_diff_norm, ...
    summary.scaled_imu13_diff_norm, ...
    string(summary.failure_reason), ...
    string(strjoin(summary.status_texts(:).', " | ")), ...
    'VariableNames', { ...
        'accel_mode', ...
        'baseline_valid_rx_count', ...
        'baseline_response_tx_count', ...
        'baseline_json_tx_count', ...
        'baseline_last_frame_count', ...
        'heartbeat_received', ...
        'system_id', ...
        'component_id', ...
        'INS_USE', ...
        'INS_USE2', ...
        'INS_USE3', ...
        'INS_ENABLE_MASK', ...
        'raw_imu_count', ...
        'scaled_imu_count', ...
        'scaled_imu2_count', ...
        'scaled_imu3_count', ...
        'highres_imu_count', ...
        'attitude_count', ...
        'local_position_ned_count', ...
        'statustext_count', ...
        'command_ack_count', ...
        'arm_ack_result', ...
        'arm_succeeded', ...
        'scaled_imu_xacc', 'scaled_imu_yacc', 'scaled_imu_zacc', ...
        'scaled_imu2_xacc', 'scaled_imu2_yacc', 'scaled_imu2_zacc', ...
        'scaled_imu3_xacc', 'scaled_imu3_yacc', 'scaled_imu3_zacc', ...
        'scaled_imu12_diff_norm', ...
        'scaled_imu13_diff_norm', ...
        'failure_reason', ...
        'status_texts'});
end

function text_value = local_make_log_text(summary)
%LOCAL_MAKE_LOG_TEXT Сформировать текстовый журнал диагностики ИНС.

lines = strings(0, 1);
lines(end + 1, 1) = "Диагностика виртуальных ИНС ArduPilot";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Режим accel_body: " + string(summary.cfg_override.json_accel_mode);
lines(end + 1, 1) = "baseline valid_rx_count: " + string(summary.baseline_valid_rx_count);
lines(end + 1, 1) = "baseline response_tx_count: " + string(summary.baseline_response_tx_count);
lines(end + 1, 1) = "baseline last_frame_count: " + string(summary.baseline_last_frame_count);
lines(end + 1, 1) = "HEARTBEAT получен: " + local_bool_text(summary.heartbeat_received);
lines(end + 1, 1) = "INS_USE: " + string(summary.INS_USE);
lines(end + 1, 1) = "INS_USE2: " + string(summary.INS_USE2);
lines(end + 1, 1) = "INS_USE3: " + string(summary.INS_USE3);
lines(end + 1, 1) = "INS_ENABLE_MASK: " + string(summary.INS_ENABLE_MASK);
lines(end + 1, 1) = "SCALED_IMU2 присутствует: " + local_bool_text(local_message_count(summary.message_counts, 'SCALED_IMU2') > 0);
lines(end + 1, 1) = "SCALED_IMU3 присутствует: " + local_bool_text(local_message_count(summary.message_counts, 'SCALED_IMU3') > 0);
lines(end + 1, 1) = "Разность SCALED_IMU и SCALED_IMU2 по норме: " + sprintf('%.6f', summary.scaled_imu12_diff_norm);
lines(end + 1, 1) = "Разность SCALED_IMU и SCALED_IMU3 по норме: " + sprintf('%.6f', summary.scaled_imu13_diff_norm);
lines(end + 1, 1) = "ACK arm: " + string(summary.arm_ack_result);
lines(end + 1, 1) = "Arm выполнен: " + local_bool_text(summary.arm_succeeded);
lines(end + 1, 1) = "Причина: " + local_text_value(summary.failure_reason);
if isfield(summary, 'command_text')
    lines(end + 1, 1) = "Команда Python: " + local_text_value(summary.command_text);
end
if isfield(summary, 'python_output_text') && strlength(strtrim(string(summary.python_output_text))) > 0
    lines(end + 1, 1) = "system() output: " + local_text_value(summary.python_output_text);
end
lines(end + 1, 1) = "";
lines(end + 1, 1) = "STATUSTEXT:";
if isempty(summary.status_texts)
    lines(end + 1, 1) = "  (нет сообщений)";
else
    for idx = 1:numel(summary.status_texts)
        lines(end + 1, 1) = "  " + local_text_value(summary.status_texts(idx));
    end
end

lines(ismissing(lines)) = "";
text_value = strjoin(lines, newline) + newline;
end

function value = local_param_value(param_struct, field_name)
%LOCAL_PARAM_VALUE Прочитать значение параметра из структуры Python-результата.

value = nan;
if isstruct(param_struct) && isfield(param_struct, field_name)
    candidate = double(param_struct.(field_name));
    if isfinite(candidate)
        value = candidate;
    end
end
end

function vec = local_message_vector(message_struct, field_name, component_names)
%LOCAL_MESSAGE_VECTOR Прочитать вектор из последнего MAVLink-сообщения.

vec = nan(numel(component_names), 1);
if ~isstruct(message_struct) || ~isfield(message_struct, field_name)
    return;
end

msg = message_struct.(field_name);
for idx = 1:numel(component_names)
    component_name = component_names{idx};
    if isstruct(msg) && isfield(msg, component_name)
        value = double(msg.(component_name));
        if isfinite(value)
            vec(idx) = value;
        end
    end
end
end

function count_value = local_message_count(message_counts, field_name)
%LOCAL_MESSAGE_COUNT Прочитать число полученных MAVLink-сообщений.

count_value = 0;
if isstruct(message_counts) && isfield(message_counts, field_name)
    count_value = double(message_counts.(field_name));
end
end

function value = local_diff_norm(vec_a, vec_b)
%LOCAL_DIFF_NORM Норма разности двух векторов.

if any(~isfinite(vec_a)) || any(~isfinite(vec_b))
    value = nan;
else
    value = norm(double(vec_a(:) - vec_b(:)));
end
end

function value = local_struct_scalar(data_struct, field_name, default_value)
%LOCAL_STRUCT_SCALAR Прочитать скалярное поле структуры.

value = default_value;
if isstruct(data_struct) && isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    value = data_struct.(field_name);
end
end

function value = local_struct_string(data_struct, field_name, default_value)
%LOCAL_STRUCT_STRING Прочитать строковое поле структуры.

value = string(default_value);
if isstruct(data_struct) && isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    value = string(data_struct.(field_name));
end
end

function value = local_struct_field_or_default(data_struct, field_name, default_value)
%LOCAL_STRUCT_FIELD_OR_DEFAULT Прочитать произвольное поле структуры.

value = default_value;
if isstruct(data_struct) && isfield(data_struct, field_name) && ~isempty(data_struct.(field_name))
    value = data_struct.(field_name);
end
end

function merged = local_overlay_struct(base_struct, overlay_struct)
%LOCAL_OVERLAY_STRUCT Наложить поля overlay_struct на base_struct.

merged = base_struct;
field_names = fieldnames(overlay_struct);
for idx = 1:numel(field_names)
    field_name = field_names{idx};
    merged.(field_name) = overlay_struct.(field_name);
end
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

function literal = local_python_literal(text_value)
%LOCAL_PYTHON_LITERAL Подготовить строковый литерал Python.

text_value = string(text_value);
literal = strings(size(text_value));
for idx = 1:numel(text_value)
    value = replace(text_value(idx), "\", "\\");
    value = replace(value, "'", "\'");
    literal(idx) = "'" + value + "'";
end
end

function local_assign_base(var_name, value)
%LOCAL_ASSIGN_BASE Поместить переменную в base workspace.

assignin('base', var_name, value);
end

function local_cleanup_temp_files(path_list)
%LOCAL_CLEANUP_TEMP_FILES Удалить временные файлы.

for idx = 1:numel(path_list)
    local_delete_if_exists(path_list{idx});
end
end

function local_delete_if_exists(path_value)
%LOCAL_DELETE_IF_EXISTS Удалить файл при наличии.

if exist(path_value, 'file') == 2
    delete(path_value);
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

folder_path = fileparts(char(path_value));
if strlength(string(folder_path)) > 0 && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(char(path_value), 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task19:insinstances:OpenFile', ...
        'Не удалось открыть файл %s.', string(path_value));
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русский текст.

if logical(flag_value)
    text_value = 'да';
else
    text_value = 'нет';
end
end

function text_value = local_text_value(value)
%LOCAL_TEXT_VALUE Безопасно преобразовать значение к тексту без missing.

text_value = string(value);
if ismissing(text_value)
    text_value = "";
end
end

function local_stop_existing_sitl(distro_name, host_ip, mavlink_port)
%LOCAL_STOP_EXISTING_SITL Остановить экземпляр arducopter текущего стенда.

pattern = sprintf( ...
    'build/sitl/bin/arducopter.*JSON:%s.*udpclient:%s:%d', ...
    char(host_ip), ...
    char(host_ip), ...
    mavlink_port);
system(sprintf( ...
    'wsl -d %s -- bash -lc "pkill -f ''%s'' >/dev/null 2>&1 || true"', ...
    char(distro_name), pattern)); %#ok<NASGU>
pause(1.0);
end
