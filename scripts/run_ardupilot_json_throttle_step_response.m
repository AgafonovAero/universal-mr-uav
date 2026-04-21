%% RUN_ARDUPILOT_JSON_THROTTLE_STEP_RESPONSE Выполнить диагностический опыт с шагами газа.
% Назначение:
%   После устойчивого JSON/UDP-обмена выполняет arm и серию ступенчатых
%   команд газа по RC override, чтобы проверить способность ArduPilot
%   вывести ШИМ выше idle без использования команды `takeoff`.
%
% Входы:
%   none
%
% Выходы:
%   task_23_throttle_step_response - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   частота вращения - рад/с;
%   высота - метры.
%
% Допущения:
%   Используется тот же профиль TASK-22, что и для GUIDED takeoff.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
log_path = fullfile(logs_dir, 'task_23_throttle_step_response.txt');
csv_path = fullfile(reports_dir, 'task_23_throttle_step_response.csv');
mat_path = fullfile(reports_dir, 'task_23_throttle_step_response.mat');
parm_path = local_select_parm_path(repo_root);

local_prepare_parent(log_path);
local_prepare_parent(csv_path);

cfg = uav.ardupilot.default_json_config();
params = uav.sim.default_params_quad_x250();
cleanup_stop = onCleanup(@() uav.ardupilot.stop_existing_sitl( ...
    cfg.wsl_distro_name, ...
    string(cfg.udp_remote_ip), ...
    cfg.mavlink_udp_port)); %#ok<NASGU>

precalibration = local_run_precalibration(repo_root, cfg);
if ~precalibration.calibration_ready
    result = struct();
    result.executed = false;
    result.parm_path = string(parm_path);
    result.precalibration = precalibration;
    result.failure_reason = string(precalibration.failure_reason);
    result.history_table = table();
    save(mat_path, 'result');
    writetable(table(string(result.failure_reason), 'VariableNames', {'failure_reason'}), csv_path);
    uav.ardupilot.write_utf8_text_file(log_path, ...
        "TASK-23: throttle step response не выполнен." + newline + ...
        "Причина: " + string(result.failure_reason) + newline);
    error('uav:task23:throttle:PrecalibrationFailed', ...
        'Не подтверждена простая калибровка акселерометра перед throttle step: %s', char(result.failure_reason));
end

baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_23_throttle");
if ~baseline.baseline_restored
    result = struct();
    result.executed = false;
    result.parm_path = string(parm_path);
    result.baseline = baseline;
    result.failure_reason = string(baseline.first_failure_reason);
    result.history_table = table();
    save(mat_path, 'result');
    writetable(table(string(result.failure_reason), 'VariableNames', {'failure_reason'}), csv_path);
    uav.ardupilot.write_utf8_text_file(log_path, ...
        "TASK-23: throttle step response не выполнен." + newline + ...
        "Причина: " + string(result.failure_reason) + newline);
    error('uav:task23:throttle:BaselineFailed', ...
        'Не восстановлен устойчивый обмен перед throttle step: %s', char(result.failure_reason));
end

helper = local_prepare_throttle_helper(cfg);
cleanup_tmp = onCleanup(@() local_cleanup_temp({ ...
    helper.python_script_path_win, ...
    helper.launcher_script_path_win, ...
    helper.output_json_path_win, ...
    helper.output_log_path_win})); %#ok<NASGU>

system(helper.launch_command); %#ok<ASGLU>
assignin('base', 'ardupilot_live_backend_duration_s', 34.0);
cleanup_base = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
run(fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m'));
live_result = evalin('base', 'ardupilot_json_udp_live_backend');
metrics = uav.ardupilot.summarize_live_backend_metrics(live_result);
helper_result = local_read_helper_result(helper.output_json_path_win, helper.output_log_path_win);
history_table = local_make_history_table(live_result, helper_result.telemetry_table, params);

result = struct();
result.executed = true;
result.parm_path = string(parm_path);
result.precalibration = precalibration;
result.baseline = baseline;
result.helper_result = helper_result;
result.live_result = live_result;
result.metrics = metrics;
result.history_table = history_table;
result.arm_succeeded = helper_result.arm_succeeded;
result.command_ack = helper_result.arm_ack_result;
if result.arm_succeeded
    result.failure_reason = "";
else
    result.failure_reason = string(local_pick_failure_reason(helper_result));
end
result.max_total_thrust_to_weight = max(history_table.total_thrust_to_weight, [], 'omitnan');
result.max_altitude_m = max(history_table.altitude_m, [], 'omitnan');
result.motor_pwm_min_us = min(history_table{:, ["pwm_1_us","pwm_2_us","pwm_3_us","pwm_4_us"]}, [], 'all', 'omitnan');
result.motor_pwm_max_us = max(history_table{:, ["pwm_1_us","pwm_2_us","pwm_3_us","pwm_4_us"]}, [], 'all', 'omitnan');
result.motor_cmd_min_radps = min(history_table{:, ["motor_1_radps","motor_2_radps","motor_3_radps","motor_4_radps"]}, [], 'all', 'omitnan');
result.motor_cmd_max_radps = max(history_table{:, ["motor_1_radps","motor_2_radps","motor_3_radps","motor_4_radps"]}, [], 'all', 'omitnan');

writetable(history_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_23_throttle_step_response', result);
log_text = local_make_log_text(result);
if ismissing(log_text) || strlength(strtrim(log_text)) == 0
    log_text = "TASK-23: throttle step response" + newline + ...
        "Причина: " + local_empty_as_none(result.failure_reason) + newline;
end
uav.ardupilot.write_utf8_text_file(log_path, log_text);

fprintf('TASK-23: throttle step response\n');
fprintf('  arm accepted                           : %s\n', local_bool_text(result.arm_succeeded));
fprintf('  COMMAND_ACK                            : %.0f\n', result.command_ack);
fprintf('  max PWM [us]                           : %.0f\n', result.motor_pwm_max_us);
fprintf('  max motor command [rad/s]              : %.3f\n', result.motor_cmd_max_radps);

function parm_path = local_select_parm_path(repo_root)
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_200.parm');
if evalin('base', 'exist(''task23_active_parm_path'', ''var'')')
    candidate = string(evalin('base', 'task23_active_parm_path'));
    if strlength(candidate) > 0
        parm_path = char(candidate);
    end
end
if ~isfile(parm_path)
    error('uav:task23:throttle:MissingParm', 'Не найден профиль TASK-23: %s', parm_path);
end
end

function baseline = local_run_baseline(repo_root, logs_dir, parm_path, tag)
cfg_local = uav.ardupilot.default_json_config();
tag_char = char(tag);
baseline_mat_tmp = [tempname, '_', tag_char, '_baseline.mat'];
baseline_csv_tmp = [tempname, '_', tag_char, '_baseline.csv'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp})); %#ok<NASGU>

assignin('base', 'ardupilot_task15_extra_defaults_win_path', parm_path);
assignin('base', 'ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, [tag_char, '_baseline.txt']));
assignin('base', 'ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, [tag_char, '_wait.txt']));
assignin('base', 'ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, [tag_char, '_handshake.txt']));
assignin('base', 'ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, [tag_char, '_live.txt']));
assignin('base', 'ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
assignin('base', 'ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
assignin('base', 'ardupilot_live_backend_duration_s', 20.0);
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

function precalibration = local_run_precalibration(repo_root, cfg)
precalibration = struct( ...
    'calibration_ready', false, ...
    'selected_method', "", ...
    'command_ack', nan, ...
    'failure_reason', "", ...
    'output_excerpt', "");

if evalin('base', 'exist(''task23_precalibration_result'', ''var'')')
    cached = evalin('base', 'task23_precalibration_result');
    if isstruct(cached) && isfield(cached, 'calibration_ready')
        precalibration = cached;
        return;
    end
end

run(fullfile(repo_root, 'scripts', 'run_ardupilot_json_accel_simple_calibration.m'));
if ~evalin('base', 'exist(''task_21_json_accel_simple_calibration'', ''var'')')
    precalibration.failure_reason = "Не сформирован результат TASK-21 простой калибровки акселерометра.";
    assignin('base', 'task23_precalibration_result', precalibration);
    return;
end

calibration_result = evalin('base', 'task_21_json_accel_simple_calibration');
precalibration = local_extract_precalibration_result(calibration_result);
assignin('base', 'task23_precalibration_result', precalibration);
uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
pause(2.0);
end

function precalibration = local_extract_precalibration_result(calibration_result)
precalibration = struct( ...
    'calibration_ready', false, ...
    'selected_method', "", ...
    'command_ack', nan, ...
    'failure_reason', "", ...
    'output_excerpt', "");

if ~(isstruct(calibration_result) && isfield(calibration_result, 'attempt_table'))
    precalibration.failure_reason = "Результат TASK-21 не содержит таблицу попыток калибровки.";
    return;
end

attempt_table = calibration_result.attempt_table;
selected_method = string(local_get_field(calibration_result, 'selected_method', ""));
precalibration.selected_method = selected_method;

selected_idx = [];
if height(attempt_table) > 0 && strlength(selected_method) > 0
    selected_idx = find(string(attempt_table.method_name) == selected_method, 1, 'first');
end
if isempty(selected_idx) && height(attempt_table) > 0
    selected_idx = find(double(attempt_table.ack_result) == 0, 1, 'first');
end

if ~isempty(selected_idx)
    precalibration.command_ack = double(attempt_table.ack_result(selected_idx));
    precalibration.failure_reason = string(attempt_table.failure_reason(selected_idx));
    precalibration.output_excerpt = string(attempt_table.output_excerpt(selected_idx));
end

precalibration.calibration_ready = logical(local_get_field(calibration_result, 'simple_calibration_performed', false) || ...
    local_get_field(calibration_result, 'force_save_used', false) || ...
    any(double(attempt_table.ack_result) == 0));

if ~precalibration.calibration_ready && strlength(strtrim(precalibration.failure_reason)) == 0
    if height(attempt_table) > 0
        precalibration.failure_reason = string(attempt_table.failure_reason(end));
        if strlength(strtrim(precalibration.failure_reason)) == 0
            precalibration.failure_reason = string(attempt_table.output_excerpt(end));
        end
    else
        precalibration.failure_reason = "Простая калибровка акселерометра не подтверждена.";
    end
end
end

function helper = local_prepare_throttle_helper(cfg)
python_script_path_win = [tempname, '_task23_throttle.py'];
launcher_script_path_win = [tempname, '_task23_throttle.ps1'];
output_json_path_win = [tempname, '_task23_throttle.json'];
output_log_path_win = [tempname, '_task23_throttle.log'];

python_script_path_wsl = uav.ardupilot.windows_to_wsl_path(python_script_path_win);
output_json_path_wsl = uav.ardupilot.windows_to_wsl_path(output_json_path_win);
output_log_path_wsl = uav.ardupilot.windows_to_wsl_path(output_log_path_win);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);
step_values = [1100, 1300, 1500, 1700];

py_lines = strings(0, 1);
py_lines(end + 1, 1) = "from pymavlink import mavutil";
py_lines(end + 1, 1) = "import json";
py_lines(end + 1, 1) = "import math";
py_lines(end + 1, 1) = "import time";
py_lines(end + 1, 1) = "from pathlib import Path";
py_lines(end + 1, 1) = "OUTPUT_PATH = Path(" + local_py(output_json_path_wsl) + ")";
py_lines(end + 1, 1) = "STEP_VALUES = [1100, 1300, 1500, 1700]";
py_lines(end + 1, 1) = "result = {";
py_lines(end + 1, 1) = "    'heartbeat_received': False,";
py_lines(end + 1, 1) = "    'arm_succeeded': False,";
py_lines(end + 1, 1) = "    'arm_ack_result': None,";
py_lines(end + 1, 1) = "    'failure_reason': '',";
py_lines(end + 1, 1) = "    'status_texts': [],";
py_lines(end + 1, 1) = "    'telemetry': [],";
py_lines(end + 1, 1) = "    'applied_steps': []";
py_lines(end + 1, 1) = "}";
py_lines(end + 1, 1) = "current_rel_alt = None";
py_lines(end + 1, 1) = "current_armed = False";
py_lines(end + 1, 1) = "current_mode = ''";
py_lines(end + 1, 1) = "current_throttle_target = 0";
py_lines(end + 1, 1) = "last_sample_t = -1.0e9";
py_lines(end + 1, 1) = "start_t = time.time()";
py_lines(end + 1, 1) = "heartbeat_count = 0";
py_lines(end + 1, 1) = "global_position_count = 0";
py_lines(end + 1, 1) = "status_text_count = 0";
py_lines(end + 1, 1) = "def finite_or_none(value):";
py_lines(end + 1, 1) = "    try:";
py_lines(end + 1, 1) = "        numeric = float(value)";
py_lines(end + 1, 1) = "    except Exception:";
py_lines(end + 1, 1) = "        return None";
py_lines(end + 1, 1) = "    return numeric if math.isfinite(numeric) else None";
py_lines(end + 1, 1) = "def record_sample(now_s):";
py_lines(end + 1, 1) = "    global last_sample_t";
py_lines(end + 1, 1) = "    if now_s - last_sample_t < 0.5:";
py_lines(end + 1, 1) = "        return";
py_lines(end + 1, 1) = "    result['telemetry'].append({";
py_lines(end + 1, 1) = "        't_s': finite_or_none(now_s - start_t),";
py_lines(end + 1, 1) = "        'armed': bool(current_armed),";
py_lines(end + 1, 1) = "        'mode': str(current_mode),";
py_lines(end + 1, 1) = "        'relative_alt_m': finite_or_none(current_rel_alt),";
py_lines(end + 1, 1) = "        'throttle_target_us': int(current_throttle_target),";
py_lines(end + 1, 1) = "        'heartbeat_count': int(heartbeat_count),";
py_lines(end + 1, 1) = "        'global_position_count': int(global_position_count),";
py_lines(end + 1, 1) = "        'status_text_count': int(status_text_count)";
py_lines(end + 1, 1) = "    })";
py_lines(end + 1, 1) = "    last_sample_t = now_s";
py_lines(end + 1, 1) = "def process_message(master, msg):";
py_lines(end + 1, 1) = "    global current_rel_alt, current_armed, current_mode, heartbeat_count, global_position_count, status_text_count";
py_lines(end + 1, 1) = "    if msg is None:";
py_lines(end + 1, 1) = "        return";
py_lines(end + 1, 1) = "    mtype = msg.get_type()";
py_lines(end + 1, 1) = "    now_s = time.time()";
py_lines(end + 1, 1) = "    if mtype == 'HEARTBEAT':";
py_lines(end + 1, 1) = "        heartbeat_count += 1";
py_lines(end + 1, 1) = "        current_armed = bool(int(msg.base_mode) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))";
py_lines(end + 1, 1) = "        try:";
py_lines(end + 1, 1) = "            current_mode = mavutil.mode_string_v10(msg)";
py_lines(end + 1, 1) = "        except Exception:";
py_lines(end + 1, 1) = "            current_mode = ''";
py_lines(end + 1, 1) = "    elif mtype == 'GLOBAL_POSITION_INT':";
py_lines(end + 1, 1) = "        global_position_count += 1";
py_lines(end + 1, 1) = "        current_rel_alt = float(msg.relative_alt) / 1000.0";
py_lines(end + 1, 1) = "    elif mtype == 'STATUSTEXT':";
py_lines(end + 1, 1) = "        text = str(getattr(msg, 'text', '')).strip()";
py_lines(end + 1, 1) = "        if text:";
py_lines(end + 1, 1) = "            result['status_texts'].append(text)";
py_lines(end + 1, 1) = "            status_text_count = len(result['status_texts'])";
py_lines(end + 1, 1) = "    elif mtype == 'COMMAND_ACK' and int(getattr(msg, 'command', -1)) == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):";
py_lines(end + 1, 1) = "        result['arm_ack_result'] = int(getattr(msg, 'result', -1))";
py_lines(end + 1, 1) = "    record_sample(now_s)";
py_lines(end + 1, 1) = "def send_override(master, throttle_us):";
py_lines(end + 1, 1) = "    master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, int(throttle_us), 65535, 65535, 65535, 65535, 65535)";
py_lines(end + 1, 1) = "try:";
py_lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=245, source_component=190)";
py_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
py_lines(end + 1, 1) = "    if hb is None:";
py_lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT по tcp:127.0.0.1:5763.')"; 
py_lines(end + 1, 1) = "    result['heartbeat_received'] = True";
py_lines(end + 1, 1) = "    process_message(master, hb)";
py_lines(end + 1, 1) = "    warmup_deadline = time.time() + 2.0";
py_lines(end + 1, 1) = "    while time.time() < warmup_deadline:";
py_lines(end + 1, 1) = "        process_message(master, master.recv_match(blocking=True, timeout=1))";
py_lines(end + 1, 1) = "    try:";
py_lines(end + 1, 1) = "        master.set_mode_apm('ALT_HOLD')";
py_lines(end + 1, 1) = "    except Exception:";
py_lines(end + 1, 1) = "        pass";
py_lines(end + 1, 1) = "    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)";
py_lines(end + 1, 1) = "    arm_deadline = time.time() + 20.0";
py_lines(end + 1, 1) = "    while time.time() < arm_deadline:";
py_lines(end + 1, 1) = "        process_message(master, master.recv_match(blocking=True, timeout=1))";
py_lines(end + 1, 1) = "        if current_armed:";
py_lines(end + 1, 1) = "            result['arm_succeeded'] = True";
py_lines(end + 1, 1) = "            break";
py_lines(end + 1, 1) = "    if result['arm_succeeded']:";
py_lines(end + 1, 1) = "        for throttle_us in STEP_VALUES:";
py_lines(end + 1, 1) = "            current_throttle_target = int(throttle_us)";
py_lines(end + 1, 1) = "            result['applied_steps'].append(int(throttle_us))";
py_lines(end + 1, 1) = "            step_deadline = time.time() + 4.0";
py_lines(end + 1, 1) = "            while time.time() < step_deadline:";
py_lines(end + 1, 1) = "                send_override(master, throttle_us)";
py_lines(end + 1, 1) = "                inner_deadline = time.time() + 1.0";
py_lines(end + 1, 1) = "                while time.time() < inner_deadline:";
py_lines(end + 1, 1) = "                    process_message(master, master.recv_match(blocking=True, timeout=0.5))";
py_lines(end + 1, 1) = "        current_throttle_target = 0";
py_lines(end + 1, 1) = "        for _ in range(5):";
py_lines(end + 1, 1) = "            master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, 0, 65535, 65535, 65535, 65535, 65535)";
py_lines(end + 1, 1) = "            process_message(master, master.recv_match(blocking=True, timeout=0.5))";
py_lines(end + 1, 1) = "    else:";
py_lines(end + 1, 1) = "        if result['status_texts']:";
py_lines(end + 1, 1) = "            result['failure_reason'] = result['status_texts'][-1]";
py_lines(end + 1, 1) = "        elif result['arm_ack_result'] is not None:";
py_lines(end + 1, 1) = "            result['failure_reason'] = 'Команда arm отклонена с кодом {}'.format(result['arm_ack_result'])";
py_lines(end + 1, 1) = "        else:";
py_lines(end + 1, 1) = "            result['failure_reason'] = 'ArduPilot не подтвердил взведение перед throttle step.'";
py_lines(end + 1, 1) = "except Exception as exc:";
py_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
py_lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')";

uav.ardupilot.write_utf8_text_file(python_script_path_win, strjoin(py_lines, newline) + newline);

helper = struct();
helper.python_script_path_win = string(python_script_path_win);
helper.output_json_path_win = string(output_json_path_win);
helper.output_log_path_win = string(output_log_path_win);
helper_log_path_wsl = uav.ardupilot.windows_to_wsl_path(output_log_path_win);
bash_payload = "sleep 10.000; " + string(python_command_wsl) + " " + ...
    local_bash_quote(python_script_path_wsl) + " >" + local_bash_quote(helper_log_path_wsl) + " 2>&1";
launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$ArgumentList = @('-d','" + string(cfg.wsl_distro_name) + "','--','bash','-lc'," + local_ps_quote(bash_payload) + ")";
launcher_lines(end + 1, 1) = "Start-Process -WindowStyle Hidden -FilePath 'wsl.exe' -ArgumentList $ArgumentList | Out-Null";
uav.ardupilot.write_utf8_text_file(launcher_script_path_win, strjoin(launcher_lines, newline) + newline);

helper.launcher_script_path_win = string(launcher_script_path_win);
helper.launch_command = sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', launcher_script_path_win);
helper.step_values_us = step_values;
end

function helper_result = local_read_helper_result(json_path_win, log_path_win)
deadline = tic;
while toc(deadline) < 50.0
    if isfile(char(json_path_win))
        break;
    end
    pause(0.2);
end

helper_result = struct();
helper_result.telemetry_table = table();
helper_result.heartbeat_received = false;
helper_result.arm_succeeded = false;
helper_result.arm_ack_result = nan;
helper_result.failure_reason = "";
helper_result.status_texts = strings(0, 1);
helper_result.applied_steps = [];

if isfile(char(json_path_win))
    decoded = jsondecode(fileread(char(json_path_win)));
    helper_result.heartbeat_received = logical(local_get_field(decoded, 'heartbeat_received', false));
    helper_result.arm_succeeded = logical(local_get_field(decoded, 'arm_succeeded', false));
    helper_result.arm_ack_result = double(local_get_field(decoded, 'arm_ack_result', nan));
    helper_result.failure_reason = string(local_get_field(decoded, 'failure_reason', ""));
    if isfield(decoded, 'status_texts') && ~isempty(decoded.status_texts)
        helper_result.status_texts = string(decoded.status_texts(:));
    end
    if isfield(decoded, 'applied_steps') && ~isempty(decoded.applied_steps)
        helper_result.applied_steps = double(decoded.applied_steps(:));
    end
    helper_result.telemetry_table = local_make_telemetry_table(local_get_field(decoded, 'telemetry', struct([])));
elseif isfile(char(log_path_win))
    helper_result.failure_reason = string(local_tail_excerpt(fileread(char(log_path_win))));
end
end

function telemetry_table = local_make_telemetry_table(telemetry)
if isempty(telemetry)
    telemetry_table = table();
    return;
end

row_count = numel(telemetry);
t_s = nan(row_count, 1);
armed = false(row_count, 1);
mode = strings(row_count, 1);
relative_alt_m = nan(row_count, 1);
throttle_target_us = nan(row_count, 1);
heartbeat_count = nan(row_count, 1);
global_position_count = nan(row_count, 1);
status_text_count = nan(row_count, 1);

for idx = 1:row_count
    item = telemetry(idx);
    t_s(idx) = double(local_get_field(item, 't_s', nan));
    armed(idx) = logical(local_get_field(item, 'armed', false));
    mode(idx) = string(local_get_field(item, 'mode', ""));
    relative_alt_m(idx) = double(local_get_field(item, 'relative_alt_m', nan));
    throttle_target_us(idx) = double(local_get_field(item, 'throttle_target_us', nan));
    heartbeat_count(idx) = double(local_get_field(item, 'heartbeat_count', nan));
    global_position_count(idx) = double(local_get_field(item, 'global_position_count', nan));
    status_text_count(idx) = double(local_get_field(item, 'status_text_count', nan));
end

telemetry_table = table( ...
    t_s, armed, mode, relative_alt_m, throttle_target_us, ...
    heartbeat_count, global_position_count, status_text_count);
end

function table_value = local_make_history_table(live_result, telemetry_table, params)
n = numel(live_result.time_s);
mode_hist = strings(n, 1);
armed_hist = false(n, 1);
throttle_target_us = zeros(n, 1);
time_s = double(live_result.time_s(:));
valid_rx_count = cumsum(double(live_result.udp_valid_rx_count_hist(:)));
response_flags = arrayfun(@(item) double(logical(item.tx_ok && item.rx_valid)), live_result.exchange_diag(:));
response_tx_count = cumsum(response_flags(:));

if ~isempty(telemetry_table)
    telem_time = double(telemetry_table.t_s(:));
    telem_armed = logical(telemetry_table.armed(:));
    telem_mode = string(telemetry_table.mode(:));
    telem_throttle = double(telemetry_table.throttle_target_us(:));
    idx = 1;
    current_mode = "";
    current_armed = false;
    current_throttle = 0.0;
    for k = 1:n
        while idx <= numel(telem_time) && telem_time(idx) <= time_s(k)
            current_mode = telem_mode(idx);
            current_armed = telem_armed(idx);
            current_throttle = telem_throttle(idx);
            idx = idx + 1;
        end
        mode_hist(k) = current_mode;
        armed_hist(k) = current_armed;
        throttle_target_us(k) = current_throttle;
    end
end

pwm_matrix = nan(n, 4);
frame_count = nan(n, 1);
for k = 1:n
    item = live_result.sitl_output(k);
    [frame_count(k), pwm_matrix(k, :)] = local_extract_pwm_frame(item);
end

motor_cmd = double(live_result.motor_cmd_radps);
[thrust_matrix_N, ~] = uav.vmg.rotor_simple( ...
    motor_cmd, ...
    params.rotor.kT_N_per_radps2, ...
    params.rotor.kQ_Nm_per_radps2);
total_thrust_N = sum(thrust_matrix_N, 2);
weight_N = double(params.mass_kg) * double(params.gravity_mps2);

altitude_m = nan(n, 1);
vertical_speed_up_mps = nan(n, 1);
roll_rad = nan(n, 1);
pitch_rad = nan(n, 1);
yaw_rad = nan(n, 1);
quat_norm = nan(n, 1);
for k = 1:n
    state_k = live_result.state(k);
    c_nb = uav.core.quat_to_dcm(double(state_k.q_nb(:)));
    v_ned_mps = c_nb * double(state_k.v_b_mps(:));
    altitude_m(k) = -double(state_k.p_ned_m(3));
    vertical_speed_up_mps(k) = -double(v_ned_mps(3));
    [roll_rad(k), pitch_rad(k), yaw_rad(k)] = local_quat_to_euler321(double(state_k.q_nb(:)));
    quat_norm(k) = norm(double(state_k.q_nb(:)));
end

table_value = table( ...
    time_s, ...
    valid_rx_count, ...
    response_tx_count, ...
    mode_hist, ...
    armed_hist, ...
    throttle_target_us, ...
    frame_count, ...
    pwm_matrix(:, 1), pwm_matrix(:, 2), pwm_matrix(:, 3), pwm_matrix(:, 4), ...
    motor_cmd(:, 1), motor_cmd(:, 2), motor_cmd(:, 3), motor_cmd(:, 4), ...
    total_thrust_N, ...
    total_thrust_N ./ weight_N, ...
    altitude_m, ...
    vertical_speed_up_mps, ...
    roll_rad, pitch_rad, yaw_rad, ...
    quat_norm, ...
    'VariableNames', { ...
        'time_s', 'valid_rx_count', 'response_tx_count', 'mode', 'armed', ...
        'throttle_target_us', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_1_radps', 'motor_2_radps', 'motor_3_radps', 'motor_4_radps', ...
        'total_thrust_N', 'total_thrust_to_weight', ...
        'altitude_m', 'vertical_speed_up_mps', ...
        'roll_rad', 'pitch_rad', 'yaw_rad', ...
        'quat_norm'});
end

function [roll_rad, pitch_rad, yaw_rad] = local_quat_to_euler321(q_nb)
q_nb = q_nb(:) ./ norm(q_nb);
q0 = q_nb(1);
q1 = q_nb(2);
q2 = q_nb(3);
q3 = q_nb(4);
roll_rad = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2));
sin_pitch = 2.0 * (q0*q2 - q3*q1);
sin_pitch = min(max(sin_pitch, -1.0), 1.0);
pitch_rad = asin(sin_pitch);
yaw_rad = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3));
end

function text_value = local_make_log_text(result)
helper = result.helper_result;
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-23: диагностический throttle step";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль запуска: " + string(result.parm_path);
lines(end + 1, 1) = "precalibration ready: " + local_bool_text(result.precalibration.calibration_ready);
lines(end + 1, 1) = "precalibration method: " + local_empty_as_none(result.precalibration.selected_method);
lines(end + 1, 1) = "precalibration ACK: " + string(result.precalibration.command_ack);
lines(end + 1, 1) = "baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "valid_rx_count during throttle step: " + string(result.metrics.valid_rx_count);
lines(end + 1, 1) = "response_tx_count during throttle step: " + string(result.metrics.response_tx_count);
lines(end + 1, 1) = "arm succeeded: " + local_bool_text(result.arm_succeeded);
lines(end + 1, 1) = "COMMAND_ACK: " + string(result.command_ack);
lines(end + 1, 1) = "failure reason: " + local_empty_as_none(result.failure_reason);
if ~isempty(helper.applied_steps)
    lines(end + 1, 1) = "applied throttle steps [us]: " + strjoin(string(helper.applied_steps), ", ");
end
if ~isempty(helper.status_texts)
    tail = helper.status_texts(max(1, numel(helper.status_texts)-7):end);
    lines(end + 1, 1) = "STATUSTEXT tail: " + strjoin(string(tail), " | ");
end
lines(end + 1, 1) = sprintf('motor_pwm_us range: %.6f .. %.6f', result.motor_pwm_min_us, result.motor_pwm_max_us);
lines(end + 1, 1) = sprintf('motor_cmd_radps range: %.6f .. %.6f', result.motor_cmd_min_radps, result.motor_cmd_max_radps);
lines(end + 1, 1) = sprintf('max total thrust / weight: %.6f', result.max_total_thrust_to_weight);
lines(end + 1, 1) = sprintf('max altitude [m]: %.6f', result.max_altitude_m);
text_value = strjoin(lines, newline) + newline;
end

function value = local_get_field(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name) && ~isempty(data.(field_name))
    value = data.(field_name);
else
    value = default_value;
end
end

function text_value = local_pick_failure_reason(helper_result)
if isfield(helper_result, 'status_texts') && ~isempty(helper_result.status_texts)
    status_texts = string(helper_result.status_texts(:));
    match_idx = find(startsWith(status_texts, "Arm:") | startsWith(status_texts, "PreArm:"), 1, 'first');
    if ~isempty(match_idx)
        text_value = status_texts(match_idx);
        return;
    end
end
if strlength(string(helper_result.failure_reason)) > 0
    text_value = string(helper_result.failure_reason);
elseif ~isempty(helper_result.status_texts)
    text_value = string(helper_result.status_texts(end));
else
    text_value = "";
end
end

function text_value = local_tail_excerpt(raw_text)
lines = splitlines(string(raw_text));
lines = strtrim(lines);
lines(lines == "") = [];
if isempty(lines)
    text_value = "";
else
    text_value = strjoin(lines(max(1, numel(lines)-7):end), " | ");
end
end

function literal = local_py(text_value)
literal = "'" + replace(string(text_value), "'", "\\'") + "'";
end

function local_prepare_parent(path_value)
parent_dir = fileparts(path_value);
if strlength(string(parent_dir)) > 0 && ~isfolder(parent_dir)
    mkdir(parent_dir);
end
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    path_value = file_list{idx};
    if isfile(path_value)
        delete(path_value);
    end
end
end

function [frame_value, pwm_row] = local_extract_pwm_frame(item)
frame_value = nan;
pwm_row = nan(1, 4);

if isstruct(item) && isfield(item, 'frame_count') && ~isempty(item.frame_count)
    frame_value = double(item.frame_count);
end

if ~(isstruct(item) && isfield(item, 'motor_pwm_us')) || isempty(item.motor_pwm_us)
    return;
end

raw_pwm = double(item.motor_pwm_us(:));
copy_count = min(4, numel(raw_pwm));
pwm_row(1:copy_count) = raw_pwm(1:copy_count).';
end

function quoted_value = local_bash_quote(value)
quoted_value = "'" + string(value) + "'";
end

function quoted_value = local_ps_quote(value)
quoted_value = "'" + replace(string(value), "'", "''") + "'";
end


function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function text_value = local_empty_as_none(value)
value = string(value);
if strlength(strtrim(value)) == 0
    text_value = "нет";
else
    text_value = value;
end
end
