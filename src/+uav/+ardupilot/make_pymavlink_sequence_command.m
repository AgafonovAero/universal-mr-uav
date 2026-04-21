function command_info = make_pymavlink_sequence_command(seq_cfg)
%MAKE_PYMAVLINK_SEQUENCE_COMMAND Подготовить сценарий pymavlink для MAVLink-команд.
% Назначение:
%   Формирует временный Python-сценарий с использованием `pymavlink`,
%   который подключается к ArduPilot, ожидает HEARTBEAT, при необходимости
%   задает режим, выполняет одну или несколько попыток arm и сохраняет
%   структурированный результат в JSON-файл.
%
% Входы:
%   seq_cfg - структура конфигурации сценария pymavlink
%
% Выходы:
%   command_info - структура с путями временного Python-сценария и файла
%                  результата JSON
%
% Единицы измерения:
%   интервалы ожидания задаются в секундах;
%   целевая высота takeoff задается в метрах
%
% Допущения:
%   `pymavlink` уже установлен в выбранной среде выполнения.

seq_cfg = local_normalize_cfg(seq_cfg);

python_path_win = string(seq_cfg.python_script_path);
output_json_path_win = string(seq_cfg.output_json_path);

switch seq_cfg.runtime
    case "wsl"
        python_path_runtime = local_windows_to_wsl_path(python_path_win);
        output_json_path_runtime = local_windows_to_wsl_path(output_json_path_win);
    case "windows"
        python_path_runtime = python_path_win;
        output_json_path_runtime = output_json_path_win;
    otherwise
        error( ...
            'uav:ardupilot:make_pymavlink_sequence_command:BadRuntime', ...
            'Неподдерживаемое окружение выполнения: %s', ...
            char(seq_cfg.runtime));
end

local_write_utf8_text(python_path_win, local_make_python_text(seq_cfg, output_json_path_runtime));

command_info = struct();
command_info.runtime = seq_cfg.runtime;
command_info.wsl_distro_name = string(seq_cfg.wsl_distro_name);
command_info.python_script_path_win = python_path_win;
command_info.python_script_path_wsl = string(python_path_runtime);
command_info.output_json_path_win = output_json_path_win;
command_info.output_json_path_wsl = string(output_json_path_runtime);

switch seq_cfg.runtime
    case "wsl"
        command_info.command_text = sprintf( ...
            'wsl.exe -d "%s" -- bash --noprofile --norc -lc "python3 %s"', ...
            char(seq_cfg.wsl_distro_name), ...
            char(python_path_runtime));
    case "windows"
        command_info.command_text = sprintf('python "%s"', char(python_path_runtime));
end
end

function seq_cfg = local_normalize_cfg(seq_cfg)
%LOCAL_NORMALIZE_CFG Нормализовать конфигурацию сценария pymavlink.

if nargin < 1 || isempty(seq_cfg)
    seq_cfg = struct();
end

if ~isstruct(seq_cfg) || ~isscalar(seq_cfg)
    error( ...
        'uav:ardupilot:make_pymavlink_sequence_command:CfgType', ...
        'Ожидалась скалярная структура seq_cfg.');
end

defaults = struct();
defaults.wsl_distro_name = string(uav.ardupilot.default_json_config().wsl_distro_name);
defaults.connection_candidates = ["tcp:127.0.0.1:5760"; "tcp:127.0.0.1:5762"];
defaults.mode_name = "GUIDED";
defaults.wait_before_command_s = 0.0;
defaults.heartbeat_timeout_s = 20.0;
defaults.arm_timeout_s = 20.0;
defaults.monitor_duration_s = 20.0;
defaults.takeoff_alt_m = nan;
defaults.sample_period_s = 0.5;
defaults.arm_attempt_times_s = [];
defaults.python_script_path = string(tempname) + ".py";
defaults.output_json_path = string(tempname) + ".json";
defaults.runtime = "wsl";

field_names = fieldnames(defaults);
for idx = 1:numel(field_names)
    field_name = field_names{idx};
    if ~isfield(seq_cfg, field_name) || isempty(seq_cfg.(field_name))
        seq_cfg.(field_name) = defaults.(field_name);
    end
end

seq_cfg.wsl_distro_name = string(seq_cfg.wsl_distro_name);
seq_cfg.connection_candidates = string(seq_cfg.connection_candidates(:));
seq_cfg.mode_name = string(seq_cfg.mode_name);
seq_cfg.python_script_path = string(seq_cfg.python_script_path);
seq_cfg.output_json_path = string(seq_cfg.output_json_path);
seq_cfg.runtime = lower(string(seq_cfg.runtime));
seq_cfg.arm_attempt_times_s = reshape(double(seq_cfg.arm_attempt_times_s), 1, []);

validateattributes(seq_cfg.wait_before_command_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'seq_cfg.wait_before_command_s');
validateattributes(seq_cfg.heartbeat_timeout_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'seq_cfg.heartbeat_timeout_s');
validateattributes(seq_cfg.arm_timeout_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'seq_cfg.arm_timeout_s');
validateattributes(seq_cfg.monitor_duration_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'seq_cfg.monitor_duration_s');
validateattributes(seq_cfg.sample_period_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'seq_cfg.sample_period_s');
validateattributes(seq_cfg.takeoff_alt_m, {'numeric'}, ...
    {'real', 'scalar'}, ...
    mfilename, 'seq_cfg.takeoff_alt_m');
validateattributes(seq_cfg.arm_attempt_times_s, {'numeric'}, ...
    {'real', 'vector', 'nonnegative'}, ...
    mfilename, 'seq_cfg.arm_attempt_times_s');
end

function text_value = local_make_python_text(seq_cfg, output_json_path_runtime)
%LOCAL_MAKE_PYTHON_TEXT Сформировать текст временного Python-сценария.

candidate_list = join("    " + local_python_literal(seq_cfg.connection_candidates) + ",", newline);
mode_literal = local_python_literal(seq_cfg.mode_name);

if isnan(seq_cfg.takeoff_alt_m)
    takeoff_literal = "None";
else
    takeoff_literal = sprintf('%.6f', double(seq_cfg.takeoff_alt_m));
end

if isempty(seq_cfg.arm_attempt_times_s)
    arm_attempts_literal = "";
else
    arm_attempts_literal = join(compose('%.6f', seq_cfg.arm_attempt_times_s), ", ");
end

python_lines = strings(0, 1);
python_lines(end + 1, 1) = "from pymavlink import mavutil";
python_lines(end + 1, 1) = "import json";
python_lines(end + 1, 1) = "import math";
python_lines(end + 1, 1) = "import time";
python_lines(end + 1, 1) = "from pathlib import Path";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "OUTPUT_PATH = Path(" + local_python_literal(output_json_path_runtime) + ")";
python_lines(end + 1, 1) = "CANDIDATES = [";
python_lines(end + 1, 1) = candidate_list;
python_lines(end + 1, 1) = "]";
python_lines(end + 1, 1) = "MODE_NAME = " + mode_literal;
python_lines(end + 1, 1) = "WAIT_BEFORE_COMMAND_S = " + sprintf('%.6f', double(seq_cfg.wait_before_command_s));
python_lines(end + 1, 1) = "HEARTBEAT_TIMEOUT_S = " + sprintf('%.6f', double(seq_cfg.heartbeat_timeout_s));
python_lines(end + 1, 1) = "ARM_TIMEOUT_S = " + sprintf('%.6f', double(seq_cfg.arm_timeout_s));
python_lines(end + 1, 1) = "MONITOR_DURATION_S = " + sprintf('%.6f', double(seq_cfg.monitor_duration_s));
python_lines(end + 1, 1) = "TAKEOFF_ALT_M = " + takeoff_literal;
python_lines(end + 1, 1) = "SAMPLE_PERIOD_S = " + sprintf('%.6f', double(seq_cfg.sample_period_s));
python_lines(end + 1, 1) = "ARM_ATTEMPT_TIMES_S = [" + arm_attempts_literal + "]";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "result = {";
python_lines(end + 1, 1) = "    'attempted': True,";
python_lines(end + 1, 1) = "    'method': 'pymavlink',";
python_lines(end + 1, 1) = "    'heartbeat_received': False,";
python_lines(end + 1, 1) = "    'connection_string': '',";
python_lines(end + 1, 1) = "    'connection_attempts': [],";
python_lines(end + 1, 1) = "    'system_id': 0,";
python_lines(end + 1, 1) = "    'component_id': 0,";
python_lines(end + 1, 1) = "    'requested_mode': str(MODE_NAME),";
python_lines(end + 1, 1) = "    'mode_command_sent': False,";
python_lines(end + 1, 1) = "    'arm_requested': False,";
python_lines(end + 1, 1) = "    'arm_succeeded': False,";
python_lines(end + 1, 1) = "    'arm_ack_result': None,";
python_lines(end + 1, 1) = "    'takeoff_requested': False,";
python_lines(end + 1, 1) = "    'takeoff_ack_result': None,";
python_lines(end + 1, 1) = "    'failure_reason': '',";
python_lines(end + 1, 1) = "    'status_texts': [],";
python_lines(end + 1, 1) = "    'status_text_events': [],";
python_lines(end + 1, 1) = "    'arm_attempts': [],";
python_lines(end + 1, 1) = "    'command_acks': [],";
python_lines(end + 1, 1) = "    'telemetry': [],";
python_lines(end + 1, 1) = "    'heartbeat_count': 0,";
python_lines(end + 1, 1) = "    'global_position_count': 0,";
python_lines(end + 1, 1) = "    'status_text_count': 0,";
python_lines(end + 1, 1) = "    'max_relative_alt_m': None,";
python_lines(end + 1, 1) = "    'height_changed': False,";
python_lines(end + 1, 1) = "    'last_sys_status': None,";
python_lines(end + 1, 1) = "    'last_ekf_status_report': None,";
python_lines(end + 1, 1) = "    'last_ahrs2': None,";
python_lines(end + 1, 1) = "    'last_attitude': None,";
python_lines(end + 1, 1) = "    'last_local_position_ned': None,";
python_lines(end + 1, 1) = "    'last_gps_raw_int': None,";
python_lines(end + 1, 1) = "    'elapsed_s': 0.0";
python_lines(end + 1, 1) = "}";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "current_rel_alt = None";
python_lines(end + 1, 1) = "current_armed = False";
python_lines(end + 1, 1) = "current_mode = ''";
python_lines(end + 1, 1) = "last_sample_t = -1.0e9";
python_lines(end + 1, 1) = "start_t = time.time()";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def finite_or_none(value):";
python_lines(end + 1, 1) = "    try:";
python_lines(end + 1, 1) = "        numeric = float(value)";
python_lines(end + 1, 1) = "    except Exception:";
python_lines(end + 1, 1) = "        return None";
python_lines(end + 1, 1) = "    if math.isfinite(numeric):";
python_lines(end + 1, 1) = "        return numeric";
python_lines(end + 1, 1) = "    return None";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def append_status(text_value):";
python_lines(end + 1, 1) = "    text_value = str(text_value).strip()";
python_lines(end + 1, 1) = "    if not text_value:";
python_lines(end + 1, 1) = "        return";
python_lines(end + 1, 1) = "    result['status_texts'].append(text_value)";
python_lines(end + 1, 1) = "    result['status_text_events'].append({'t_s': finite_or_none(time.time() - start_t), 'text': text_value})";
python_lines(end + 1, 1) = "    result['status_text_count'] = len(result['status_texts'])";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def recent_status_texts(window_s):";
python_lines(end + 1, 1) = "    cutoff = finite_or_none(time.time() - start_t - window_s)";
python_lines(end + 1, 1) = "    if cutoff is None:";
python_lines(end + 1, 1) = "        return []";
python_lines(end + 1, 1) = "    items = []";
python_lines(end + 1, 1) = "    for item in result['status_text_events']:";
python_lines(end + 1, 1) = "        t_s = finite_or_none(item.get('t_s'))";
python_lines(end + 1, 1) = "        if t_s is not None and t_s >= cutoff:";
python_lines(end + 1, 1) = "            items.append(str(item.get('text', '')).strip())";
python_lines(end + 1, 1) = "    return items";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def record_sample(now_s):";
python_lines(end + 1, 1) = "    global last_sample_t";
python_lines(end + 1, 1) = "    if now_s - last_sample_t < SAMPLE_PERIOD_S:";
python_lines(end + 1, 1) = "        return";
python_lines(end + 1, 1) = "    result['telemetry'].append({";
python_lines(end + 1, 1) = "        't_s': finite_or_none(now_s - start_t),";
python_lines(end + 1, 1) = "        'armed': bool(current_armed),";
python_lines(end + 1, 1) = "        'mode': str(current_mode),";
python_lines(end + 1, 1) = "        'relative_alt_m': finite_or_none(current_rel_alt),";
python_lines(end + 1, 1) = "        'heartbeat_count': int(result['heartbeat_count']),";
python_lines(end + 1, 1) = "        'global_position_count': int(result['global_position_count']),";
python_lines(end + 1, 1) = "        'status_text_count': int(result['status_text_count'])";
python_lines(end + 1, 1) = "    })";
python_lines(end + 1, 1) = "    last_sample_t = now_s";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "def process_message(master, msg):";
python_lines(end + 1, 1) = "    global current_rel_alt, current_armed, current_mode";
python_lines(end + 1, 1) = "    if msg is None:";
python_lines(end + 1, 1) = "        return";
python_lines(end + 1, 1) = "    msg_type = msg.get_type()";
python_lines(end + 1, 1) = "    now_s = time.time()";
python_lines(end + 1, 1) = "    if msg_type == 'HEARTBEAT':";
python_lines(end + 1, 1) = "        result['heartbeat_count'] += 1";
python_lines(end + 1, 1) = "        current_armed = bool(int(msg.base_mode) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))";
python_lines(end + 1, 1) = "        try:";
python_lines(end + 1, 1) = "            current_mode = mavutil.mode_string_v10(msg)";
python_lines(end + 1, 1) = "        except Exception:";
python_lines(end + 1, 1) = "            current_mode = ''";
python_lines(end + 1, 1) = "    elif msg_type == 'GLOBAL_POSITION_INT':";
python_lines(end + 1, 1) = "        result['global_position_count'] += 1";
python_lines(end + 1, 1) = "        current_rel_alt = float(msg.relative_alt) / 1000.0";
python_lines(end + 1, 1) = "        if result['max_relative_alt_m'] is None or current_rel_alt > result['max_relative_alt_m']:";
python_lines(end + 1, 1) = "            result['max_relative_alt_m'] = current_rel_alt";
python_lines(end + 1, 1) = "    elif msg_type == 'VFR_HUD' and current_rel_alt is None:";
python_lines(end + 1, 1) = "        current_rel_alt = finite_or_none(getattr(msg, 'alt', None))";
python_lines(end + 1, 1) = "    elif msg_type == 'STATUSTEXT':";
python_lines(end + 1, 1) = "        append_status(getattr(msg, 'text', ''))";
python_lines(end + 1, 1) = "    elif msg_type == 'SYS_STATUS':";
python_lines(end + 1, 1) = "        result['last_sys_status'] = {";
python_lines(end + 1, 1) = "            'onboard_control_sensors_health': int(getattr(msg, 'onboard_control_sensors_health', 0)),";
python_lines(end + 1, 1) = "            'load': finite_or_none(getattr(msg, 'load', None)),";
python_lines(end + 1, 1) = "            'voltage_battery': finite_or_none(getattr(msg, 'voltage_battery', None)),";
python_lines(end + 1, 1) = "            'current_battery': finite_or_none(getattr(msg, 'current_battery', None))";
python_lines(end + 1, 1) = "        }";
python_lines(end + 1, 1) = "    elif msg_type == 'EKF_STATUS_REPORT':";
python_lines(end + 1, 1) = "        result['last_ekf_status_report'] = {";
python_lines(end + 1, 1) = "            'flags': int(getattr(msg, 'flags', 0)),";
python_lines(end + 1, 1) = "            'velocity_variance': finite_or_none(getattr(msg, 'velocity_variance', None)),";
python_lines(end + 1, 1) = "            'pos_horiz_variance': finite_or_none(getattr(msg, 'pos_horiz_variance', None)),";
python_lines(end + 1, 1) = "            'pos_vert_variance': finite_or_none(getattr(msg, 'pos_vert_variance', None)),";
python_lines(end + 1, 1) = "            'compass_variance': finite_or_none(getattr(msg, 'compass_variance', None)),";
python_lines(end + 1, 1) = "            'terrain_alt_variance': finite_or_none(getattr(msg, 'terrain_alt_variance', None))";
python_lines(end + 1, 1) = "        }";
python_lines(end + 1, 1) = "    elif msg_type == 'AHRS2':";
python_lines(end + 1, 1) = "        result['last_ahrs2'] = {";
python_lines(end + 1, 1) = "            'roll': finite_or_none(getattr(msg, 'roll', None)),";
python_lines(end + 1, 1) = "            'pitch': finite_or_none(getattr(msg, 'pitch', None)),";
python_lines(end + 1, 1) = "            'yaw': finite_or_none(getattr(msg, 'yaw', None)),";
python_lines(end + 1, 1) = "            'altitude': finite_or_none(getattr(msg, 'altitude', None))";
python_lines(end + 1, 1) = "        }";
python_lines(end + 1, 1) = "    elif msg_type == 'ATTITUDE':";
python_lines(end + 1, 1) = "        result['last_attitude'] = {";
python_lines(end + 1, 1) = "            'roll': finite_or_none(getattr(msg, 'roll', None)),";
python_lines(end + 1, 1) = "            'pitch': finite_or_none(getattr(msg, 'pitch', None)),";
python_lines(end + 1, 1) = "            'yaw': finite_or_none(getattr(msg, 'yaw', None)),";
python_lines(end + 1, 1) = "            'rollspeed': finite_or_none(getattr(msg, 'rollspeed', None)),";
python_lines(end + 1, 1) = "            'pitchspeed': finite_or_none(getattr(msg, 'pitchspeed', None)),";
python_lines(end + 1, 1) = "            'yawspeed': finite_or_none(getattr(msg, 'yawspeed', None))";
python_lines(end + 1, 1) = "        }";
python_lines(end + 1, 1) = "    elif msg_type == 'LOCAL_POSITION_NED':";
python_lines(end + 1, 1) = "        result['last_local_position_ned'] = {";
python_lines(end + 1, 1) = "            'x': finite_or_none(getattr(msg, 'x', None)),";
python_lines(end + 1, 1) = "            'y': finite_or_none(getattr(msg, 'y', None)),";
python_lines(end + 1, 1) = "            'z': finite_or_none(getattr(msg, 'z', None)),";
python_lines(end + 1, 1) = "            'vx': finite_or_none(getattr(msg, 'vx', None)),";
python_lines(end + 1, 1) = "            'vy': finite_or_none(getattr(msg, 'vy', None)),";
python_lines(end + 1, 1) = "            'vz': finite_or_none(getattr(msg, 'vz', None))";
python_lines(end + 1, 1) = "        }";
python_lines(end + 1, 1) = "    elif msg_type == 'GPS_RAW_INT':";
python_lines(end + 1, 1) = "        result['last_gps_raw_int'] = {";
python_lines(end + 1, 1) = "            'fix_type': int(getattr(msg, 'fix_type', 0)),";
python_lines(end + 1, 1) = "            'satellites_visible': int(getattr(msg, 'satellites_visible', 0)),";
python_lines(end + 1, 1) = "            'lat': finite_or_none(getattr(msg, 'lat', None)),";
python_lines(end + 1, 1) = "            'lon': finite_or_none(getattr(msg, 'lon', None)),";
python_lines(end + 1, 1) = "            'alt': finite_or_none(getattr(msg, 'alt', None))";
python_lines(end + 1, 1) = "        }";
python_lines(end + 1, 1) = "    elif msg_type == 'COMMAND_ACK':";
python_lines(end + 1, 1) = "        command_id = int(getattr(msg, 'command', -1))";
python_lines(end + 1, 1) = "        ack_result = int(getattr(msg, 'result', -1))";
python_lines(end + 1, 1) = "        result['command_acks'].append({'command': command_id, 'result': ack_result})";
python_lines(end + 1, 1) = "        if command_id == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):";
python_lines(end + 1, 1) = "            result['arm_ack_result'] = ack_result";
python_lines(end + 1, 1) = "        elif command_id == int(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):";
python_lines(end + 1, 1) = "            result['takeoff_ack_result'] = ack_result";
python_lines(end + 1, 1) = "    record_sample(now_s)";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "master = None";
python_lines(end + 1, 1) = "try:";
python_lines(end + 1, 1) = "    for candidate in CANDIDATES:";
python_lines(end + 1, 1) = "        attempt = {'connection_string': str(candidate), 'heartbeat_received': False, 'error': ''}";
python_lines(end + 1, 1) = "        try:";
python_lines(end + 1, 1) = "            master = mavutil.mavlink_connection(candidate, source_system=245, source_component=190, autoreconnect=False)";
python_lines(end + 1, 1) = "            heartbeat = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)";
python_lines(end + 1, 1) = "            if heartbeat is None:";
python_lines(end + 1, 1) = "                raise RuntimeError('Не получен HEARTBEAT.')";
python_lines(end + 1, 1) = "            result['heartbeat_received'] = True";
python_lines(end + 1, 1) = "            result['connection_string'] = str(candidate)";
python_lines(end + 1, 1) = "            result['system_id'] = int(heartbeat.get_srcSystem())";
python_lines(end + 1, 1) = "            result['component_id'] = int(heartbeat.get_srcComponent())";
python_lines(end + 1, 1) = "            attempt['heartbeat_received'] = True";
python_lines(end + 1, 1) = "            result['connection_attempts'].append(attempt)";
python_lines(end + 1, 1) = "            process_message(master, heartbeat)";
python_lines(end + 1, 1) = "            break";
python_lines(end + 1, 1) = "        except Exception as exc:";
python_lines(end + 1, 1) = "            attempt['error'] = str(exc)";
python_lines(end + 1, 1) = "            result['connection_attempts'].append(attempt)";
python_lines(end + 1, 1) = "            master = None";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "    if master is None or not result['heartbeat_received']:";
python_lines(end + 1, 1) = "        result['failure_reason'] = 'Не удалось установить MAVLink-соединение с ArduPilot.'";
python_lines(end + 1, 1) = "    else:";
python_lines(end + 1, 1) = "        try:";
python_lines(end + 1, 1) = "            master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)";
python_lines(end + 1, 1) = "        except Exception:";
python_lines(end + 1, 1) = "            pass";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        warmup_deadline = time.time() + WAIT_BEFORE_COMMAND_S";
python_lines(end + 1, 1) = "        while time.time() < warmup_deadline:";
python_lines(end + 1, 1) = "            process_message(master, master.recv_match(blocking=True, timeout=1))";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        if MODE_NAME:";
python_lines(end + 1, 1) = "            try:";
python_lines(end + 1, 1) = "                master.set_mode_apm(str(MODE_NAME))";
python_lines(end + 1, 1) = "                result['mode_command_sent'] = True";
python_lines(end + 1, 1) = "            except Exception as exc:";
python_lines(end + 1, 1) = "                append_status(f'Ошибка задания режима {MODE_NAME}: {exc}')";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        attempt_times = ARM_ATTEMPT_TIMES_S if ARM_ATTEMPT_TIMES_S else [WAIT_BEFORE_COMMAND_S]";
python_lines(end + 1, 1) = "        for attempt_idx, attempt_time_s in enumerate(attempt_times, start=1):";
python_lines(end + 1, 1) = "            target_time = start_t + float(attempt_time_s)";
python_lines(end + 1, 1) = "            while time.time() < target_time:";
python_lines(end + 1, 1) = "                process_message(master, master.recv_match(blocking=True, timeout=1))";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "            result['arm_requested'] = True";
python_lines(end + 1, 1) = "            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)";
python_lines(end + 1, 1) = "            arm_deadline = time.time() + ARM_TIMEOUT_S";
python_lines(end + 1, 1) = "            attempt_arm_succeeded = False";
python_lines(end + 1, 1) = "            while time.time() < arm_deadline:";
python_lines(end + 1, 1) = "                process_message(master, master.recv_match(blocking=True, timeout=1))";
python_lines(end + 1, 1) = "                if current_armed:";
python_lines(end + 1, 1) = "                    attempt_arm_succeeded = True";
python_lines(end + 1, 1) = "                    result['arm_succeeded'] = True";
python_lines(end + 1, 1) = "                    break";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "            recent_status = recent_status_texts(30.0)";
python_lines(end + 1, 1) = "            attempt_failure_reason = ''";
python_lines(end + 1, 1) = "            if not attempt_arm_succeeded:";
python_lines(end + 1, 1) = "                if recent_status:";
python_lines(end + 1, 1) = "                    attempt_failure_reason = recent_status[-1]";
python_lines(end + 1, 1) = "                elif result['arm_ack_result'] is not None:";
python_lines(end + 1, 1) = "                    attempt_failure_reason = 'Команда взведения отклонена с кодом {}.'.format(result['arm_ack_result'])";
python_lines(end + 1, 1) = "                else:";
python_lines(end + 1, 1) = "                    attempt_failure_reason = 'ArduPilot не подтвердил состояние взведения.'";
python_lines(end + 1, 1) = "            result['arm_attempts'].append({";
python_lines(end + 1, 1) = "                'attempt_index': int(attempt_idx),";
python_lines(end + 1, 1) = "                'scheduled_time_s': finite_or_none(attempt_time_s),";
python_lines(end + 1, 1) = "                'heartbeat_received': bool(result['heartbeat_received']),";
python_lines(end + 1, 1) = "                'guided_seen': bool(current_mode == 'GUIDED' or 'GUIDED' in str(current_mode)),";
python_lines(end + 1, 1) = "                'arm_succeeded': bool(attempt_arm_succeeded),";
python_lines(end + 1, 1) = "                'arm_ack_result': result['arm_ack_result'],";
python_lines(end + 1, 1) = "                'failure_reason': str(attempt_failure_reason),";
python_lines(end + 1, 1) = "                'recent_status_texts': recent_status,";
python_lines(end + 1, 1) = "                'last_sys_status': result['last_sys_status'],";
python_lines(end + 1, 1) = "                'last_ekf_status_report': result['last_ekf_status_report'],";
python_lines(end + 1, 1) = "                'last_ahrs2': result['last_ahrs2'],";
python_lines(end + 1, 1) = "                'last_attitude': result['last_attitude'],";
python_lines(end + 1, 1) = "                'last_local_position_ned': result['last_local_position_ned'],";
python_lines(end + 1, 1) = "                'last_gps_raw_int': result['last_gps_raw_int']";
python_lines(end + 1, 1) = "            })";
python_lines(end + 1, 1) = "            if attempt_arm_succeeded:";
python_lines(end + 1, 1) = "                break";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        if result['arm_succeeded'] and TAKEOFF_ALT_M is not None:";
python_lines(end + 1, 1) = "            result['takeoff_requested'] = True";
python_lines(end + 1, 1) = "            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, float('nan'), 0, 0, float(TAKEOFF_ALT_M))";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        monitor_deadline = time.time() + MONITOR_DURATION_S";
python_lines(end + 1, 1) = "        while time.time() < monitor_deadline:";
python_lines(end + 1, 1) = "            process_message(master, master.recv_match(blocking=True, timeout=1))";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        if result['max_relative_alt_m'] is not None and result['max_relative_alt_m'] > 0.3:";
python_lines(end + 1, 1) = "            result['height_changed'] = True";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "        if not result['arm_succeeded']:";
python_lines(end + 1, 1) = "            if result['arm_attempts']:";
python_lines(end + 1, 1) = "                result['failure_reason'] = str(result['arm_attempts'][0].get('failure_reason', ''))";
python_lines(end + 1, 1) = "            elif result['status_texts']:";
python_lines(end + 1, 1) = "                result['failure_reason'] = result['status_texts'][-1]";
python_lines(end + 1, 1) = "            elif result['arm_ack_result'] is not None:";
python_lines(end + 1, 1) = "                result['failure_reason'] = 'Команда взведения отклонена с кодом {}.'.format(result['arm_ack_result'])";
python_lines(end + 1, 1) = "            else:";
python_lines(end + 1, 1) = "                result['failure_reason'] = 'ArduPilot не подтвердил состояние взведения.'";
python_lines(end + 1, 1) = "except Exception as exc:";
python_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
python_lines(end + 1, 1) = "";
python_lines(end + 1, 1) = "result['elapsed_s'] = finite_or_none(time.time() - start_t)";
python_lines(end + 1, 1) = "OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)";
python_lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";
python_lines(end + 1, 1) = "print(json.dumps({'output_path': str(OUTPUT_PATH), 'arm_succeeded': bool(result['arm_succeeded']), 'failure_reason': str(result['failure_reason'])}, ensure_ascii=False))";

text_value = strjoin(python_lines, newline) + newline;
end

function literal = local_python_literal(text_values)
%LOCAL_PYTHON_LITERAL Сформировать строковый литерал Python.

text_values = string(text_values);
literal = strings(size(text_values));

for idx = 1:numel(text_values)
    value = replace(text_values(idx), "\", "\\");
    value = replace(value, "'", "\'");
    literal(idx) = "'" + value + "'";
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

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

path_value = string(path_value);
folder_path = fileparts(char(path_value));
if strlength(string(folder_path)) > 0 && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(char(path_value), 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:ardupilot:make_pymavlink_sequence_command:OpenFile', ...
        'Не удалось открыть файл %s.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end
