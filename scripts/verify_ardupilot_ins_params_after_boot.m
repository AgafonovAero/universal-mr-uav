%% VERIFY_ARDUPILOT_INS_PARAMS_AFTER_BOOT Проверить реальные параметры ИНС.
% Назначение:
%   Для нескольких профилей запуска `ArduPilot JSON + MATLAB-модель`
%   поднимает устойчивый обмен, читает параметры `INS_*` и связанные
%   параметры через `MAVLink` после загрузки и сравнивает их с ожидаемыми
%   значениями из `.parm`-файлов.
%
% Входы:
%   none
%
% Выходы:
%   task_20_ins_params_after_boot - структура результата в base workspace
%
% Единицы измерения:
%   параметры и счетчики - безразмерные;
%   время - секунды
%
% Допущения:
%   `ArduPilot` уже собран, `pymavlink` доступен внутри `WSL`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

log_path = fullfile(logs_dir, 'task_20_verify_ins_params_after_boot.txt');
csv_path = fullfile(reports_dir, 'task_20_ins_params_after_boot.csv');
mat_path = fullfile(reports_dir, 'task_20_ins_params_after_boot.mat');

cfg = uav.ardupilot.default_json_config();

profiles = [ ...
    struct('name', "default", 'parm_path', ""), ...
    struct('name', "task19_single_imu", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task19_single_imu_diagnostic.parm')), ...
    struct('name', "task20_enable_mask", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm'))];

rows = repmat(local_empty_row(), numel(profiles), 1);
for idx = 1:numel(profiles)
    rows(idx) = local_run_profile(cfg, profiles(idx), repo_root, logs_dir);
end

result = struct();
result.rows = rows;
result.table = struct2table(rows);
save(mat_path, 'result');
writetable(result.table, csv_path);
assignin('base', 'task_20_ins_params_after_boot', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(rows));

fprintf('Проверка фактических параметров ИНС после загрузки TASK-20\n');
for idx = 1:numel(rows)
    fprintf('  %-24s baseline=%s INS_ENABLE_MASK=%.0f INS_USE2=%.0f INS_USE3=%.0f\n', ...
        char(rows(idx).profile_name), ...
        local_bool_text(rows(idx).baseline_restored), ...
        rows(idx).INS_ENABLE_MASK, ...
        rows(idx).INS_USE2, ...
        rows(idx).INS_USE3);
end

function row = local_run_profile(cfg, profile, repo_root, logs_dir)
parm_text = string(profile.parm_path);
if strlength(parm_text) > 0 && ~isfile(parm_text)
    error('uav:task20:verifyIns:MissingParm', ...
        'Не найден файл параметров профиля %s: %s', ...
        char(profile.name), char(parm_text));
end

baseline_mat_tmp = [tempname, '_task20_verify_ins.mat'];
baseline_csv_tmp = [tempname, '_task20_verify_ins.csv'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp})); %#ok<NASGU>

local_assign_base('ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, "task_20_verify_" + profile.name + "_baseline.txt"));
local_assign_base('ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, "task_20_verify_" + profile.name + "_wait.txt"));
local_assign_base('ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, "task_20_verify_" + profile.name + "_handshake.txt"));
local_assign_base('ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, "task_20_verify_" + profile.name + "_live.txt"));
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
if strlength(parm_text) > 0
    local_assign_base('ardupilot_task15_extra_defaults_win_path', char(parm_text));
end
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_task15_baseline_diag_log_path'', ' ...
    '''ardupilot_task15_baseline_wait_log_path'', ' ...
    '''ardupilot_task15_baseline_handshake_log_path'', ' ...
    '''ardupilot_task15_baseline_live_log_path'', ' ...
    '''ardupilot_task15_baseline_mat_report_path'', ' ...
    '''ardupilot_task15_baseline_csv_report_path'', ' ...
    '''ardupilot_task15_extra_defaults_win_path'');'])); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m'));
baseline = evalin('base', 'task_17_task15_baseline_exchange');

param_result = local_fetch_params(cfg);
uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);

requested = local_parse_parm_file(parm_text);
row = local_empty_row();
row.profile_name = string(profile.name);
row.parm_path = parm_text;
row.baseline_restored = logical(baseline.baseline_restored);
row.valid_rx_count = double(baseline.metrics.valid_rx_count);
row.response_tx_count = double(baseline.metrics.response_tx_count);
row.last_frame_count = double(baseline.metrics.last_frame_count);
row.heartbeat_received = logical(param_result.heartbeat_received);
row.failure_reason = string(param_result.failure_reason);
row.requested_INS_USE = local_requested_value(requested, 'INS_USE');
row.requested_INS_USE2 = local_requested_value(requested, 'INS_USE2');
row.requested_INS_USE3 = local_requested_value(requested, 'INS_USE3');
row.requested_INS_ENABLE_MASK = local_requested_value(requested, 'INS_ENABLE_MASK');
row.INS_USE = local_param_value(param_result.param_values, 'INS_USE');
row.INS_USE2 = local_param_value(param_result.param_values, 'INS_USE2');
row.INS_USE3 = local_param_value(param_result.param_values, 'INS_USE3');
row.INS_ENABLE_MASK = local_param_value(param_result.param_values, 'INS_ENABLE_MASK');
row.INS_ACC_ID = local_param_value(param_result.param_values, 'INS_ACC_ID');
row.INS_ACC2_ID = local_param_value(param_result.param_values, 'INS_ACC2_ID');
row.INS_ACC3_ID = local_param_value(param_result.param_values, 'INS_ACC3_ID');
row.INS_GYR_ID = local_param_value(param_result.param_values, 'INS_GYR_ID');
row.INS_GYR2_ID = local_param_value(param_result.param_values, 'INS_GYR2_ID');
row.INS_GYR3_ID = local_param_value(param_result.param_values, 'INS_GYR3_ID');
row.SIM_RATE_HZ = local_param_value(param_result.param_values, 'SIM_RATE_HZ');
row.SCHED_LOOP_RATE = local_param_value(param_result.param_values, 'SCHED_LOOP_RATE');
row.LOG_DISARMED = local_param_value(param_result.param_values, 'LOG_DISARMED');
row.ARMING_CHECK = local_param_value(param_result.param_values, 'ARMING_CHECK');
row.SIM_IMU_COUNT = local_param_value(param_result.param_values, 'SIM_IMU_COUNT');
end

function param_result = local_fetch_params(cfg)
json_tmp_path = [tempname, '_task20_verify_params.json'];
python_tmp_path = [tempname, '_task20_verify_params.py'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({json_tmp_path, python_tmp_path})); %#ok<NASGU>

param_names = [ ...
    "INS_USE"; "INS_USE2"; "INS_USE3"; "INS_ENABLE_MASK"; ...
    "INS_ACC_ID"; "INS_ACC2_ID"; "INS_ACC3_ID"; ...
    "INS_GYR_ID"; "INS_GYR2_ID"; "INS_GYR3_ID"; ...
    "SIM_RATE_HZ"; "SCHED_LOOP_RATE"; "LOG_DISARMED"; ...
    "ARMING_CHECK"; "SIM_IMU_COUNT"];

python_path_wsl = uav.ardupilot.windows_to_wsl_path(python_tmp_path);
json_path_wsl = uav.ardupilot.windows_to_wsl_path(json_tmp_path);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);
param_block = join("    '" + replace(param_names, "'", "\\'") + "',", newline);

lines = strings(0, 1);
lines(end + 1, 1) = "from pymavlink import mavutil";
lines(end + 1, 1) = "import json";
lines(end + 1, 1) = "import math";
lines(end + 1, 1) = "from pathlib import Path";
lines(end + 1, 1) = "OUTPUT_PATH = Path(" + local_py(json_path_wsl) + ")";
lines(end + 1, 1) = "PARAM_NAMES = [";
lines(end + 1, 1) = param_block;
lines(end + 1, 1) = "]";
lines(end + 1, 1) = "result = {'heartbeat_received': False, 'failure_reason': '', 'param_values': {}}";
lines(end + 1, 1) = "try:";
lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=245, source_component=190)";
lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
lines(end + 1, 1) = "    if hb is None:";
lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT по tcp:5763.')";
lines(end + 1, 1) = "    result['heartbeat_received'] = True";
lines(end + 1, 1) = "    for name in PARAM_NAMES:";
lines(end + 1, 1) = "        try:";
lines(end + 1, 1) = "            master.param_fetch_one(name)";
lines(end + 1, 1) = "        except Exception:";
lines(end + 1, 1) = "            pass";
lines(end + 1, 1) = "    deadline = __import__('time').time() + 10.0";
lines(end + 1, 1) = "    while __import__('time').time() < deadline and len(result['param_values']) < len(PARAM_NAMES):";
lines(end + 1, 1) = "        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)";
lines(end + 1, 1) = "        if msg is None:";
lines(end + 1, 1) = "            continue";
lines(end + 1, 1) = "        name = str(getattr(msg, 'param_id', '')).rstrip('\\x00')";
lines(end + 1, 1) = "        if name:";
lines(end + 1, 1) = "            value = float(getattr(msg, 'param_value', float('nan')))";
lines(end + 1, 1) = "            result['param_values'][name] = value if math.isfinite(value) else None";
lines(end + 1, 1) = "except Exception as exc:";
lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";

uav.ardupilot.write_utf8_text_file(python_tmp_path, strjoin(lines, newline) + newline);
command = sprintf('wsl -d %s -- bash -lc ''%s %s''', ...
    char(cfg.wsl_distro_name), ...
    char(python_command_wsl), ...
    char(python_path_wsl));
[status_code, output_text] = system(command); %#ok<ASGLU>

if ~isfile(json_tmp_path)
    error('uav:task20:verifyIns:ParamReadFailed', ...
        'Не удалось прочитать параметры ИНС после загрузки. Вывод: %s', ...
        char(string(output_text)));
end
param_result = jsondecode(fileread(json_tmp_path));
end

function requested = local_parse_parm_file(parm_path)
requested = struct();
parm_path = string(parm_path);
if strlength(parm_path) == 0 || ~isfile(parm_path)
    return;
end

lines = splitlines(string(fileread(parm_path)));
for idx = 1:numel(lines)
    line_text = strtrim(lines(idx));
    if line_text == "" || startsWith(line_text, "#")
        continue;
    end
    parts = split(line_text, ",");
    if numel(parts) < 2
        continue;
    end
    name = matlab.lang.makeValidName(char(strtrim(parts(1))));
    value = str2double(strtrim(parts(2)));
    if isfinite(value)
        requested.(name) = value;
    end
end
end

function value = local_requested_value(requested, name)
key = matlab.lang.makeValidName(name);
if isfield(requested, key)
    value = double(requested.(key));
else
    value = nan;
end
end

function value = local_param_value(param_values, name)
if isfield(param_values, name) && ~isempty(param_values.(name))
    value = double(param_values.(name));
else
    value = nan;
end
end

function row = local_empty_row()
row = struct( ...
    'profile_name', "", ...
    'parm_path', "", ...
    'baseline_restored', false, ...
    'valid_rx_count', 0, ...
    'response_tx_count', 0, ...
    'last_frame_count', 0, ...
    'heartbeat_received', false, ...
    'failure_reason', "", ...
    'requested_INS_USE', nan, ...
    'requested_INS_USE2', nan, ...
    'requested_INS_USE3', nan, ...
    'requested_INS_ENABLE_MASK', nan, ...
    'INS_USE', nan, ...
    'INS_USE2', nan, ...
    'INS_USE3', nan, ...
    'INS_ENABLE_MASK', nan, ...
    'INS_ACC_ID', nan, ...
    'INS_ACC2_ID', nan, ...
    'INS_ACC3_ID', nan, ...
    'INS_GYR_ID', nan, ...
    'INS_GYR2_ID', nan, ...
    'INS_GYR3_ID', nan, ...
    'SIM_RATE_HZ', nan, ...
    'SCHED_LOOP_RATE', nan, ...
    'LOG_DISARMED', nan, ...
    'ARMING_CHECK', nan, ...
    'SIM_IMU_COUNT', nan);
end

function local_assign_base(name, value)
assignin('base', name, value);
end

function text_value = local_make_log_text(rows)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-20: проверка параметров ИНС после загрузки ArduPilot";
lines(end + 1, 1) = "============================================================";
for idx = 1:numel(rows)
    row = rows(idx);
    lines(end + 1, 1) = "Профиль: " + row.profile_name;
    lines(end + 1, 1) = "  parm_path: " + row.parm_path;
    lines(end + 1, 1) = "  baseline_restored: " + local_bool_text(row.baseline_restored);
    lines(end + 1, 1) = "  valid_rx_count: " + string(row.valid_rx_count);
    lines(end + 1, 1) = "  response_tx_count: " + string(row.response_tx_count);
    lines(end + 1, 1) = "  last_frame_count: " + string(row.last_frame_count);
    lines(end + 1, 1) = "  heartbeat_received: " + local_bool_text(row.heartbeat_received);
    lines(end + 1, 1) = "  failure_reason: " + string(row.failure_reason);
    lines(end + 1, 1) = "  INS_USE: " + string(row.INS_USE);
    lines(end + 1, 1) = "  INS_USE2: " + string(row.INS_USE2);
    lines(end + 1, 1) = "  INS_USE3: " + string(row.INS_USE3);
    lines(end + 1, 1) = "  INS_ENABLE_MASK: " + string(row.INS_ENABLE_MASK);
    lines(end + 1, 1) = "  SIM_IMU_COUNT: " + string(row.SIM_IMU_COUNT);
    lines(end + 1, 1) = "";
end
text_value = strjoin(lines, newline) + newline;
end

function literal = local_py(text_value)
literal = "'" + replace(string(text_value), "'", "\\'") + "'";
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    if isfile(file_list{idx})
        delete(file_list{idx});
    end
end
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
