%% RUN_ARDUPILOT_JSON_ARM_AFTER_ACCEL_LOG_FIX Повторить arm после исправления ИНС.
% Назначение:
%   Запускает устойчивый режим `ArduPilot JSON + MATLAB-модель` с выбранным
%   параметрическим исправлением по акселерометрам, подтверждает
%   фактические параметры ИНС после загрузки и выполняет повторную попытку
%   взведения.
%
% Входы:
%   none
%
% Выходы:
%   task_20_arm_after_accel_log_fix - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   В качестве минимального исправления применяется параметрический
%   профиль `INS_ENABLE_MASK`, подтвержденный внутренними журналами
%   `ArduPilot`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

log_path = fullfile(logs_dir, 'task_20_arm_after_accel_log_fix.txt');
csv_path = fullfile(reports_dir, 'task_20_arm_after_accel_log_fix.csv');
mat_path = fullfile(reports_dir, 'task_20_arm_after_accel_log_fix.mat');
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');

if ~isfile(parm_path)
    error('uav:task20:armfix:MissingParm', ...
        'Не найден профиль параметров исправления: %s', parm_path);
end

cfg = uav.ardupilot.default_json_config();
baseline_mat_tmp = [tempname, '_task20_arm_fix_baseline.mat'];
baseline_csv_tmp = [tempname, '_task20_arm_fix_baseline.csv'];
arm_mat_tmp = [tempname, '_task20_arm_fix_response.mat'];
arm_csv_tmp = [tempname, '_task20_arm_fix_response.csv'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp, arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

local_assign_base('ardupilot_task15_extra_defaults_win_path', parm_path);
local_assign_base('ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, 'task_20_arm_fix_baseline.txt'));
local_assign_base('ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, 'task_20_arm_fix_wait.txt'));
local_assign_base('ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, 'task_20_arm_fix_handshake.txt'));
local_assign_base('ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, 'task_20_arm_fix_live.txt'));
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
local_assign_base('ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
local_assign_base('ardupilot_live_backend_duration_s', 40.0);
local_assign_base('ardupilot_arm_attempt_delay_s', 30.0);
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_task15_extra_defaults_win_path'', ' ...
    '''ardupilot_task15_baseline_diag_log_path'', ' ...
    '''ardupilot_task15_baseline_wait_log_path'', ' ...
    '''ardupilot_task15_baseline_handshake_log_path'', ' ...
    '''ardupilot_task15_baseline_live_log_path'', ' ...
    '''ardupilot_task15_baseline_mat_report_path'', ' ...
    '''ardupilot_task15_baseline_csv_report_path'', ' ...
    '''ardupilot_arm_pwm_response_mat_path'', ' ...
    '''ardupilot_arm_pwm_response_csv_path'', ' ...
    '''ardupilot_live_backend_duration_s'', ' ...
    '''ardupilot_arm_attempt_delay_s'');'])); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m'));
baseline = evalin('base', 'task_17_task15_baseline_exchange');
param_snapshot = local_fetch_params(cfg);

response = struct();
if baseline.baseline_restored
    local_assign_base('ardupilot_live_backend_duration_s', 40.0);
    local_assign_base('ardupilot_arm_attempt_delay_s', 30.0);
    run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
    response = evalin('base', 'ardupilot_arm_pwm_response');
end

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);

result = struct();
result.parm_path = string(parm_path);
result.baseline = baseline;
result.param_snapshot = param_snapshot;
result.arm_response = response;
result.fix_profile_applied = local_param_match(param_snapshot, 'INS_ENABLE_MASK', 1.0) ...
    && local_param_match(param_snapshot, 'INS_USE2', 0.0) ...
    && local_param_match(param_snapshot, 'INS_USE3', 0.0);
result.arm_succeeded = false;
result.command_ack = nan;
result.failure_reason = string(baseline.first_failure_reason);
result.motor_pwm_us_range = [nan nan];
result.motor_cmd_radps_range = [nan nan];
result.valid_rx_count = double(baseline.metrics.valid_rx_count);
result.response_tx_count = double(baseline.metrics.response_tx_count);

if ~isempty(fieldnames(response))
    result.arm_succeeded = logical(response.arm_attempt.arm_succeeded);
    result.command_ack = double(response.arm_attempt.ack_result);
    result.failure_reason = string(response.arm_attempt.failure_reason);
    result.motor_pwm_us_range = local_range(response.live_result.sitl_output, 'motor_pwm_us');
    result.motor_cmd_radps_range = local_numeric_range(response.live_result.motor_cmd_radps);
    result.valid_rx_count = double(response.live_result.valid_rx_count);
    result.response_tx_count = double(response.live_result.response_tx_count);
end

save(mat_path, 'result');
writetable(local_make_result_table(result), csv_path);
assignin('base', 'task_20_arm_after_accel_log_fix', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('Повторная попытка arm после исправления TASK-20\n');
fprintf('  fix profile applied                    : %s\n', local_bool_text(result.fix_profile_applied));
fprintf('  valid_rx_count                         : %d\n', result.valid_rx_count);
fprintf('  response_tx_count                      : %d\n', result.response_tx_count);
fprintf('  arm succeeded                          : %s\n', local_bool_text(result.arm_succeeded));
fprintf('  command_ack                            : %.0f\n', result.command_ack);
fprintf('  failure reason                         : %s\n', char(result.failure_reason));

function param_snapshot = local_fetch_params(cfg)
json_tmp_path = [tempname, '_task20_arm_fix_params.json'];
python_tmp_path = [tempname, '_task20_arm_fix_params.py'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({json_tmp_path, python_tmp_path})); %#ok<NASGU>

param_names = ["INS_USE"; "INS_USE2"; "INS_USE3"; "INS_ENABLE_MASK"; "LOG_DISARMED"];
param_block = join("    '" + param_names + "',", newline);
python_path_wsl = uav.ardupilot.windows_to_wsl_path(python_tmp_path);
json_path_wsl = uav.ardupilot.windows_to_wsl_path(json_tmp_path);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);

lines = strings(0, 1);
lines(end + 1, 1) = "from pymavlink import mavutil";
lines(end + 1, 1) = "import json";
lines(end + 1, 1) = "from pathlib import Path";
lines(end + 1, 1) = "OUTPUT_PATH = Path('" + json_path_wsl + "')";
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
lines(end + 1, 1) = "        master.param_fetch_one(name)";
lines(end + 1, 1) = "    import time";
lines(end + 1, 1) = "    deadline = time.time() + 8.0";
lines(end + 1, 1) = "    while time.time() < deadline and len(result['param_values']) < len(PARAM_NAMES):";
lines(end + 1, 1) = "        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)";
lines(end + 1, 1) = "        if msg is None:";
lines(end + 1, 1) = "            continue";
lines(end + 1, 1) = "        name = str(getattr(msg, 'param_id', '')).rstrip('\\x00')";
lines(end + 1, 1) = "        result['param_values'][name] = float(getattr(msg, 'param_value', float('nan')))";
lines(end + 1, 1) = "except Exception as exc:";
lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";

uav.ardupilot.write_utf8_text_file(python_tmp_path, strjoin(lines, newline) + newline);
command = sprintf('wsl -d %s -- bash -lc ''%s %s''', ...
    char(cfg.wsl_distro_name), ...
    char(python_command_wsl), ...
    char(python_path_wsl));
[status_code, output_text] = system(command); %#ok<ASGLU>

if isfile(json_tmp_path)
    param_snapshot = jsondecode(fileread(json_tmp_path));
else
    param_snapshot = struct('heartbeat_received', false, 'failure_reason', string(output_text), 'param_values', struct());
end
end

function tf = local_param_match(snapshot, name, expected_value)
tf = false;
if isfield(snapshot, 'param_values') && isfield(snapshot.param_values, name)
    actual = double(snapshot.param_values.(name));
    tf = isfinite(actual) && abs(actual - expected_value) < 1.0e-6;
end
end

function range_value = local_range(packet_array, field_name)
values = [];
for idx = 1:numel(packet_array)
    if isfield(packet_array(idx), field_name)
        item = double(packet_array(idx).(field_name));
        item = item(isfinite(item));
        values = [values; item(:)]; %#ok<AGROW>
    end
end
range_value = local_numeric_range(values);
end

function range_value = local_numeric_range(values)
values = double(values(:));
values = values(isfinite(values));
if isempty(values)
    range_value = [nan nan];
else
    range_value = [min(values), max(values)];
end
end

function table_value = local_make_result_table(result)
table_value = table( ...
    string(result.parm_path), ...
    logical(result.fix_profile_applied), ...
    double(result.valid_rx_count), ...
    double(result.response_tx_count), ...
    logical(result.arm_succeeded), ...
    double(result.command_ack), ...
    string(result.failure_reason), ...
    double(result.motor_pwm_us_range(1)), ...
    double(result.motor_pwm_us_range(2)), ...
    double(result.motor_cmd_radps_range(1)), ...
    double(result.motor_cmd_radps_range(2)), ...
    'VariableNames', { ...
        'parm_path', ...
        'fix_profile_applied', ...
        'valid_rx_count', ...
        'response_tx_count', ...
        'arm_succeeded', ...
        'command_ack', ...
        'failure_reason', ...
        'motor_pwm_min_us', ...
        'motor_pwm_max_us', ...
        'motor_cmd_min_radps', ...
        'motor_cmd_max_radps'});
end

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-20: повторная попытка arm после исправления по акселерометрам";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль: " + string(result.parm_path);
lines(end + 1, 1) = "Профиль применен: " + local_bool_text(result.fix_profile_applied);
lines(end + 1, 1) = "Baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "Baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "valid_rx_count: " + string(result.valid_rx_count);
lines(end + 1, 1) = "response_tx_count: " + string(result.response_tx_count);
lines(end + 1, 1) = "Arm succeeded: " + local_bool_text(result.arm_succeeded);
lines(end + 1, 1) = "COMMAND_ACK: " + string(result.command_ack);
lines(end + 1, 1) = "Failure reason: " + string(result.failure_reason);
lines(end + 1, 1) = "motor_pwm_us range: [" + local_format_range(result.motor_pwm_us_range) + "]";
lines(end + 1, 1) = "motor_cmd_radps range: [" + local_format_range(result.motor_cmd_radps_range) + "]";
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_format_range(range_value)
if numel(range_value) ~= 2 || any(~isfinite(range_value))
    text_value = "NaN NaN";
else
    text_value = sprintf('%.6f %.6f', range_value(1), range_value(2));
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

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
