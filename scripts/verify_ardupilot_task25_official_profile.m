%% VERIFY_ARDUPILOT_TASK25_OFFICIAL_PROFILE
% Проверить применение профиля TASK-25 в официальном MATLAB backend ArduPilot.
%
% Назначение:
%   Запускает ArduPilot JSON SITL с официальным lockstep MATLAB backend,
%   заранее поднимает Windows-side pymavlink helper на UDP-портах 14550 и
%   14552, выполняет 20 секунд устойчивого обмена и затем фиксирует
%   фактически считанные параметры ArduPilot.
%
% Входы:
%   none
%
% Выходы:
%   task_25_official_profile_after_boot - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   частоты - герцы;
%   значения параметров - в единицах ArduPilot.
%
% Допущения:
%   На Windows установлен Python с pymavlink, а ArduPilot JSON SITL
%   запускается внутри WSL.

repo_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(repo_root, 'scripts', 'bootstrap_project.m'));

logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
if ~isfolder(logs_dir)
    mkdir(logs_dir);
end
if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_25_verify_official_profile.txt');
csv_path = fullfile(reports_dir, 'task_25_official_profile_after_boot.csv');
mat_path = fullfile(reports_dir, 'task_25_official_profile_after_boot.mat');
sitl_console_log = fullfile(logs_dir, 'task_25_verify_official_profile_arducopter_console.txt');

cfg = uav.ardupilot.default_json_config();
cfg.udp_timeout_s = 1.0;
cfg.physics_max_timestep_s = 1.0 / 50.0;
cfg.json_send_quaternion = false;
cfg.udp_remote_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
cfg.mavlink_udp_ip = cfg.udp_remote_ip;
cfg.mavlink_monitor_udp_port = 14552;

profile_win_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task25_official_matlab_json.parm');
param_names = [ ...
    "INS_USE"; "INS_USE2"; "INS_USE3"; "INS_ENABLE_MASK"; ...
    "LOG_DISARMED"; "SCHED_LOOP_RATE"; "AHRS_EKF_TYPE"; ...
    "GPS1_TYPE"; "GPS_TYPE"; "SIM_MAG1_DEVID"; ...
    "ARMING_CHECK"];

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port);
cleanup_sitl = onCleanup(@() uav.ardupilot.stop_existing_sitl( ...
    cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port)); %#ok<NASGU>

ctx = uav.ardupilot.json_lockstep_open(cfg);
cleanup_udp = onCleanup(@() uav.ardupilot.json_lockstep_close(ctx)); %#ok<NASGU>

launch_command = local_launch_sitl(cfg, profile_win_path, sitl_console_log);
[probe_info, cleanup_list] = local_prepare_param_probe(cfg, param_names);
cleanup_probe = onCleanup(@() local_cleanup_temp(cleanup_list)); %#ok<NASGU>
system(probe_info.launch_command); %#ok<ASGLU>

[ctx, first_frame, wait_elapsed_s] = local_wait_for_valid_frame(ctx, 15.0, cfg);

if ~first_frame.valid
    result = struct();
    result.executed = false;
    result.failure_reason = "Первый валидный пакет SITL не получен.";
    result.launch_command = string(launch_command);
    result.profile_win_path = string(profile_win_path);
    result.wait_elapsed_s = wait_elapsed_s;
    result.sitl_console_tail = local_read_text_tail(sitl_console_log, 120);
    save(mat_path, 'result');
    writetable(table(string(result.failure_reason), 'VariableNames', {'failure_reason'}), csv_path);
    uav.ardupilot.write_utf8_text_file(log_path, ...
        "TASK-25: проверка официального профиля" + newline + ...
        "Причина: " + string(result.failure_reason) + newline);
    error('uav:task25:verifyprofile:NoFirstFrame', ...
        'Первый валидный пакет SITL не получен.');
end

params = uav.sim.make_deterministic_demo_params();
state = uav.core.state_unpack(params.demo.initial_state_plant);
physics_time_s = 0.0;
duration_s = 20.0;
wall_deadline_s = 120.0;
current_frame = first_frame;
step_index = 0;
max_steps = 25000;

wall_tic = tic;
while physics_time_s < duration_s && toc(wall_tic) < wall_deadline_s && step_index < max_steps
    if current_frame.controller_reset
        state = uav.core.state_unpack(params.demo.initial_state_plant);
    end

    dt_s = min(1.0 / max(double(current_frame.frame_rate_hz), 1.0), cfg.physics_max_timestep_s);
    motor_cmd = uav.ardupilot.pwm_to_motor_radps(current_frame.motor_pwm_us, params, cfg);
    [state_next, plant_diag] = uav.sim.plant_step_struct(state, motor_cmd, dt_s, params);
    state_next = local_apply_official_ground_clamp(state_next);

    physics_time_s = physics_time_s + dt_s;
    sensors = uav.sensors.sensors_step(state_next, plant_diag, params);
    [sample, ~] = uav.ardupilot.convert_state_to_official_json( ...
        state_next, sensors, physics_time_s, params, cfg, 'SnapshotDiag', plant_diag);
    json_frame = uav.ardupilot.build_official_json_frame(sample, cfg);
    [ctx, ~] = uav.ardupilot.json_lockstep_write_frame(ctx, json_frame);

    state = state_next;
    step_index = step_index + 1;

    [ctx, next_frame] = local_wait_for_valid_frame(ctx, 2.0, cfg);
    if ~next_frame.valid
        break;
    end
    current_frame = next_frame;
end

ctx = uav.ardupilot.json_lockstep_close(ctx);
probe_result = local_read_param_probe_result(probe_info.output_json_path_win, probe_info.stdout_path_win, probe_info.stderr_path_win);

param_table = local_make_param_table(param_names, probe_result);
result = struct();
result.executed = true;
result.launch_command = string(launch_command);
result.profile_win_path = string(profile_win_path);
result.wait_elapsed_s = wait_elapsed_s;
result.step_count = step_index;
result.valid_rx_count = double(ctx.valid_rx_count);
result.json_response_tx_count = double(ctx.json_response_tx_count);
result.last_frame_count = double(ctx.last_frame_count);
result.probe_result = probe_result;
result.param_table = param_table;
result.sitl_console_tail = local_read_text_tail(sitl_console_log, 160);

save(mat_path, 'result');
writetable(param_table, csv_path);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));
assignin('base', 'task_25_official_profile_after_boot', result);

fprintf('TASK-25: проверка официального профиля\n');
fprintf('  valid_rx_count                         : %d\n', result.valid_rx_count);
fprintf('  json_response_tx_count                : %d\n', result.json_response_tx_count);
fprintf('  last_frame_count                      : %d\n', result.last_frame_count);
fprintf('  heartbeat_received                    : %s\n', local_bool_text(probe_result.heartbeat_received));
fprintf('  failure_reason                        : %s\n', char(local_empty_as_none(probe_result.failure_reason)));

function [probe_info, cleanup_list] = local_prepare_param_probe(cfg, param_names)
output_json_path_win = [tempname, '_task25_param_probe.json'];
stdout_path_win = [tempname, '_task25_param_probe_stdout.log'];
stderr_path_win = [tempname, '_task25_param_probe_stderr.log'];
python_path_win = [tempname, '_task25_param_probe.py'];
launcher_path_win = [tempname, '_task25_param_probe.ps1'];

uav.ardupilot.write_utf8_text_file( ...
    python_path_win, ...
    local_make_param_probe_python_text(cfg, param_names, output_json_path_win));

launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$p = Start-Process -FilePath 'python' -ArgumentList @(" + ...
    local_ps_quote(python_path_win) + ...
    ") -WindowStyle Hidden -RedirectStandardOutput " + local_ps_quote(stdout_path_win) + ...
    " -RedirectStandardError " + local_ps_quote(stderr_path_win) + " -PassThru";
launcher_lines(end + 1, 1) = "Write-Output ('PY_PARAM_PROBE_PID={0}' -f $p.Id)";
uav.ardupilot.write_utf8_text_file(launcher_path_win, strjoin(launcher_lines, newline) + newline);

probe_info = struct();
probe_info.launch_command = sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', launcher_path_win);
probe_info.output_json_path_win = string(output_json_path_win);
probe_info.stdout_path_win = string(stdout_path_win);
probe_info.stderr_path_win = string(stderr_path_win);

cleanup_list = {python_path_win, output_json_path_win, stdout_path_win, stderr_path_win, launcher_path_win};
end

function text_value = local_make_param_probe_python_text(cfg, param_names, output_json_path_win)
param_names = string(param_names(:));
candidate_list = [ ...
    "udpin:0.0.0.0:" + string(cfg.mavlink_udp_port); ...
    "udpin:0.0.0.0:" + string(cfg.mavlink_monitor_udp_port)];

lines = strings(0, 1);
lines(end + 1, 1) = "from pymavlink import mavutil";
lines(end + 1, 1) = "import json";
lines(end + 1, 1) = "import math";
lines(end + 1, 1) = "import time";
lines(end + 1, 1) = "from pathlib import Path";
lines(end + 1, 1) = "OUTPUT_PATH = Path(" + local_py(output_json_path_win) + ")";
lines(end + 1, 1) = "CANDIDATES = [";
for idx = 1:numel(candidate_list)
    lines(end + 1, 1) = "    " + local_py(candidate_list(idx)) + ",";
end
lines(end + 1, 1) = "]";
lines(end + 1, 1) = "PARAM_NAMES = [";
for idx = 1:numel(param_names)
    lines(end + 1, 1) = "    " + local_py(param_names(idx)) + ",";
end
lines(end + 1, 1) = "]";
lines(end + 1, 1) = "result = {";
lines(end + 1, 1) = "    'heartbeat_received': False,";
lines(end + 1, 1) = "    'failure_reason': '',";
lines(end + 1, 1) = "    'param_values': {},";
lines(end + 1, 1) = "    'param_names': [],";
lines(end + 1, 1) = "    'connection_string': '',";
lines(end + 1, 1) = "    'connection_attempts': []";
lines(end + 1, 1) = "}";
lines(end + 1, 1) = "master = None";
lines(end + 1, 1) = "try:";
lines(end + 1, 1) = "    for candidate in CANDIDATES:";
lines(end + 1, 1) = "        attempt = {'connection_string': str(candidate), 'heartbeat_received': False, 'error': ''}";
lines(end + 1, 1) = "        try:";
lines(end + 1, 1) = "            master = mavutil.mavlink_connection(candidate, source_system=245, source_component=190, autoreconnect=False)";
lines(end + 1, 1) = "            hb = master.wait_heartbeat(timeout=30)";
lines(end + 1, 1) = "            if hb is None:";
lines(end + 1, 1) = "                raise RuntimeError('Не получен HEARTBEAT по каналу ' + str(candidate))";
lines(end + 1, 1) = "            result['heartbeat_received'] = True";
lines(end + 1, 1) = "            result['connection_string'] = str(candidate)";
lines(end + 1, 1) = "            attempt['heartbeat_received'] = True";
lines(end + 1, 1) = "            result['connection_attempts'].append(attempt)";
lines(end + 1, 1) = "            break";
lines(end + 1, 1) = "        except Exception as exc:";
lines(end + 1, 1) = "            attempt['error'] = str(exc)";
lines(end + 1, 1) = "            result['connection_attempts'].append(attempt)";
lines(end + 1, 1) = "            master = None";
lines(end + 1, 1) = "    if master is None or not result['heartbeat_received']:";
lines(end + 1, 1) = "        raise RuntimeError(result['connection_attempts'][-1]['error'] if result['connection_attempts'] else 'Не удалось открыть MAVLink-подключение.')";
lines(end + 1, 1) = "    for name in PARAM_NAMES:";
lines(end + 1, 1) = "        try:";
lines(end + 1, 1) = "            master.param_fetch_one(name)";
lines(end + 1, 1) = "        except Exception:";
lines(end + 1, 1) = "            pass";
lines(end + 1, 1) = "    deadline = time.time() + 25.0";
lines(end + 1, 1) = "    while time.time() < deadline and len(result['param_values']) < len(PARAM_NAMES):";
lines(end + 1, 1) = "        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)";
lines(end + 1, 1) = "        if msg is None:";
lines(end + 1, 1) = "            continue";
lines(end + 1, 1) = "        name = str(getattr(msg, 'param_id', '')).rstrip('\\x00')";
lines(end + 1, 1) = "        if not name:";
lines(end + 1, 1) = "            continue";
lines(end + 1, 1) = "        value = float(getattr(msg, 'param_value', float('nan')))";
lines(end + 1, 1) = "        result['param_values'][name] = value if math.isfinite(value) else None";
lines(end + 1, 1) = "    result['param_names'] = sorted(result['param_values'].keys())";
lines(end + 1, 1) = "except Exception as exc:";
lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";
text_value = strjoin(lines, newline) + newline;
end

function probe_result = local_read_param_probe_result(output_json_path_win, stdout_path_win, stderr_path_win)
deadline = tic;
while toc(deadline) < 60.0
    if isfile(char(output_json_path_win))
        break;
    end
    pause(0.2);
end

probe_result = struct();
probe_result.heartbeat_received = false;
probe_result.failure_reason = "Не сформирован JSON-результат проверки профиля.";
probe_result.param_values = struct();
probe_result.param_names = strings(0, 1);
probe_result.connection_string = "";
probe_result.connection_attempts = strings(0, 1);

if ~isfile(char(output_json_path_win))
    if isfile(char(stderr_path_win))
        probe_result.failure_reason = string(local_tail_excerpt(fileread(char(stderr_path_win))));
    elseif isfile(char(stdout_path_win))
        probe_result.failure_reason = string(local_tail_excerpt(fileread(char(stdout_path_win))));
    end
    return;
end

decoded = jsondecode(fileread(char(output_json_path_win)));
probe_result.heartbeat_received = isfield(decoded, 'heartbeat_received') && logical(decoded.heartbeat_received);
probe_result.failure_reason = string(local_get_field(decoded, 'failure_reason', ""));
probe_result.param_values = local_get_field(decoded, 'param_values', struct());
probe_result.param_names = string(fieldnames(probe_result.param_values));
probe_result.connection_string = string(local_get_field(decoded, 'connection_string', ""));
if isfield(decoded, 'connection_attempts') && ~isempty(decoded.connection_attempts)
    attempts = strings(numel(decoded.connection_attempts), 1);
    for idx = 1:numel(decoded.connection_attempts)
        item = decoded.connection_attempts(idx);
        attempts(idx) = string(item.connection_string) + " -> HEARTBEAT=" + ...
            local_bool_text(local_get_field(item, 'heartbeat_received', false)) + ...
            ", error=" + local_empty_as_none(local_get_field(item, 'error', ""));
    end
    probe_result.connection_attempts = attempts;
end
end

function launch_command = local_launch_sitl(cfg, profile_win_path, sitl_console_log_win)
profile_wsl_path = uav.ardupilot.windows_to_wsl_path(profile_win_path);
sitl_log_wsl = uav.ardupilot.windows_to_wsl_path(sitl_console_log_win);
wsl_pid_path = '/home/oaleg/task25_verify_profile_arducopter.pid';
wsl_exit_path = '/home/oaleg/task25_verify_profile_arducopter.exit';

cleanup_command = sprintf('rm -f %s %s %s >/dev/null 2>&1 || true', ...
    wsl_pid_path, wsl_exit_path, sitl_log_wsl);
system(sprintf('wsl -d %s -- bash -lc "%s"', ...
    char(cfg.wsl_distro_name), cleanup_command));

defaults_argument = "../Tools/autotest/default_params/copter.parm," + string(profile_wsl_path);
supervisor_script = sprintf([ ...
    'set -euo pipefail\n' ...
    'rm -f %s %s %s\n' ...
    'cd %s/ArduCopter\n' ...
    '(\n' ...
    '  ../build/sitl/bin/arducopter \\\n' ...
    '    --model JSON:%s \\\n' ...
    '    --speedup 1 \\\n' ...
    '    --slave 0 \\\n' ...
    '    --serial0=udpclient:%s:%d \\\n' ...
    '    --serial1=udpclient:%s:%d \\\n' ...
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
    sitl_log_wsl, ...
    char(cfg.ardupilot_root), ...
    char(cfg.udp_remote_ip), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_udp_port), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_monitor_udp_port), ...
    char(defaults_argument), ...
    char(cfg.udp_remote_ip), ...
    sitl_log_wsl, ...
    wsl_pid_path, ...
    wsl_exit_path);

tmp_ps1 = fullfile(tempdir, 'task25_verify_profile_launch.ps1');
ps_lines = strings(0, 1);
ps_lines(end + 1, 1) = "$wslCommand = @'";
ps_lines = [ps_lines; splitlines(string(supervisor_script))];
ps_lines(end + 1, 1) = "'@";
ps_lines(end + 1, 1) = "$p = Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d','" + string(cfg.wsl_distro_name) + "','--','bash','-lc',$wslCommand) -WindowStyle Hidden -PassThru";
ps_lines(end + 1, 1) = "Write-Output ('WSL_WRAPPER_PID={0}' -f $p.Id)";
uav.ardupilot.write_utf8_text_file(tmp_ps1, strjoin(ps_lines, newline) + newline);

launch_command = "powershell -NoProfile -ExecutionPolicy Bypass -File " + local_windows_quote(tmp_ps1);
system(char(launch_command));
end

function [ctx, frame, elapsed_s] = local_wait_for_valid_frame(ctx, timeout_s, cfg)
frame = struct('valid', false);
wait_tic = tic;
while toc(wait_tic) < timeout_s
    [ctx, frame] = uav.ardupilot.json_lockstep_read_frame(ctx);
    if isfield(frame, 'valid') && frame.valid
        elapsed_s = toc(wait_tic);
        return;
    end
    pause(double(cfg.udp_receive_pause_s));
end
elapsed_s = toc(wait_tic);
end

function state = local_apply_official_ground_clamp(state)
if double(state.p_ned_m(3)) >= 0.0
    state.p_ned_m(3) = 0.0;
    state.v_b_mps = [0.0; 0.0; 0.0];
    state.w_b_radps = [0.0; 0.0; 0.0];
end
end

function host_ip = local_resolve_windows_host_ip(distro_name, fallback_ip)
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

function table_value = local_make_param_table(param_names, probe_result)
param_names = string(param_names(:));
present = false(numel(param_names), 1);
values = nan(numel(param_names), 1);

for idx = 1:numel(param_names)
    name = char(param_names(idx));
    if isfield(probe_result.param_values, name)
        values(idx) = double(probe_result.param_values.(name));
        present(idx) = true;
    end
end

table_value = table(param_names, present, values, ...
    'VariableNames', {'param_name', 'present', 'value'});
end

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-25: проверка официального профиля";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "launch_command: " + string(result.launch_command);
lines(end + 1, 1) = "profile_win_path: " + string(result.profile_win_path);
lines(end + 1, 1) = "valid_rx_count: " + string(result.valid_rx_count);
lines(end + 1, 1) = "json_response_tx_count: " + string(result.json_response_tx_count);
lines(end + 1, 1) = "last_frame_count: " + string(result.last_frame_count);
lines(end + 1, 1) = "connection_string: " + local_empty_as_none(result.probe_result.connection_string);
lines(end + 1, 1) = "heartbeat_received: " + local_bool_text(result.probe_result.heartbeat_received);
if strlength(result.probe_result.failure_reason) > 0
    lines(end + 1, 1) = "failure_reason: " + string(result.probe_result.failure_reason);
end
if ~isempty(result.probe_result.connection_attempts)
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "connection_attempts:";
    lines = [lines; "  " + string(result.probe_result.connection_attempts(:))]; %#ok<AGROW>
end
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Примененные параметры:";
for idx = 1:height(result.param_table)
    if result.param_table.present(idx)
        value_text = sprintf('%.6f', result.param_table.value(idx));
    else
        value_text = "не найден";
    end
    lines(end + 1, 1) = "  " + result.param_table.param_name(idx) + " = " + value_text;
end
text_value = strjoin(lines, newline) + newline;
end

function tail_text = local_read_text_tail(file_path, line_count)
tail_text = "";
if ~isfile(file_path)
    return;
end
file_text = string(fileread(file_path));
lines = splitlines(file_text);
if isempty(lines)
    return;
end
start_index = max(numel(lines) - round(line_count) + 1, 1);
tail_text = strjoin(lines(start_index:end), newline);
end

function text_value = local_tail_excerpt(file_text)
lines = splitlines(string(file_text));
if isempty(lines)
    text_value = "";
    return;
end
start_index = max(numel(lines) - 10 + 1, 1);
text_value = strjoin(lines(start_index:end), " | ");
end

function value = local_get_field(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name)
    value = data.(field_name);
else
    value = default_value;
end
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    file_path = char(string(file_list{idx}));
    if isfile(file_path)
        delete(file_path);
    end
end
end

function quoted = local_windows_quote(text_value)
quoted = '"' + replace(string(text_value), '"', '""') + '"';
end

function literal = local_ps_quote(text_value)
escaped = replace(string(text_value), "'", "''");
literal = "'" + escaped + "'";
end

function literal = local_py(text_value)
escaped = replace(string(text_value), "\", "\\");
escaped = replace(escaped, "'", "\\'");
literal = "'" + escaped + "'";
end

function text_value = local_empty_as_none(value)
value = string(value);
if strlength(strtrim(value)) == 0
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
