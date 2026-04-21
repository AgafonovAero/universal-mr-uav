%% RUN_ARDUPILOT_JSON_OFFICIAL_ARM_TAKEOFF_ATTEMPT
% Выполнить попытку взведения и набора тяги в официальном lockstep-режиме
% MATLAB backend для ArduPilot JSON SITL.
%
% Назначение:
%   Запускает ArduPilot JSON SITL с профилем TASK-25, поднимает
%   официальный контур обмена "пакет -> шаг -> JSON-ответ", затем через
%   отдельный Windows-side pymavlink helper пытается:
%   1. дождаться HEARTBEAT;
%   2. перевести аппарат в GUIDED;
%   3. выполнить arm;
%   4. при успешном arm выдать takeoff 1 м.
%
% Входы:
%   none
%
% Выходы:
%   task_25_official_arm_takeoff_attempt - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   углы - радианы;
%   частоты вращения - рад/с;
%   высота - метры.
%
% Допущения:
%   Используется официальный JSON backend ArduPilot на UDP-порту 9002.

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

log_path = fullfile(logs_dir, 'task_25_official_arm_takeoff_attempt.txt');
csv_path = fullfile(reports_dir, 'task_25_official_arm_takeoff_attempt.csv');
mat_path = fullfile(reports_dir, 'task_25_official_arm_takeoff_attempt.mat');
sitl_console_log = fullfile(logs_dir, 'task_25_official_arm_takeoff_attempt_arducopter_console.txt');

cfg = uav.ardupilot.default_json_config();
cfg.udp_timeout_s = 1.0;
cfg.physics_max_timestep_s = 1.0 / 50.0;
cfg.json_send_quaternion = false;
cfg.udp_remote_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
cfg.mavlink_udp_ip = cfg.udp_remote_ip;
cfg.mavlink_monitor_udp_port = 14552;

profile_win_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task25_official_matlab_json.parm');
uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port);
cleanup_sitl = onCleanup(@() uav.ardupilot.stop_existing_sitl( ...
    cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port)); %#ok<NASGU>

ctx = uav.ardupilot.json_lockstep_open(cfg);
cleanup_udp = onCleanup(@() uav.ardupilot.json_lockstep_close(ctx)); %#ok<NASGU>

launch_command = local_launch_sitl(cfg, profile_win_path, sitl_console_log);

seq_cfg = struct();
seq_cfg.runtime = "windows";
seq_cfg.connection_candidates = [ ...
    "udpin:0.0.0.0:" + string(cfg.mavlink_udp_port); ...
    "udpin:0.0.0.0:" + string(cfg.mavlink_monitor_udp_port)];
seq_cfg.mode_name = "GUIDED";
seq_cfg.wait_before_command_s = 20.0;
seq_cfg.heartbeat_timeout_s = 25.0;
seq_cfg.arm_timeout_s = 20.0;
seq_cfg.monitor_duration_s = 15.0;
seq_cfg.takeoff_alt_m = 1.0;
seq_cfg.sample_period_s = 0.5;
command_info = uav.ardupilot.make_pymavlink_sequence_command(seq_cfg);
[launcher_info, cleanup_list] = local_prepare_sequence_launcher(command_info);
cleanup_helper = onCleanup(@() local_cleanup_temp(cleanup_list)); %#ok<NASGU>
system(launcher_info.launch_command); %#ok<ASGLU>

[ctx, first_frame, wait_elapsed_s] = local_wait_for_valid_frame(ctx, 15.0, cfg);

result = struct();
result.executed = false;
result.launch_command = string(launch_command);
result.profile_win_path = string(profile_win_path);
result.wait_elapsed_s = wait_elapsed_s;
result.arm = false;
result.guided = false;
result.takeoff_accepted = false;
result.command_ack_arm = nan;
result.command_ack_takeoff = nan;
result.failure_reason = "";
result.max_altitude_m = 0.0;
result.height_changed = false;
result.max_total_thrust_to_weight = 0.0;

if ~first_frame.valid
    result.failure_reason = "Первый валидный пакет SITL не получен.";
    result.sitl_console_tail = local_read_text_tail(sitl_console_log, 120);
    save(mat_path, 'result');
    writetable(table(string(result.failure_reason), 'VariableNames', {'failure_reason'}), csv_path);
    uav.ardupilot.write_utf8_text_file(log_path, ...
        "TASK-25: попытка arm/takeoff в официальном режиме" + newline + ...
        "Причина: " + string(result.failure_reason) + newline);
    error('uav:task25:officialarm:NoFirstFrame', ...
        'Первый валидный пакет SITL не получен.');
end

params = uav.sim.make_deterministic_demo_params();
state = uav.core.state_unpack(params.demo.initial_state_plant);
physics_time_s = 0.0;
duration_s = 40.0;
wall_deadline_s = 180.0;
current_frame = first_frame;
step_index = 0;
max_steps = 25000;

time_s = nan(max_steps, 1);
frame_rate_hz = nan(max_steps, 1);
frame_count = nan(max_steps, 1);
pwm_matrix = nan(max_steps, 4);
motor_cmd_radps = nan(max_steps, 4);
altitude_m = nan(max_steps, 1);
vertical_speed_up_mps = nan(max_steps, 1);
roll_rad = nan(max_steps, 1);
pitch_rad = nan(max_steps, 1);
yaw_rad = nan(max_steps, 1);
total_thrust_N = nan(max_steps, 1);
total_thrust_to_weight = nan(max_steps, 1);
valid_rx_count = nan(max_steps, 1);
json_response_tx_count = nan(max_steps, 1);
duplicate_frame_count = nan(max_steps, 1);
missed_frame_count = nan(max_steps, 1);

weight_N = double(params.mass_kg) * double(params.gravity_mps2);
wall_tic = tic;
while physics_time_s < duration_s && toc(wall_tic) < wall_deadline_s
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

    step_index = step_index + 1;
    state = state_next;

    euler_rpy = local_quat_to_euler_rpy(state.q_nb);
    c_nb = uav.core.quat_to_dcm(double(state.q_nb(:)));
    v_ned_mps = c_nb * double(state.v_b_mps(:));
    [thrust_vec_N, ~] = uav.vmg.rotor_simple( ...
        double(motor_cmd(:)), ...
        params.rotor.kT_N_per_radps2, ...
        params.rotor.kQ_Nm_per_radps2);

    time_s(step_index) = physics_time_s;
    frame_rate_hz(step_index) = double(current_frame.frame_rate_hz);
    frame_count(step_index) = double(current_frame.frame_count);
    pwm_matrix(step_index, :) = double(current_frame.motor_pwm_us(:)).';
    motor_cmd_radps(step_index, :) = double(motor_cmd(:)).';
    altitude_m(step_index) = -double(state.p_ned_m(3));
    vertical_speed_up_mps(step_index) = -double(v_ned_mps(3));
    roll_rad(step_index) = euler_rpy(1);
    pitch_rad(step_index) = euler_rpy(2);
    yaw_rad(step_index) = euler_rpy(3);
    total_thrust_N(step_index) = sum(thrust_vec_N);
    total_thrust_to_weight(step_index) = total_thrust_N(step_index) / weight_N;
    valid_rx_count(step_index) = double(ctx.valid_rx_count);
    json_response_tx_count(step_index) = double(ctx.json_response_tx_count);
    duplicate_frame_count(step_index) = double(ctx.duplicate_frame_count);
    missed_frame_count(step_index) = double(ctx.missed_frame_count);

    [ctx, next_frame] = local_wait_for_valid_frame(ctx, 2.0, cfg);
    if ~next_frame.valid
        break;
    end
    current_frame = next_frame;
end

ctx = uav.ardupilot.json_lockstep_close(ctx);
seq_result = local_read_sequence_result( ...
    command_info.output_json_path_win, ...
    launcher_info.helper_stdout_path_win, ...
    launcher_info.helper_stderr_path_win);

time_s = time_s(1:step_index);
frame_rate_hz = frame_rate_hz(1:step_index);
frame_count = frame_count(1:step_index);
pwm_matrix = pwm_matrix(1:step_index, :);
motor_cmd_radps = motor_cmd_radps(1:step_index, :);
altitude_m = altitude_m(1:step_index);
vertical_speed_up_mps = vertical_speed_up_mps(1:step_index);
roll_rad = roll_rad(1:step_index);
pitch_rad = pitch_rad(1:step_index);
yaw_rad = yaw_rad(1:step_index);
total_thrust_N = total_thrust_N(1:step_index);
total_thrust_to_weight = total_thrust_to_weight(1:step_index);
valid_rx_count = valid_rx_count(1:step_index);
json_response_tx_count = json_response_tx_count(1:step_index);
duplicate_frame_count = duplicate_frame_count(1:step_index);
missed_frame_count = missed_frame_count(1:step_index);

history_table = table( ...
    time_s, frame_rate_hz, frame_count, ...
    pwm_matrix(:, 1), pwm_matrix(:, 2), pwm_matrix(:, 3), pwm_matrix(:, 4), ...
    motor_cmd_radps(:, 1), motor_cmd_radps(:, 2), motor_cmd_radps(:, 3), motor_cmd_radps(:, 4), ...
    altitude_m, vertical_speed_up_mps, roll_rad, pitch_rad, yaw_rad, ...
    total_thrust_N, total_thrust_to_weight, ...
    valid_rx_count, json_response_tx_count, duplicate_frame_count, missed_frame_count, ...
    'VariableNames', { ...
        'time_s', 'frame_rate_hz', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_1_radps', 'motor_2_radps', 'motor_3_radps', 'motor_4_radps', ...
        'altitude_m', 'vertical_speed_up_mps', 'roll_rad', 'pitch_rad', 'yaw_rad', ...
        'total_thrust_N', 'total_thrust_to_weight', ...
        'valid_rx_count', 'json_response_tx_count', 'duplicate_frame_count', 'missed_frame_count'});

result.executed = true;
result.valid_rx_count = double(ctx.valid_rx_count);
result.json_response_tx_count = double(ctx.json_response_tx_count);
result.missed_frame_count = double(ctx.missed_frame_count);
result.duplicate_frame_count = double(ctx.duplicate_frame_count);
result.last_frame_count = double(ctx.last_frame_count);
result.arm = logical(local_get_field(seq_result, 'arm_succeeded', false));
result.takeoff_accepted = isequal(local_get_field(seq_result, 'takeoff_ack_result', nan), 0);
result.command_ack_arm = double(local_get_field(seq_result, 'arm_ack_result', nan));
result.command_ack_takeoff = double(local_get_field(seq_result, 'takeoff_ack_result', nan));
result.failure_reason = string(local_pick_failure_reason(seq_result));
result.motor_pwm_min_us = min(pwm_matrix, [], 'all', 'omitnan');
result.motor_pwm_max_us = max(pwm_matrix, [], 'all', 'omitnan');
result.motor_cmd_min_radps = min(motor_cmd_radps, [], 'all', 'omitnan');
result.motor_cmd_max_radps = max(motor_cmd_radps, [], 'all', 'omitnan');
result.max_total_thrust_to_weight = max(total_thrust_to_weight, [], 'omitnan');
result.max_altitude_m = max(altitude_m, [], 'omitnan');
result.height_changed = (result.max_altitude_m - min(altitude_m, [], 'omitnan')) > 0.05;
result.sequence_result = seq_result;
result.history_table = history_table;
result.sitl_console_tail = local_read_text_tail(sitl_console_log, 160);
result.guided = local_sequence_guided(seq_result);

save(mat_path, 'result');
writetable(history_table, csv_path);
uav.ardupilot.write_utf8_text_file(log_path, local_safe_log_text(result));
assignin('base', 'task_25_official_arm_takeoff_attempt', result);

fprintf('TASK-25: official arm/takeoff attempt\n');
fprintf('  arm                                  : %s\n', local_bool_text(result.arm));
fprintf('  arm ACK                              : %.0f\n', result.command_ack_arm);
fprintf('  takeoff ACK                          : %.0f\n', result.command_ack_takeoff);
fprintf('  valid_rx_count                       : %d\n', result.valid_rx_count);
fprintf('  json_response_tx_count               : %d\n', result.json_response_tx_count);
fprintf('  motor pwm range [us]                 : %.0f .. %.0f\n', result.motor_pwm_min_us, result.motor_pwm_max_us);
fprintf('  motor cmd range [rad/s]              : %.3f .. %.3f\n', result.motor_cmd_min_radps, result.motor_cmd_max_radps);
fprintf('  max thrust / weight                  : %.4f\n', result.max_total_thrust_to_weight);
fprintf('  max altitude [m]                     : %.4f\n', result.max_altitude_m);

function [launcher_info, cleanup_list] = local_prepare_sequence_launcher(command_info)
helper_stdout_path_win = [tempname, '_task25_official_helper_stdout.log'];
helper_stderr_path_win = [tempname, '_task25_official_helper_stderr.log'];
launcher_path_win = [tempname, '_task25_official_helper.ps1'];

launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$p = Start-Process -FilePath 'python' -ArgumentList @(" + ...
    local_ps_quote(command_info.python_script_path_win) + ...
    ") -WindowStyle Hidden -RedirectStandardOutput " + local_ps_quote(helper_stdout_path_win) + ...
    " -RedirectStandardError " + local_ps_quote(helper_stderr_path_win) + " -PassThru";
launcher_lines(end + 1, 1) = "Write-Output ('PY_HELPER_PID={0}' -f $p.Id)";
uav.ardupilot.write_utf8_text_file(launcher_path_win, strjoin(launcher_lines, newline) + newline);

launcher_info = struct();
launcher_info.launch_command = sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', launcher_path_win);
launcher_info.helper_stdout_path_win = string(helper_stdout_path_win);
launcher_info.helper_stderr_path_win = string(helper_stderr_path_win);
cleanup_list = {char(command_info.python_script_path_win), char(command_info.output_json_path_win), helper_stdout_path_win, helper_stderr_path_win, launcher_path_win};
end

function seq_result = local_read_sequence_result(json_path_win, helper_stdout_path_win, helper_stderr_path_win)
deadline = tic;
while toc(deadline) < 90.0
    if isfile(char(json_path_win))
        break;
    end
    pause(0.2);
end

if isfile(char(json_path_win))
    seq_result = uav.ardupilot.read_pymavlink_sequence_result(json_path_win);
else
    seq_result = struct();
    seq_result.telemetry_table = table();
    seq_result.arm_succeeded = false;
    seq_result.arm_ack_result = nan;
    seq_result.takeoff_ack_result = nan;
    seq_result.status_texts = strings(0, 1);
    if isfile(char(helper_stderr_path_win))
        seq_result.failure_reason = string(local_tail_excerpt(fileread(char(helper_stderr_path_win))));
    elseif isfile(char(helper_stdout_path_win))
        seq_result.failure_reason = string(local_tail_excerpt(fileread(char(helper_stdout_path_win))));
    else
        seq_result.failure_reason = "Не сформирован JSON-результат pymavlink helper.";
    end
end
end

function launch_command = local_launch_sitl(cfg, profile_win_path, sitl_console_log_win)
profile_wsl_path = uav.ardupilot.windows_to_wsl_path(profile_win_path);
sitl_log_wsl = uav.ardupilot.windows_to_wsl_path(sitl_console_log_win);
wsl_pid_path = '/home/oaleg/task25_official_arm_arducopter.pid';
wsl_exit_path = '/home/oaleg/task25_official_arm_arducopter.exit';

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

tmp_ps1 = fullfile(tempdir, 'task25_launch_official_arm.ps1');
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

function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
q_nb = q_nb(:) ./ norm(q_nb);
q0 = q_nb(1);
q1 = q_nb(2);
q2 = q_nb(3);
q3 = q_nb(4);
roll_rad = atan2(2.0 .* (q0 .* q1 + q2 .* q3), ...
    1.0 - 2.0 .* (q1.^2 + q2.^2));
pitch_arg = 2.0 .* (q0 .* q2 - q3 .* q1);
pitch_arg = min(max(pitch_arg, -1.0), 1.0);
pitch_rad = asin(pitch_arg);
yaw_rad = atan2(2.0 .* (q0 .* q3 + q1 .* q2), ...
    1.0 - 2.0 .* (q2.^2 + q3.^2));
euler_rpy_rad = [roll_rad; pitch_rad; yaw_rad];
end

function value = local_get_field(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name)
    value = data.(field_name);
else
    value = default_value;
end
end

function flag_value = local_sequence_guided(seq_result)
flag_value = false;
if ~isstruct(seq_result) || ~isfield(seq_result, 'telemetry_table')
    return;
end
telem = seq_result.telemetry_table;
if ~istable(telem) || isempty(telem) || ~ismember('mode', telem.Properties.VariableNames)
    return;
end
flag_value = any(strcmpi(string(telem.mode), 'GUIDED'));
end

function reason = local_pick_failure_reason(seq_result)
reason = string(local_get_field(seq_result, 'failure_reason', ""));
status_texts = string(local_get_field(seq_result, 'status_texts', strings(0, 1)));
arm_idx = find(contains(status_texts, "Arm:"), 1, 'first');
if ~isempty(arm_idx)
    reason = status_texts(arm_idx);
    return;
end

if strlength(strtrim(reason)) > 0 && ~contains(reason, "Field Elevation Set")
    return;
end

status_texts = status_texts(strlength(strtrim(status_texts)) > 0);
if ~isempty(status_texts)
    reason = status_texts(end);
else
    reason = "Причина отказа не зафиксирована.";
end
end

function text_value = local_safe_log_text(result)
text_value = local_make_log_text(result);
if ismissing(text_value) || strlength(strtrim(text_value)) == 0
    lines = strings(0, 1);
    lines(end + 1, 1) = "TASK-25: попытка arm/takeoff в официальном MATLAB backend";
    lines(end + 1, 1) = "arm: " + local_bool_text(result.arm);
    lines(end + 1, 1) = "command_ack_arm: " + string(result.command_ack_arm);
    lines(end + 1, 1) = "failure_reason: " + local_empty_as_none(result.failure_reason);
    lines(end + 1, 1) = "valid_rx_count: " + string(result.valid_rx_count);
    lines(end + 1, 1) = "json_response_tx_count: " + string(result.json_response_tx_count);
    text_value = strjoin(lines, newline) + newline;
end
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

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-25: попытка arm/takeoff в официальном MATLAB backend";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "launch_command: " + string(result.launch_command);
lines(end + 1, 1) = "profile_win_path: " + string(result.profile_win_path);
lines(end + 1, 1) = "valid_rx_count: " + string(result.valid_rx_count);
lines(end + 1, 1) = "json_response_tx_count: " + string(result.json_response_tx_count);
lines(end + 1, 1) = "missed_frame_count: " + string(result.missed_frame_count);
lines(end + 1, 1) = "duplicate_frame_count: " + string(result.duplicate_frame_count);
lines(end + 1, 1) = "last_frame_count: " + string(result.last_frame_count);
lines(end + 1, 1) = "arm: " + local_bool_text(result.arm);
lines(end + 1, 1) = "command_ack_arm: " + string(result.command_ack_arm);
lines(end + 1, 1) = "command_ack_takeoff: " + string(result.command_ack_takeoff);
lines(end + 1, 1) = "guided: " + local_bool_text(result.guided);
lines(end + 1, 1) = "takeoff_accepted: " + local_bool_text(result.takeoff_accepted);
lines(end + 1, 1) = "failure_reason: " + local_empty_as_none(result.failure_reason);
lines(end + 1, 1) = sprintf('motor_pwm_us: %.0f .. %.0f', result.motor_pwm_min_us, result.motor_pwm_max_us);
lines(end + 1, 1) = sprintf('motor_cmd_radps: %.6f .. %.6f', result.motor_cmd_min_radps, result.motor_cmd_max_radps);
lines(end + 1, 1) = sprintf('max_total_thrust_to_weight: %.6f', result.max_total_thrust_to_weight);
lines(end + 1, 1) = sprintf('max_altitude_m: %.6f', result.max_altitude_m);
lines(end + 1, 1) = "height_changed: " + local_bool_text(result.height_changed);
text_value = strjoin(lines, newline) + newline;
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
