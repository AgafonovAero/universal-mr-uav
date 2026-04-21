%% RUN_ARDUPILOT_JSON_OFFICIAL_LOCKSTEP_20S Выполнить 20-секундный официальный lockstep-прогон.
% Назначение:
%   Запускает ArduPilot JSON SITL с единым профилем TASK-25 и выполняет
%   обмен по официальной схеме MATLAB backend:
%   один валидный бинарный пакет -> один шаг физики -> один JSON-ответ.

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

log_path = fullfile(logs_dir, 'task_25_official_lockstep_20s.txt');
csv_path = fullfile(reports_dir, 'task_25_official_lockstep_20s.csv');
mat_path = fullfile(reports_dir, 'task_25_official_lockstep_20s.mat');
sitl_console_log = fullfile(logs_dir, 'task_25_official_lockstep_20s_arducopter_console.txt');

cfg = uav.ardupilot.default_json_config();
cfg.udp_timeout_s = 1.0;
cfg.physics_max_timestep_s = 1.0 / 50.0;
cfg.json_send_quaternion = false;
cfg.json_diagnostic_ground_clamp = false;
cfg.udp_remote_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
cfg.mavlink_udp_ip = cfg.udp_remote_ip;
cfg.mavlink_monitor_udp_port = 14552;

profile_win_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task25_official_matlab_json.parm');
uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port);
cleanup_sitl = onCleanup(@() uav.ardupilot.stop_existing_sitl( ...
    cfg.wsl_distro_name, cfg.udp_remote_ip, cfg.mavlink_udp_port)); %#ok<NASGU>

ctx = uav.ardupilot.json_lockstep_open(cfg);
cleanup_udp = onCleanup(@() uav.ardupilot.json_lockstep_close(ctx)); %#ok<NASGU>

launch_command = local_launch_sitl( ...
    cfg, ...
    profile_win_path, ...
    sitl_console_log);

[ctx, first_frame, wait_elapsed_s] = local_wait_for_valid_frame(ctx, 15.0, cfg);

result = struct();
result.cfg = cfg;
result.launch_command = string(launch_command);
result.profile_win_path = string(profile_win_path);
result.first_frame_received = false;
result.valid_rx_count = 0;
result.json_response_tx_count = 0;
result.missed_frame_count = 0;
result.duplicate_frame_count = 0;
result.invalid_rx_count = 0;
result.last_frame_count = 0;
result.arducopter_alive = false;
result.max_altitude_m = 0.0;
result.has_nan_inf = false;
result.wait_elapsed_s = wait_elapsed_s;
result.received_datagram_count = double(ctx.received_datagram_count);
result.transport_message = string(ctx.transport.message);
result.last_status = string(ctx.last_status);
result.last_error = string(ctx.last_error);

if ~first_frame.valid
    result.failure_reason = "Первый валидный пакет SITL не получен.";
    result.arducopter_alive = local_is_arducopter_alive(cfg, cfg.udp_remote_ip);
    result.sitl_console_tail = local_read_text_tail(sitl_console_log, 80);
    save(mat_path, 'result');
    writetable(table(string(result.failure_reason), 'VariableNames', {'failure_reason'}), csv_path);
    uav.ardupilot.write_utf8_text_file(log_path, ...
        "Официальный lockstep-прогон TASK-25" + newline + ...
        "Первый валидный пакет SITL не получен." + newline + ...
        "arducopter_alive: " + local_bool_text(result.arducopter_alive) + newline + ...
        "received_datagram_count: " + string(result.received_datagram_count) + newline + ...
        "last_status: " + string(result.last_status) + newline + ...
        "last_error: " + string(result.last_error) + newline + ...
        "transport_message: " + string(result.transport_message) + newline + ...
        "launch_command: " + string(launch_command) + newline);
    assignin('base', 'task_25_official_lockstep_20s', result);
    error('uav:task25:officiallockstep:NoFirstFrame', ...
        'Первый валидный пакет SITL не получен.');
end

params = uav.sim.make_deterministic_demo_params();
state = uav.core.state_unpack(params.demo.initial_state_plant);
physics_time_s = 0.0;
duration_s = 20.0;
wall_deadline_s = 120.0;
current_frame = first_frame;
step_index = 0;
max_steps = 20000;

time_s = nan(max_steps, 1);
frame_rate_hz = nan(max_steps, 1);
frame_count = nan(max_steps, 1);
motor_pwm_us = nan(max_steps, 4);
motor_cmd_radps = nan(max_steps, 4);
altitude_m = nan(max_steps, 1);
roll_rad = nan(max_steps, 1);
pitch_rad = nan(max_steps, 1);
yaw_rad = nan(max_steps, 1);
accel_body_x = nan(max_steps, 1);
accel_body_y = nan(max_steps, 1);
accel_body_z = nan(max_steps, 1);
received_datagram_count = nan(max_steps, 1);
valid_rx_count = nan(max_steps, 1);
json_response_tx_count = nan(max_steps, 1);
duplicate_frame_count = nan(max_steps, 1);
missed_frame_count = nan(max_steps, 1);

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
    [ctx, tx_info] = uav.ardupilot.json_lockstep_write_frame(ctx, json_frame); %#ok<ASGLU>

    step_index = step_index + 1;
    state = state_next;

    euler_rpy = local_quat_to_euler_rpy(state.q_nb);
    time_s(step_index) = physics_time_s;
    frame_rate_hz(step_index) = double(current_frame.frame_rate_hz);
    frame_count(step_index) = double(current_frame.frame_count);
    motor_pwm_us(step_index, :) = double(current_frame.motor_pwm_us(:)).';
    motor_cmd_radps(step_index, :) = double(motor_cmd(:)).';
    altitude_m(step_index) = -double(state.p_ned_m(3));
    roll_rad(step_index) = euler_rpy(1);
    pitch_rad(step_index) = euler_rpy(2);
    yaw_rad(step_index) = euler_rpy(3);
    accel_body_x(step_index) = double(sample.imu.accel_body(1));
    accel_body_y(step_index) = double(sample.imu.accel_body(2));
    accel_body_z(step_index) = double(sample.imu.accel_body(3));
    received_datagram_count(step_index) = double(ctx.received_datagram_count);
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

time_s = time_s(1:step_index);
frame_rate_hz = frame_rate_hz(1:step_index);
frame_count = frame_count(1:step_index);
motor_pwm_us = motor_pwm_us(1:step_index, :);
motor_cmd_radps = motor_cmd_radps(1:step_index, :);
altitude_m = altitude_m(1:step_index);
roll_rad = roll_rad(1:step_index);
pitch_rad = pitch_rad(1:step_index);
yaw_rad = yaw_rad(1:step_index);
accel_body_x = accel_body_x(1:step_index);
accel_body_y = accel_body_y(1:step_index);
accel_body_z = accel_body_z(1:step_index);
received_datagram_count = received_datagram_count(1:step_index);
valid_rx_count = valid_rx_count(1:step_index);
json_response_tx_count = json_response_tx_count(1:step_index);
duplicate_frame_count = duplicate_frame_count(1:step_index);
missed_frame_count = missed_frame_count(1:step_index);

result.first_frame_received = true;
result.failure_reason = "";
result.time_s = time_s;
result.frame_rate_hz = frame_rate_hz;
result.frame_count = frame_count;
result.motor_pwm_us = motor_pwm_us;
result.motor_cmd_radps = motor_cmd_radps;
result.altitude_m = altitude_m;
result.roll_rad = roll_rad;
result.pitch_rad = pitch_rad;
result.yaw_rad = yaw_rad;
result.accel_body_x = accel_body_x;
result.accel_body_y = accel_body_y;
result.accel_body_z = accel_body_z;
result.received_datagram_count_hist = received_datagram_count;
result.valid_rx_count_hist = valid_rx_count;
result.json_response_tx_count_hist = json_response_tx_count;
result.duplicate_frame_count_hist = duplicate_frame_count;
result.missed_frame_count_hist = missed_frame_count;
result.received_datagram_count = double(ctx.received_datagram_count);
result.valid_rx_count = double(ctx.valid_rx_count);
result.json_response_tx_count = double(ctx.json_response_tx_count);
result.missed_frame_count = double(ctx.missed_frame_count);
result.duplicate_frame_count = double(ctx.duplicate_frame_count);
result.invalid_rx_count = double(ctx.invalid_rx_count);
result.last_frame_count = double(ctx.last_frame_count);
result.last_status = string(ctx.last_status);
result.last_error = string(ctx.last_error);
result.last_motor_pwm_us = double(ctx.last_motor_pwm_us(:));
result.last_motor_cmd_radps = double(ctx.last_motor_cmd_radps(:));
result.arducopter_alive = local_is_arducopter_alive(cfg, cfg.udp_remote_ip);
result.max_altitude_m = max([0.0; altitude_m(isfinite(altitude_m))]);
result.has_nan_inf = any(~isfinite([altitude_m; roll_rad; pitch_rad; yaw_rad; accel_body_x; accel_body_y; accel_body_z]));
result.sitl_console_tail = local_read_text_tail(sitl_console_log, 120);

result_table = table( ...
    time_s, frame_rate_hz, frame_count, ...
    motor_pwm_us(:, 1), motor_pwm_us(:, 2), motor_pwm_us(:, 3), motor_pwm_us(:, 4), ...
    motor_cmd_radps(:, 1), motor_cmd_radps(:, 2), motor_cmd_radps(:, 3), motor_cmd_radps(:, 4), ...
    altitude_m, roll_rad, pitch_rad, yaw_rad, ...
    accel_body_x, accel_body_y, accel_body_z, ...
    received_datagram_count, valid_rx_count, json_response_tx_count, ...
    duplicate_frame_count, missed_frame_count, ...
    'VariableNames', { ...
        'time_s', 'frame_rate_hz', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_1_radps', 'motor_2_radps', 'motor_3_radps', 'motor_4_radps', ...
        'altitude_m', 'roll_rad', 'pitch_rad', 'yaw_rad', ...
        'accel_body_x', 'accel_body_y', 'accel_body_z', ...
        'received_datagram_count', 'valid_rx_count', 'json_response_tx_count', ...
        'duplicate_frame_count', 'missed_frame_count'});

save(mat_path, 'result');
writetable(result_table, csv_path);

log_lines = strings(0, 1);
log_lines(end + 1, 1) = "Официальный lockstep-прогон TASK-25";
log_lines(end + 1, 1) = "============================================================";
log_lines(end + 1, 1) = "launch_command: " + string(launch_command);
log_lines(end + 1, 1) = "profile_win_path: " + string(profile_win_path);
log_lines(end + 1, 1) = "valid_rx_count: " + string(result.valid_rx_count);
log_lines(end + 1, 1) = "json_response_tx_count: " + string(result.json_response_tx_count);
log_lines(end + 1, 1) = "missed_frame_count: " + string(result.missed_frame_count);
log_lines(end + 1, 1) = "duplicate_frame_count: " + string(result.duplicate_frame_count);
log_lines(end + 1, 1) = "invalid_rx_count: " + string(result.invalid_rx_count);
log_lines(end + 1, 1) = "last_frame_count: " + string(result.last_frame_count);
log_lines(end + 1, 1) = "arducopter_alive: " + local_bool_text(result.arducopter_alive);
log_lines(end + 1, 1) = "has_nan_inf: " + local_bool_text(result.has_nan_inf);
log_lines(end + 1, 1) = "max_altitude_m: " + sprintf('%.6f', result.max_altitude_m);
log_lines(end + 1, 1) = "last_status: " + string(result.last_status);
if strlength(result.last_error) > 0
    log_lines(end + 1, 1) = "last_error: " + string(result.last_error);
end
uav.ardupilot.write_utf8_text_file(log_path, strjoin(log_lines, newline) + newline);

assignin('base', 'task_25_official_lockstep_20s', result);

fprintf('Официальный lockstep-прогон TASK-25\n');
fprintf('  valid_rx_count                         : %d\n', result.valid_rx_count);
fprintf('  json_response_tx_count                : %d\n', result.json_response_tx_count);
fprintf('  missed_frame_count                    : %d\n', result.missed_frame_count);
fprintf('  duplicate_frame_count                 : %d\n', result.duplicate_frame_count);
fprintf('  last_frame_count                      : %d\n', result.last_frame_count);
fprintf('  arducopter_alive                      : %s\n', local_bool_text(result.arducopter_alive));

function launch_command = local_launch_sitl(cfg, profile_win_path, sitl_console_log_win)
profile_wsl_path = uav.ardupilot.windows_to_wsl_path(profile_win_path);
sitl_log_wsl = uav.ardupilot.windows_to_wsl_path(sitl_console_log_win);
wsl_pid_path = '/home/oaleg/task25_official_lockstep_arducopter.pid';
wsl_exit_path = '/home/oaleg/task25_official_lockstep_arducopter.exit';
wsl_wrapper_path = '/home/oaleg/task25_official_lockstep_arducopter.wrapper';

cleanup_command = sprintf('rm -f %s %s %s %s >/dev/null 2>&1 || true', ...
    wsl_pid_path, wsl_exit_path, sitl_log_wsl, wsl_wrapper_path);
system(sprintf('wsl -d %s -- bash -lc "%s"', ...
    char(cfg.wsl_distro_name), ...
    cleanup_command));

defaults_argument = "../Tools/autotest/default_params/copter.parm," + string(profile_wsl_path);
command_text = sprintf([ ...
    'cd %s/ArduCopter && ' ...
    '../build/sitl/bin/arducopter ' ...
    '--model JSON:%s ' ...
    '--speedup 1 ' ...
    '--slave 0 ' ...
    '--serial0=udpclient:%s:%d ' ...
    '--serial1=udpclient:%s:%d ' ...
    '--defaults %s ' ...
    '--sim-address=%s ' ...
    '-I0'], ...
    char(cfg.ardupilot_root), ...
    char(cfg.udp_remote_ip), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_udp_port), ...
    char(cfg.mavlink_udp_ip), ...
    double(cfg.mavlink_monitor_udp_port), ...
    char(defaults_argument), ...
    char(cfg.udp_remote_ip));

supervisor_script = sprintf([ ...
    'set -euo pipefail\n' ...
    'rm -f %s %s %s %s\n' ...
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
    wsl_wrapper_path, ...
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

tmp_ps1 = fullfile(tempdir, 'task25_launch_official_lockstep.ps1');
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

function is_alive = local_is_arducopter_alive(cfg, host_ip)
if nargin < 2
    host_ip = "";
end

host_filter = "";
if strlength(string(host_ip)) > 0
    host_filter = " | grep " + local_bash_quote("JSON:" + string(host_ip));
end

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "ps -eo pid,args | grep ''[a]rducopter''%s >/dev/null 2>&1 && printf 1 || printf 0"', ...
    char(cfg.wsl_distro_name), ...
    char(host_filter));
[~, output_text] = system(command_text);
is_alive = str2double(strtrim(output_text)) == 1;
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

function quoted = local_windows_quote(text_value)
quoted = '"' + replace(string(text_value), '"', '""') + '"';
end

function quoted = local_bash_quote(text_value)
escaped = strrep(char(string(text_value)), '''', '''"''"''');
quoted = "'" + string(escaped) + "'";
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

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end
