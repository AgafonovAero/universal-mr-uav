%% RUN_ARDUPILOT_JSON_OFFICIAL_ARM_TAKEOFF_ATTEMPT
% Выполнить серию попыток arm в официальном lockstep-режиме MATLAB backend.
%
% Назначение:
%   Поднимает ArduPilot JSON SITL с профилем TASK-25, запускает официальный
%   контур обмена "пакет -> шаг -> JSON-ответ" и выполняет три попытки arm
%   после 20, 60 и 120 секунд непрерывного обмена. При сохранении причины
%   `Arm: System not initialised` автоматически выполняет A/B-сравнение
%   штатного режима и диагностического зануления accel_body на земле.

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
cfg.json_diagnostic_ground_clamp = false;
cfg.udp_remote_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
cfg.mavlink_udp_ip = cfg.udp_remote_ip;
cfg.mavlink_monitor_udp_port = 14552;

profile_win_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task25_official_matlab_json.parm');
attempt_schedule_s = [20.0, 60.0, 120.0];

result = local_run_case( ...
    cfg, ...
    profile_win_path, ...
    sitl_console_log, ...
    attempt_schedule_s, ...
    "official_backend", ...
    false);

if isempty(result.history_table)
    writetable(table(string(result.failure_reason), 'VariableNames', {'failure_reason'}), csv_path);
else
    writetable(result.history_table, csv_path);
end

ground_clamp_ab_result = struct();
if ~result.arm && contains(string(result.failure_reason), "System not initialised", 'IgnoreCase', true)
    ground_clamp_ab_result = local_run_ground_clamp_ab_test( ...
        cfg, ...
        profile_win_path, ...
        logs_dir, ...
        reports_dir);
end

save(mat_path, 'result', 'ground_clamp_ab_result');
uav.ardupilot.write_utf8_text_file(log_path, local_make_result_log_text(result, ground_clamp_ab_result));
assignin('base', 'task_25_official_arm_takeoff_attempt', result);
assignin('base', 'task_25_ground_clamp_ab_test', ground_clamp_ab_result);

fprintf('TASK-25: official arm attempt series\n');
fprintf('  arm after 20 s                       : %s\n', local_bool_text(result.arm_after_20_s));
fprintf('  arm after 60 s                       : %s\n', local_bool_text(result.arm_after_60_s));
fprintf('  arm after 120 s                      : %s\n', local_bool_text(result.arm_after_120_s));
fprintf('  first failure reason                 : %s\n', char(local_empty_as_none(result.failure_reason)));
fprintf('  motor pwm range [us]                 : %.0f .. %.0f\n', result.motor_pwm_min_us, result.motor_pwm_max_us);
fprintf('  motor cmd range [rad/s]              : %.6f .. %.6f\n', result.motor_cmd_min_radps, result.motor_cmd_max_radps);

function result = local_run_case(cfg, profile_win_path, sitl_console_log, attempt_schedule_s, case_name, diagnostic_ground_clamp)
cfg = local_copy_cfg(cfg);
cfg.json_diagnostic_ground_clamp = logical(diagnostic_ground_clamp);

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
seq_cfg.wait_before_command_s = 0.0;
seq_cfg.heartbeat_timeout_s = 25.0;
seq_cfg.arm_timeout_s = 15.0;
seq_cfg.monitor_duration_s = 15.0;
seq_cfg.takeoff_alt_m = nan;
seq_cfg.sample_period_s = 0.5;
seq_cfg.arm_attempt_times_s = double(attempt_schedule_s(:)).';
seq_cfg.python_script_path = string(tempname) + "_" + case_name + "_attempts.py";
seq_cfg.output_json_path = string(tempname) + "_" + case_name + "_attempts.json";

command_info = uav.ardupilot.make_pymavlink_sequence_command(seq_cfg);
[launcher_info, cleanup_list] = local_prepare_sequence_launcher(command_info);
cleanup_helper = onCleanup(@() local_cleanup_temp(cleanup_list)); %#ok<NASGU>
system(launcher_info.launch_command); %#ok<ASGLU>

[ctx, first_frame, wait_elapsed_s] = local_wait_for_valid_frame(ctx, 15.0, cfg);

result = struct();
result.case_name = string(case_name);
result.diagnostic_ground_clamp = logical(diagnostic_ground_clamp);
result.launch_command = string(launch_command);
result.profile_win_path = string(profile_win_path);
result.wait_elapsed_s = wait_elapsed_s;
result.executed = false;
result.failure_reason = "";
result.arm = false;
result.arm_after_20_s = false;
result.arm_after_60_s = false;
result.arm_after_120_s = false;
result.command_ack = nan;
result.command_ack_by_attempt = nan(numel(attempt_schedule_s), 1);
result.valid_rx_count = 0;
result.json_response_tx_count = 0;
result.missed_frame_count = 0;
result.duplicate_frame_count = 0;
result.last_frame_count = 0;
result.motor_pwm_min_us = nan;
result.motor_pwm_max_us = nan;
result.motor_cmd_min_radps = nan;
result.motor_cmd_max_radps = nan;
result.history_table = table();
result.attempt_table = table();
result.sitl_console_tail = "";

if ~first_frame.valid
    result.failure_reason = "Первый валидный пакет SITL не получен.";
    result.sitl_console_tail = local_read_text_tail(sitl_console_log, 120);
    return;
end

params = uav.sim.make_deterministic_demo_params();
state = uav.core.state_unpack(params.demo.initial_state_plant);
physics_time_s = 0.0;
duration_s = max([40.0, attempt_schedule_s + 20.0]);
wall_deadline_s = max(duration_s + 60.0, 220.0);
current_frame = first_frame;
step_index = 0;
max_steps = 50000;

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
valid_rx_hist = nan(max_steps, 1);
json_tx_hist = nan(max_steps, 1);
duplicate_hist = nan(max_steps, 1);
missed_hist = nan(max_steps, 1);

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
    valid_rx_hist(step_index) = double(ctx.valid_rx_count);
    json_tx_hist(step_index) = double(ctx.json_response_tx_count);
    duplicate_hist(step_index) = double(ctx.duplicate_frame_count);
    missed_hist(step_index) = double(ctx.missed_frame_count);

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
valid_rx_hist = valid_rx_hist(1:step_index);
json_tx_hist = json_tx_hist(1:step_index);
duplicate_hist = duplicate_hist(1:step_index);
missed_hist = missed_hist(1:step_index);

result.executed = true;
result.history_table = table( ...
    time_s, frame_rate_hz, frame_count, ...
    pwm_matrix(:, 1), pwm_matrix(:, 2), pwm_matrix(:, 3), pwm_matrix(:, 4), ...
    motor_cmd_radps(:, 1), motor_cmd_radps(:, 2), motor_cmd_radps(:, 3), motor_cmd_radps(:, 4), ...
    altitude_m, vertical_speed_up_mps, roll_rad, pitch_rad, yaw_rad, ...
    total_thrust_N, total_thrust_to_weight, ...
    valid_rx_hist, json_tx_hist, duplicate_hist, missed_hist, ...
    'VariableNames', { ...
        'time_s', 'frame_rate_hz', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_1_radps', 'motor_2_radps', 'motor_3_radps', 'motor_4_radps', ...
        'altitude_m', 'vertical_speed_up_mps', 'roll_rad', 'pitch_rad', 'yaw_rad', ...
        'total_thrust_N', 'total_thrust_to_weight', ...
        'valid_rx_count', 'json_response_tx_count', 'duplicate_frame_count', 'missed_frame_count'});

result.valid_rx_count = double(ctx.valid_rx_count);
result.json_response_tx_count = double(ctx.json_response_tx_count);
result.missed_frame_count = double(ctx.missed_frame_count);
result.duplicate_frame_count = double(ctx.duplicate_frame_count);
result.last_frame_count = double(ctx.last_frame_count);

if isempty(pwm_matrix)
    result.motor_pwm_min_us = nan;
    result.motor_pwm_max_us = nan;
    result.motor_cmd_min_radps = nan;
    result.motor_cmd_max_radps = nan;
else
    result.motor_pwm_min_us = min(pwm_matrix, [], 'all', 'omitnan');
    result.motor_pwm_max_us = max(pwm_matrix, [], 'all', 'omitnan');
    result.motor_cmd_min_radps = min(motor_cmd_radps, [], 'all', 'omitnan');
    result.motor_cmd_max_radps = max(motor_cmd_radps, [], 'all', 'omitnan');
end

result.attempt_table = local_make_attempt_table(seq_result, attempt_schedule_s);
result.arm_after_20_s = local_attempt_success(result.attempt_table, 20.0);
result.arm_after_60_s = local_attempt_success(result.attempt_table, 60.0);
result.arm_after_120_s = local_attempt_success(result.attempt_table, 120.0);
result.arm = any(result.attempt_table.arm);
result.command_ack_by_attempt = result.attempt_table.command_ack_arm;
result.command_ack = local_first_finite(result.command_ack_by_attempt);
result.failure_reason = local_first_attempt_failure(result.attempt_table, seq_result);
result.sitl_console_tail = local_read_text_tail(sitl_console_log, 160);
result.max_total_thrust_to_weight = max(total_thrust_to_weight, [], 'omitnan');
result.max_altitude_m = max(altitude_m, [], 'omitnan');
result.sequence_result = seq_result;
end

function attempt_table = local_make_attempt_table(seq_result, attempt_schedule_s)
attempt_schedule_s = double(attempt_schedule_s(:));
row_count = numel(attempt_schedule_s);
label = strings(row_count, 1);
scheduled_time_s = attempt_schedule_s;
arm = false(row_count, 1);
command_ack_arm = nan(row_count, 1);
heartbeat_received = false(row_count, 1);
guided_seen = false(row_count, 1);
failure_reason = strings(row_count, 1);

attempts = local_get_field(seq_result, 'arm_attempts', struct([]));
for idx = 1:row_count
    label(idx) = "arm_after_" + string(round(attempt_schedule_s(idx))) + "_s";
    failure_reason(idx) = "Результат попытки arm не сформирован.";
    if idx <= numel(attempts)
        attempt_item = attempts(idx);
        arm(idx) = logical(local_get_field(attempt_item, 'arm_succeeded', false));
        command_ack_arm(idx) = double(local_get_field(attempt_item, 'arm_ack_result', nan));
        heartbeat_received(idx) = logical(local_get_field(attempt_item, 'heartbeat_received', false));
        guided_seen(idx) = logical(local_get_field(attempt_item, 'guided_seen', false));
        failure_reason(idx) = string(local_get_field(attempt_item, 'failure_reason', ""));
        if strlength(strtrim(failure_reason(idx))) == 0
            failure_reason(idx) = "нет";
        end
    end
end

attempt_table = table( ...
    label, scheduled_time_s, heartbeat_received, guided_seen, arm, command_ack_arm, failure_reason);
end

function success = local_attempt_success(attempt_table, scheduled_time_s)
success = false;
row_idx = find(abs(attempt_table.scheduled_time_s - scheduled_time_s) < 0.5, 1, 'first');
if ~isempty(row_idx)
    success = logical(attempt_table.arm(row_idx));
end
end

function failure_reason = local_first_attempt_failure(attempt_table, seq_result)
failure_reason = "";
for idx = 1:height(attempt_table)
    if attempt_table.arm(idx)
        continue;
    end
    failure_reason = string(attempt_table.failure_reason(idx));
    if strlength(strtrim(failure_reason)) > 0 && failure_reason ~= "нет"
        return;
    end
end
failure_reason = string(local_pick_failure_reason(seq_result));
end

function ab_result = local_run_ground_clamp_ab_test(cfg, profile_win_path, logs_dir, reports_dir)
ab_console_true = fullfile(logs_dir, 'task_25_ground_clamp_ab_test_case_a_arducopter_console.txt');
ab_console_false = fullfile(logs_dir, 'task_25_ground_clamp_ab_test_case_b_arducopter_console.txt');
log_path = fullfile(logs_dir, 'task_25_ground_clamp_ab_test.txt');
csv_path = fullfile(reports_dir, 'task_25_ground_clamp_ab_test.csv');
mat_path = fullfile(reports_dir, 'task_25_ground_clamp_ab_test.mat');

case_a = local_run_case( ...
    cfg, ...
    profile_win_path, ...
    ab_console_true, ...
    20.0, ...
    "ground_clamp_true", ...
    true);
case_b = local_run_case( ...
    cfg, ...
    profile_win_path, ...
    ab_console_false, ...
    20.0, ...
    "ground_clamp_false", ...
    false);

ab_table = table( ...
    ["A"; "B"], ...
    [true; false], ...
    [case_a.valid_rx_count; case_b.valid_rx_count], ...
    [case_a.json_response_tx_count; case_b.json_response_tx_count], ...
    [case_a.last_frame_count; case_b.last_frame_count], ...
    [case_a.arm_after_20_s; case_b.arm_after_20_s], ...
    [case_a.command_ack; case_b.command_ack], ...
    [string(case_a.failure_reason); string(case_b.failure_reason)], ...
    'VariableNames', { ...
        'case_name', 'diagnostic_ground_clamp', 'valid_rx_count', ...
        'json_response_tx_count', 'last_frame_count', ...
        'arm_after_20_s', 'command_ack_arm', 'failure_reason'});

ab_result = struct();
ab_result.case_a = case_a;
ab_result.case_b = case_b;
ab_result.summary_table = ab_table;

writetable(ab_table, csv_path);
save(mat_path, 'ab_result');

log_lines = strings(0, 1);
log_lines(end + 1, 1) = "TASK-25: A/B-проверка влияния ground clamp на arm";
log_lines(end + 1, 1) = "============================================================";
for idx = 1:height(ab_table)
    log_lines(end + 1, 1) = sprintf('%s: diagnostic_ground_clamp=%s, valid_rx_count=%d, json_response_tx_count=%d, command_ack_arm=%.0f, failure_reason=%s', ...
        char(ab_table.case_name(idx)), ...
        local_bool_text(ab_table.diagnostic_ground_clamp(idx)), ...
        ab_table.valid_rx_count(idx), ...
        ab_table.json_response_tx_count(idx), ...
        ab_table.command_ack_arm(idx), ...
        char(ab_table.failure_reason(idx)));
end
uav.ardupilot.write_utf8_text_file(log_path, strjoin(log_lines, newline) + newline);
end

function text_value = local_make_result_log_text(result, ground_clamp_ab_result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-25: попытки arm в официальном MATLAB backend";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "launch_command: " + string(result.launch_command);
lines(end + 1, 1) = "profile_win_path: " + string(result.profile_win_path);
lines(end + 1, 1) = "diagnostic_ground_clamp: " + local_bool_text(result.diagnostic_ground_clamp);
lines(end + 1, 1) = "valid_rx_count: " + string(result.valid_rx_count);
lines(end + 1, 1) = "json_response_tx_count: " + string(result.json_response_tx_count);
lines(end + 1, 1) = "missed_frame_count: " + string(result.missed_frame_count);
lines(end + 1, 1) = "duplicate_frame_count: " + string(result.duplicate_frame_count);
lines(end + 1, 1) = "last_frame_count: " + string(result.last_frame_count);
lines(end + 1, 1) = "arm_after_20_s: " + local_bool_text(result.arm_after_20_s);
lines(end + 1, 1) = "arm_after_60_s: " + local_bool_text(result.arm_after_60_s);
lines(end + 1, 1) = "arm_after_120_s: " + local_bool_text(result.arm_after_120_s);
lines(end + 1, 1) = "failure_reason: " + local_empty_as_none(result.failure_reason);
lines(end + 1, 1) = sprintf('motor_pwm_us: %.0f .. %.0f', result.motor_pwm_min_us, result.motor_pwm_max_us);
lines(end + 1, 1) = sprintf('motor_cmd_radps: %.6f .. %.6f', result.motor_cmd_min_radps, result.motor_cmd_max_radps);
lines(end + 1, 1) = "attempts:";
for idx = 1:height(result.attempt_table)
    lines(end + 1, 1) = sprintf('  %s: arm=%s, command_ack_arm=%.0f, failure_reason=%s', ...
        char(result.attempt_table.label(idx)), ...
        local_bool_text(result.attempt_table.arm(idx)), ...
        result.attempt_table.command_ack_arm(idx), ...
        char(result.attempt_table.failure_reason(idx)));
end

if isstruct(ground_clamp_ab_result) && isfield(ground_clamp_ab_result, 'summary_table') ...
        && ~isempty(ground_clamp_ab_result.summary_table)
    lines(end + 1, 1) = "ground_clamp_ab_test: выполнен";
    for idx = 1:height(ground_clamp_ab_result.summary_table)
        row = ground_clamp_ab_result.summary_table(idx, :);
        lines(end + 1, 1) = sprintf('  %s: clamp=%s, arm=%s, command_ack_arm=%.0f, failure_reason=%s', ...
            char(row.case_name), ...
            local_bool_text(row.diagnostic_ground_clamp), ...
            local_bool_text(row.arm_after_20_s), ...
            row.command_ack_arm, ...
            char(row.failure_reason));
    end
end

text_value = strjoin(lines, newline) + newline;
end

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
while toc(deadline) < 150.0
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
    seq_result.arm_attempts = struct([]);
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

function value = local_first_finite(values)
value = nan;
for idx = 1:numel(values)
    if isfinite(values(idx))
        value = values(idx);
        return;
    end
end
end

function cfg = local_copy_cfg(cfg)
cfg = orderfields(cfg);
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    file_path = char(string(file_list{idx}));
    if isfile(file_path)
        delete(file_path);
    end
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

function quoted = local_windows_quote(text_value)
quoted = '"' + replace(string(text_value), '"', '""') + '"';
end

function literal = local_ps_quote(text_value)
escaped = replace(string(text_value), "'", "''");
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
