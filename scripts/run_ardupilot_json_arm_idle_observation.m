%% RUN_ARDUPILOT_JSON_ARM_IDLE_OBSERVATION Наблюдать режим arm без команды взлета.
% Назначение:
%   Поднимает рабочий стенд `ArduPilot JSON + MATLAB-модель`, выполняет
%   команду взведения без команды `takeoff` и сохраняет 20-секундную
%   историю ШИМ, команд частоты вращения, тяги и состояния модели.
%
% Входы:
%   none
%
% Выходы:
%   task_23_arm_idle_observation - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   частота вращения - рад/с;
%   высота - метры;
%   вертикальная скорость - м/с.
%
% Допущения:
%   Используется рабочий профиль TASK-22 с фактическим `SCHED_LOOP_RATE=200`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
log_path = fullfile(logs_dir, 'task_23_arm_idle_observation.txt');
csv_path = fullfile(reports_dir, 'task_23_arm_idle_observation.csv');
mat_path = fullfile(reports_dir, 'task_23_arm_idle_observation.mat');
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
        "TASK-23: idle arm observation не выполнен." + newline + ...
        "Причина: " + string(result.failure_reason) + newline);
    error('uav:task23:idle:PrecalibrationFailed', ...
        'Не подтверждена простая калибровка акселерометра перед idle arm: %s', char(result.failure_reason));
end

baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_23_idle");
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
        "TASK-23: idle arm observation не выполнен." + newline + ...
        "Причина: " + string(result.failure_reason) + newline);
    error('uav:task23:idle:BaselineFailed', ...
        'Не восстановлен устойчивый обмен перед idle arm: %s', char(result.failure_reason));
end

seq_cfg = struct();
seq_cfg.wsl_distro_name = cfg.wsl_distro_name;
seq_cfg.connection_candidates = ["tcp:127.0.0.1:5763"; "tcp:127.0.0.1:5762"];
seq_cfg.mode_name = "STABILIZE";
seq_cfg.wait_before_command_s = 0.0;
seq_cfg.heartbeat_timeout_s = 15.0;
seq_cfg.arm_timeout_s = 20.0;
seq_cfg.monitor_duration_s = 20.0;
seq_cfg.takeoff_alt_m = nan;
seq_cfg.sample_period_s = 0.5;
command_info = uav.ardupilot.make_pymavlink_sequence_command(seq_cfg);
[launcher_info, cleanup_list] = local_prepare_sequence_launcher(cfg, command_info);
cleanup_tmp = onCleanup(@() local_cleanup_temp(cleanup_list)); %#ok<NASGU>

system(launcher_info.launch_command); %#ok<ASGLU>
assignin('base', 'ardupilot_live_backend_duration_s', 30.0);
cleanup_base = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
run(fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m'));
live_result = evalin('base', 'ardupilot_json_udp_live_backend');
metrics = uav.ardupilot.summarize_live_backend_metrics(live_result);
seq_result = local_read_sequence_result(command_info.output_json_path_win, launcher_info.helper_log_path_win);
history_table = local_make_history_table(live_result, seq_result.telemetry_table, params);

result = struct();
result.executed = true;
result.parm_path = string(parm_path);
result.precalibration = precalibration;
result.baseline = baseline;
result.sequence_result = seq_result;
result.live_result = live_result;
result.metrics = metrics;
result.history_table = history_table;
result.arm_succeeded = logical(local_getfield_or(seq_result, 'arm_succeeded', false));
result.command_ack = double(local_getfield_or(seq_result, 'arm_ack_result', nan));
if result.arm_succeeded
    result.failure_reason = "";
else
    result.failure_reason = string(local_pick_failure_reason(seq_result));
end
result.armed_observed = any(history_table.armed);
result.mode_tail = local_tail_mode(history_table.mode);
result.motor_pwm_range_us = [min(history_table.pwm_1_us, [], 'omitnan'), max(history_table.pwm_1_us, [], 'omitnan'); ...
                             min(history_table.pwm_2_us, [], 'omitnan'), max(history_table.pwm_2_us, [], 'omitnan'); ...
                             min(history_table.pwm_3_us, [], 'omitnan'), max(history_table.pwm_3_us, [], 'omitnan'); ...
                             min(history_table.pwm_4_us, [], 'omitnan'), max(history_table.pwm_4_us, [], 'omitnan')];
result.motor_cmd_range_radps = [min(history_table.motor_1_radps, [], 'omitnan'), max(history_table.motor_1_radps, [], 'omitnan'); ...
                                min(history_table.motor_2_radps, [], 'omitnan'), max(history_table.motor_2_radps, [], 'omitnan'); ...
                                min(history_table.motor_3_radps, [], 'omitnan'), max(history_table.motor_3_radps, [], 'omitnan'); ...
                                min(history_table.motor_4_radps, [], 'omitnan'), max(history_table.motor_4_radps, [], 'omitnan')];
result.max_total_thrust_to_weight = max(history_table.total_thrust_to_weight, [], 'omitnan');
result.altitude_changed = (max(history_table.altitude_m, [], 'omitnan') - min(history_table.altitude_m, [], 'omitnan')) > 0.05;
result.max_altitude_m = max(history_table.altitude_m, [], 'omitnan');

writetable(history_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_23_arm_idle_observation', result);
log_text = local_make_log_text(result);
if ismissing(log_text) || strlength(strtrim(log_text)) == 0
    log_text = "TASK-23: idle arm observation" + newline + ...
        "Причина: " + local_empty_as_none(result.failure_reason) + newline;
end
uav.ardupilot.write_utf8_text_file(log_path, log_text);

fprintf('TASK-23: idle arm observation\n');
fprintf('  arm accepted                           : %s\n', local_bool_text(result.arm_succeeded));
fprintf('  COMMAND_ACK                            : %.0f\n', result.command_ack);
fprintf('  valid_rx_count                         : %d\n', metrics.valid_rx_count);
fprintf('  response_tx_count                      : %d\n', metrics.response_tx_count);
fprintf('  motor pwm range [us]                   : %.0f .. %.0f\n', ...
    min(history_table{:, ["pwm_1_us","pwm_2_us","pwm_3_us","pwm_4_us"]}, [], 'all', 'omitnan'), ...
    max(history_table{:, ["pwm_1_us","pwm_2_us","pwm_3_us","pwm_4_us"]}, [], 'all', 'omitnan'));

function parm_path = local_select_parm_path(repo_root)
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_200.parm');
if evalin('base', 'exist(''task23_active_parm_path'', ''var'')')
    candidate = string(evalin('base', 'task23_active_parm_path'));
    if strlength(candidate) > 0
        parm_path = char(candidate);
    end
end
if ~isfile(parm_path)
    error('uav:task23:idle:MissingParm', 'Не найден профиль TASK-23: %s', parm_path);
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

function [launcher_info, cleanup_list] = local_prepare_sequence_launcher(cfg, command_info)
helper_log_path_win = [tempname, '_task23_idle_helper.log'];
launcher_path_win = [tempname, '_task23_idle_helper.ps1'];
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);
helper_log_path_wsl = uav.ardupilot.windows_to_wsl_path(helper_log_path_win);
bash_payload = "sleep 10.000; " + string(python_command_wsl) + " " + ...
    local_bash_quote(command_info.python_script_path_wsl) + " >" + ...
    local_bash_quote(helper_log_path_wsl) + " 2>&1";

launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$ArgumentList = @(";
launcher_lines(end + 1, 1) = "  '-d',";
launcher_lines(end + 1, 1) = "  '" + string(cfg.wsl_distro_name) + "',";
launcher_lines(end + 1, 1) = "  '--',";
launcher_lines(end + 1, 1) = "  'bash',";
launcher_lines(end + 1, 1) = "  '-lc',";
launcher_lines(end + 1, 1) = "  " + local_ps_quote(bash_payload);
launcher_lines(end + 1, 1) = ")";
launcher_lines(end + 1, 1) = "Start-Process -WindowStyle Hidden -FilePath 'wsl.exe' -ArgumentList $ArgumentList | Out-Null";
uav.ardupilot.write_utf8_text_file(launcher_path_win, strjoin(launcher_lines, newline) + newline);

launcher_info = struct();
launcher_info.launch_command = sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', launcher_path_win);
launcher_info.helper_log_path_win = string(helper_log_path_win);
cleanup_list = {char(command_info.python_script_path_win), char(command_info.output_json_path_win), helper_log_path_win, launcher_path_win};
end

function seq_result = local_read_sequence_result(json_path_win, helper_log_path_win)
deadline = tic;
while toc(deadline) < 45.0
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
    if isfile(char(helper_log_path_win))
        seq_result.failure_reason = string(local_tail_excerpt(fileread(char(helper_log_path_win))));
    else
        seq_result.failure_reason = "Не сформирован JSON-результат pymavlink helper.";
    end
end
end

function table_value = local_make_history_table(live_result, telemetry_table, params)
n = numel(live_result.time_s);
mode_hist = strings(n, 1);
armed_hist = false(n, 1);
time_s = double(live_result.time_s(:));
valid_rx_count = cumsum(double(live_result.udp_valid_rx_count_hist(:)));
response_flags = arrayfun(@(item) double(logical(item.tx_ok && item.rx_valid)), live_result.exchange_diag(:));
response_tx_count = cumsum(response_flags(:));

if ~isempty(telemetry_table)
    telem_time = double(telemetry_table.t_s(:));
    telem_armed = logical(telemetry_table.armed(:));
    telem_mode = string(telemetry_table.mode(:));
    idx = 1;
    current_mode = "";
    current_armed = false;
    for k = 1:n
        while idx <= numel(telem_time) && telem_time(idx) <= time_s(k)
            current_mode = telem_mode(idx);
            current_armed = telem_armed(idx);
            idx = idx + 1;
        end
        mode_hist(k) = current_mode;
        armed_hist(k) = current_armed;
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
        'frame_count', ...
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
selected_method = string(local_getfield_or(calibration_result, 'selected_method', ""));
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

precalibration.calibration_ready = logical(local_getfield_or(calibration_result, 'simple_calibration_performed', false) || ...
    local_getfield_or(calibration_result, 'force_save_used', false) || ...
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

function text_value = local_make_log_text(result)
metrics = result.metrics;
seq_result = result.sequence_result;
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-23: наблюдение arm без takeoff";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль запуска: " + string(result.parm_path);
lines(end + 1, 1) = "precalibration ready: " + local_bool_text(result.precalibration.calibration_ready);
lines(end + 1, 1) = "precalibration method: " + local_empty_as_none(result.precalibration.selected_method);
lines(end + 1, 1) = "precalibration ACK: " + string(result.precalibration.command_ack);
lines(end + 1, 1) = "baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "valid_rx_count during idle: " + string(metrics.valid_rx_count);
lines(end + 1, 1) = "response_tx_count during idle: " + string(metrics.response_tx_count);
lines(end + 1, 1) = "last_frame_count: " + string(metrics.last_frame_count);
lines(end + 1, 1) = "arm succeeded: " + local_bool_text(result.arm_succeeded);
lines(end + 1, 1) = "COMMAND_ACK: " + string(result.command_ack);
lines(end + 1, 1) = "armed observed in telemetry: " + local_bool_text(result.armed_observed);
lines(end + 1, 1) = "mode tail: " + local_empty_as_none(result.mode_tail);
lines(end + 1, 1) = "failure reason: " + local_empty_as_none(result.failure_reason);
if isfield(seq_result, 'status_texts') && ~isempty(seq_result.status_texts)
    tail = seq_result.status_texts(max(1, numel(seq_result.status_texts)-7):end);
    lines(end + 1, 1) = "STATUSTEXT tail: " + strjoin(string(tail), " | ");
end
lines(end + 1, 1) = sprintf('max total thrust / weight: %.6f', result.max_total_thrust_to_weight);
lines(end + 1, 1) = "altitude changed: " + local_bool_text(result.altitude_changed);
lines(end + 1, 1) = sprintf('max altitude [m]: %.6f', result.max_altitude_m);
text_value = strjoin(lines, newline) + newline;
end

function value = local_getfield_or(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name) && ~isempty(data.(field_name))
    value = data.(field_name);
else
    value = default_value;
end
end

function text_value = local_pick_failure_reason(seq_result)
if isstruct(seq_result) && isfield(seq_result, 'status_texts') && ~isempty(seq_result.status_texts)
    status_texts = string(seq_result.status_texts(:));
    match_idx = find(startsWith(status_texts, "Arm:") | startsWith(status_texts, "PreArm:"), 1, 'first');
    if ~isempty(match_idx)
        text_value = status_texts(match_idx);
        return;
    end
end
if isstruct(seq_result) && isfield(seq_result, 'failure_reason') && strlength(string(seq_result.failure_reason)) > 0
    text_value = string(seq_result.failure_reason);
elseif isstruct(seq_result) && isfield(seq_result, 'status_texts') && ~isempty(seq_result.status_texts)
    text_value = string(seq_result.status_texts(end));
else
    text_value = "";
end
end

function text_value = local_tail_mode(mode_values)
mode_values = string(mode_values(:));
mode_values = mode_values(strlength(mode_values) > 0);
if isempty(mode_values)
    text_value = "";
else
    tail = mode_values(max(1, numel(mode_values)-4):end);
    text_value = strjoin(tail, " -> ");
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
