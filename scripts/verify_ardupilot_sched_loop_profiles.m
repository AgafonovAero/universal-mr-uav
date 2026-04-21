%% VERIFY_ARDUPILOT_SCHED_LOOP_PROFILES Проверить применение профилей SCHED_LOOP_RATE.
% Назначение:
%   Для каждого профиля TASK-22 запускает `ArduPilot JSON SITL`,
%   восстанавливает устойчивый обмен `JSON/UDP`, затем через `MAVLink`
%   считывает фактические значения параметров:
%   `SCHED_LOOP_RATE`, `INS_USE`, `INS_USE2`, `INS_USE3`,
%   `INS_ENABLE_MASK`, `LOG_DISARMED`, `ARMING_CHECK`.
%
% Входы:
%   none
%
% Выходы:
%   task_22_sched_loop_profiles_after_boot - структура результата в base
%
% Единицы измерения:
%   частота главного цикла - Гц;
%   счетчики обмена - безразмерные.
%
% Допущения:
%   Профили TASK-22 существуют в каталоге `tools/ardupilot/wsl/`,
%   а `ArduPilot SITL` запускается прямой командой TASK-15.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

cfg = uav.ardupilot.default_json_config();
param_names = [ ...
    "SCHED_LOOP_RATE"; ...
    "SIM_RATE_HZ"; ...
    "INS_USE"; ...
    "INS_USE2"; ...
    "INS_USE3"; ...
    "INS_ENABLE_MASK"; ...
    "LOG_DISARMED"; ...
    "ARMING_CHECK"];

profiles = [ ...
    struct('profile_name', "sched_loop_250", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_250.parm'), 'requested_sched_loop_rate', 250), ...
    struct('profile_name', "sched_loop_200", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_200.parm'), 'requested_sched_loop_rate', 200), ...
    struct('profile_name', "sched_loop_100", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_100.parm'), 'requested_sched_loop_rate', 100)];

rows = repmat(local_empty_row(), numel(profiles), 1);
for idx = 1:numel(profiles)
    rows(idx) = local_run_profile_case(repo_root, logs_dir, profiles(idx), param_names, cfg);
end

result = struct();
result.rows = rows;
result.table = struct2table(rows);
result.log_path = string(fullfile(logs_dir, 'task_22_verify_sched_loop_profiles.txt'));
result.csv_path = string(fullfile(reports_dir, 'task_22_sched_loop_profiles_after_boot.csv'));
result.mat_path = string(fullfile(reports_dir, 'task_22_sched_loop_profiles_after_boot.mat'));

writetable(result.table, char(result.csv_path));
save(char(result.mat_path), 'result');
uav.ardupilot.write_utf8_text_file(result.log_path, local_make_log_text(result));
assignin('base', 'task_22_sched_loop_profiles_after_boot', result);

fprintf('TASK-22: проверка профилей SCHED_LOOP_RATE после запуска\n');
for idx = 1:numel(rows)
    fprintf('  %-16s requested=%g actual=%g baseline=%s reason=%s\n', ...
        char(rows(idx).profile_name), ...
        rows(idx).requested_sched_loop_rate, ...
        rows(idx).actual_sched_loop_rate, ...
        local_bool_text(rows(idx).baseline_restored), ...
        char(rows(idx).failure_reason));
end

function row = local_run_profile_case(repo_root, logs_dir, profile, param_names, cfg)
baseline = local_run_baseline(repo_root, logs_dir, profile.parm_path, "task_22_" + profile.profile_name + "_verify");

row = local_empty_row();
row.profile_name = string(profile.profile_name);
row.parm_path = string(profile.parm_path);
row.requested_sched_loop_rate = double(profile.requested_sched_loop_rate);
row.valid_rx_count = double(baseline.metrics.valid_rx_count);
row.response_tx_count = double(baseline.metrics.response_tx_count);
row.last_frame_count = double(baseline.metrics.last_frame_count);
row.baseline_restored = logical(baseline.baseline_restored);
row.failure_reason = string(baseline.first_failure_reason);
row.arducopter_alive = logical(baseline.process_after_live.is_alive);

if baseline.baseline_restored
    fetch_result = uav.ardupilot.fetch_mavlink_params( ...
        cfg, ...
        "tcp:127.0.0.1:5763", ...
        'ParamNames', param_names, ...
        'Timeout_s', 25.0);

    row.heartbeat_received = logical(fetch_result.heartbeat_received);
    row.fetch_failure_reason = string(fetch_result.failure_reason);
    row.actual_sched_loop_rate = local_param_value(fetch_result, "SCHED_LOOP_RATE");
    row.actual_sim_rate_hz = local_param_value(fetch_result, "SIM_RATE_HZ");
    row.actual_ins_use = local_param_value(fetch_result, "INS_USE");
    row.actual_ins_use2 = local_param_value(fetch_result, "INS_USE2");
    row.actual_ins_use3 = local_param_value(fetch_result, "INS_USE3");
    row.actual_ins_enable_mask = local_param_value(fetch_result, "INS_ENABLE_MASK");
    row.actual_log_disarmed = local_param_value(fetch_result, "LOG_DISARMED");
    row.actual_arming_check = local_param_value(fetch_result, "ARMING_CHECK");

    row.profile_applied = ...
        local_equal_param(row.actual_sched_loop_rate, row.requested_sched_loop_rate) ...
        && local_equal_param(row.actual_ins_use, 1) ...
        && local_equal_param(row.actual_ins_use2, 0) ...
        && local_equal_param(row.actual_ins_use3, 0) ...
        && local_equal_param(row.actual_ins_enable_mask, 1) ...
        && local_equal_param(row.actual_log_disarmed, 1);
else
    row.profile_applied = false;
end

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);
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

function value = local_param_value(fetch_result, param_name)
value = nan;
param_name = char(string(param_name));
if isstruct(fetch_result.param_values) && isfield(fetch_result.param_values, param_name)
    raw_value = fetch_result.param_values.(param_name);
    if ~isempty(raw_value)
        value = double(raw_value);
    end
end
end

function ok = local_equal_param(actual_value, expected_value)
ok = isfinite(actual_value) && abs(actual_value - expected_value) < 1.0e-6;
end

function row = local_empty_row()
row = struct( ...
    'profile_name', "", ...
    'parm_path', "", ...
    'requested_sched_loop_rate', nan, ...
    'actual_sched_loop_rate', nan, ...
    'actual_sim_rate_hz', nan, ...
    'actual_ins_use', nan, ...
    'actual_ins_use2', nan, ...
    'actual_ins_use3', nan, ...
    'actual_ins_enable_mask', nan, ...
    'actual_log_disarmed', nan, ...
    'actual_arming_check', nan, ...
    'heartbeat_received', false, ...
    'profile_applied', false, ...
    'baseline_restored', false, ...
    'valid_rx_count', 0, ...
    'response_tx_count', 0, ...
    'last_frame_count', 0, ...
    'arducopter_alive', false, ...
    'fetch_failure_reason', "", ...
    'failure_reason', "");
end

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-22: проверка применения профилей SCHED_LOOP_RATE";
lines(end + 1, 1) = "============================================================";
for idx = 1:height(result.table)
    row = result.table(idx, :);
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "[" + row.profile_name + "]";
    lines(end + 1, 1) = "Файл параметров: " + row.parm_path;
    lines(end + 1, 1) = "requested SCHED_LOOP_RATE: " + string(row.requested_sched_loop_rate);
    lines(end + 1, 1) = "actual SCHED_LOOP_RATE: " + string(row.actual_sched_loop_rate);
    lines(end + 1, 1) = "actual SIM_RATE_HZ: " + string(row.actual_sim_rate_hz);
    lines(end + 1, 1) = "INS_USE/2/3: " + string(row.actual_ins_use) + "/" + string(row.actual_ins_use2) + "/" + string(row.actual_ins_use3);
    lines(end + 1, 1) = "INS_ENABLE_MASK: " + string(row.actual_ins_enable_mask);
    lines(end + 1, 1) = "LOG_DISARMED: " + string(row.actual_log_disarmed);
    lines(end + 1, 1) = "ARMING_CHECK: " + string(row.actual_arming_check);
    lines(end + 1, 1) = "baseline restored: " + local_bool_text(row.baseline_restored);
    lines(end + 1, 1) = "profile applied: " + local_bool_text(row.profile_applied);
    lines(end + 1, 1) = "valid_rx_count: " + string(row.valid_rx_count);
    lines(end + 1, 1) = "response_tx_count: " + string(row.response_tx_count);
    lines(end + 1, 1) = "last_frame_count: " + string(row.last_frame_count);
    lines(end + 1, 1) = "arducopter alive: " + local_bool_text(row.arducopter_alive);
    lines(end + 1, 1) = "fetch failure reason: " + string(row.fetch_failure_reason);
    lines(end + 1, 1) = "failure reason: " + string(row.failure_reason);
end
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    if isfile(file_list{idx})
        delete(file_list{idx});
    end
end
end
