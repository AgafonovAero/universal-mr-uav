%% RUN_ARDUPILOT_GYRO_LOOP_RATE_EXPERIMENTS Выполнить серию опытов TASK-22 по SCHED_LOOP_RATE.
% Назначение:
%   Проверяет, как изменение `SCHED_LOOP_RATE` влияет на прохождение
%   предполетной проверки по частоте гироскопа в режиме
%   `ArduPilot JSON + MATLAB-модель`.
%
% Входы:
%   none
%
% Выходы:
%   task_22_gyro_loop_rate_experiments - структура результата в base
%
% Единицы измерения:
%   частоты - Гц;
%   периоды - секунды;
%   ШИМ - микросекунды;
%   команды частоты вращения - рад/с.
%
% Допущения:
%   Штатная простая калибровка акселерометра выполняется один раз перед
%   серией опытов и сохраняется в рабочем каталоге `SITL`.

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
case_defs = local_make_case_definitions(repo_root);
case_results = repmat(local_empty_case_result(), numel(case_defs), 1);

precal = local_run_precalibration(repo_root, logs_dir, cfg);

param_names = [ ...
    "SCHED_LOOP_RATE"; ...
    "SIM_RATE_HZ"; ...
    "INS_USE"; ...
    "INS_USE2"; ...
    "INS_USE3"; ...
    "INS_ENABLE_MASK"; ...
    "LOG_DISARMED"; ...
    "ARMING_CHECK"];

for idx = 1:numel(case_defs)
    case_results(idx) = local_run_case(repo_root, logs_dir, reports_dir, cfg, case_defs(idx), param_names, precal);
end

result = struct();
result.precalibration = precal;
result.cases = case_results;
result.table = struct2table(case_results);
result.arm_succeeded = any([case_results.arm_succeeded]);
result.best_case_id = local_pick_best_case(case_results);
result.log_path = string(fullfile(logs_dir, 'task_22_gyro_loop_rate_experiments.txt'));
result.csv_path = string(fullfile(reports_dir, 'task_22_gyro_loop_rate_experiments.csv'));
result.mat_path = string(fullfile(reports_dir, 'task_22_gyro_loop_rate_experiments.mat'));

writetable(result.table, char(result.csv_path));
save(char(result.mat_path), 'result');
uav.ardupilot.write_utf8_text_file(result.log_path, local_make_summary_log(result));
assignin('base', 'task_22_gyro_loop_rate_experiments', result);

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);

fprintf('TASK-22: серия опытов по частоте гироскопа и SCHED_LOOP_RATE\n');
fprintf('  простая калибровка акселерометра готова   : %s\n', local_bool_text(precal.calibration_ready));
fprintf('  лучший опыт                               : %s\n', char(result.best_case_id));
fprintf('  arm подтвержден хотя бы в одном опыте     : %s\n', local_bool_text(result.arm_succeeded));

function precal = local_run_precalibration(repo_root, logs_dir, cfg)
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');
baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_22_precal");

precal = struct();
precal.parm_path = string(parm_path);
precal.calibration_ready = false;
precal.command_ack = nan;
precal.failure_reason = string(baseline.first_failure_reason);
precal.status_text_tail = "";
precal.valid_rx_count = double(baseline.metrics.valid_rx_count);
precal.response_tx_count = double(baseline.metrics.response_tx_count);

if ~baseline.baseline_restored
    return;
end

[helper_info, cleanup_list] = local_prepare_simple_calibration_helper(cfg);
cleanup_obj = onCleanup(@() local_cleanup_temp(cleanup_list)); %#ok<NASGU>
system(helper_info.launch_command); %#ok<ASGLU>

assignin('base', 'ardupilot_live_backend_duration_s', 20.0);
cleanup_base = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
run(fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m'));
live_result = evalin('base', 'ardupilot_json_udp_live_backend');
metrics = uav.ardupilot.summarize_live_backend_metrics(live_result);
helper_result = local_read_helper_result(helper_info.output_json_path_win, helper_info.output_log_path_win);

precal.valid_rx_count = double(metrics.valid_rx_count);
precal.response_tx_count = double(metrics.response_tx_count);
precal.command_ack = double(helper_result.ack_result);
precal.failure_reason = string(helper_result.failure_reason);
precal.status_text_tail = string(helper_result.output_excerpt);
precal.calibration_ready = helper_result.helper_completed && helper_result.ack_result == 0;

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
pause(2.0);

if precal.calibration_ready
    post_baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_22_postcal");
    precal.valid_rx_count = double(post_baseline.metrics.valid_rx_count);
    precal.response_tx_count = double(post_baseline.metrics.response_tx_count);
    if ~post_baseline.baseline_restored
        precal.calibration_ready = false;
        precal.failure_reason = string(post_baseline.first_failure_reason);
    end
    uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
end
end

function case_result = local_run_case(repo_root, logs_dir, reports_dir, cfg, case_def, param_names, precal)
baseline = local_run_baseline(repo_root, logs_dir, case_def.parm_path, "task_22_case_" + case_def.case_id);

case_result = local_empty_case_result();
case_result.case_id = string(case_def.case_id);
case_result.case_title = string(case_def.case_title);
case_result.parm_path = string(case_def.parm_path);
case_result.precalibration_ready = logical(precal.calibration_ready);
case_result.valid_rx_count = double(baseline.metrics.valid_rx_count);
case_result.response_tx_count = double(baseline.metrics.response_tx_count);
case_result.json_tx_count = double(baseline.metrics.json_tx_count);
case_result.last_frame_count = double(baseline.metrics.last_frame_count);
case_result.arducopter_alive = logical(baseline.process_after_live.is_alive);
case_result.failure_reason = string(baseline.first_failure_reason);
case_result.arm_succeeded = false;

if baseline.baseline_restored
    fetch_result = uav.ardupilot.fetch_mavlink_params( ...
        cfg, ...
        "tcp:127.0.0.1:5763", ...
        'ParamNames', param_names, ...
        'Timeout_s', 25.0);
    case_result.actual_sched_loop_rate = local_param_value(fetch_result, "SCHED_LOOP_RATE");
    case_result.actual_sim_rate_hz = local_param_value(fetch_result, "SIM_RATE_HZ");
    case_result.actual_ins_use = local_param_value(fetch_result, "INS_USE");
    case_result.actual_ins_use2 = local_param_value(fetch_result, "INS_USE2");
    case_result.actual_ins_use3 = local_param_value(fetch_result, "INS_USE3");
    case_result.actual_ins_enable_mask = local_param_value(fetch_result, "INS_ENABLE_MASK");
    case_result.actual_log_disarmed = local_param_value(fetch_result, "LOG_DISARMED");
    case_result.actual_arming_check = local_param_value(fetch_result, "ARMING_CHECK");

    arm_response = local_run_arm_response(repo_root, case_def.case_id);
    metrics = uav.ardupilot.summarize_live_backend_metrics(arm_response.live_result);
    case_result.valid_rx_count = double(metrics.valid_rx_count);
    case_result.response_tx_count = double(metrics.response_tx_count);
    case_result.json_tx_count = double(metrics.json_tx_count);
    case_result.last_frame_count = double(metrics.last_frame_count);
    case_result.average_valid_rx_rate_hz = double(metrics.valid_rx_rate_hz);
    case_result.average_response_tx_rate_hz = double(metrics.response_tx_rate_hz);
    case_result.average_exchange_period_s = double(metrics.response_tx_period_mean_s);
    case_result.p95_exchange_period_s = double(metrics.response_tx_period_p95_s);
    case_result.gyro_rate_hz = local_extract_gyro_rate_hz(arm_response.arm_attempt.status_texts);
    if isfinite(case_result.actual_sched_loop_rate)
        case_result.min_required_gyro_rate_hz = 1.8 * case_result.actual_sched_loop_rate;
    end
    case_result.command_ack = double(arm_response.arm_attempt.ack_result);
    case_result.arm_succeeded = logical(arm_response.arm_attempt.arm_succeeded);
    case_result.failure_reason = local_pick_first_failure_reason(arm_response.arm_attempt);
    case_result.status_text_tail = local_tail_status_texts(arm_response.arm_attempt.status_texts, 8);
    [case_result.motor_pwm_min_us, case_result.motor_pwm_max_us] = local_range_from_matrix( ...
        local_extract_pwm_matrix(arm_response.live_result.sitl_output));
    [case_result.motor_cmd_min_radps, case_result.motor_cmd_max_radps] = local_range_from_matrix( ...
        double(arm_response.live_result.motor_cmd_radps));
    case_result.arducopter_alive = local_is_wsl_pid_alive(cfg.wsl_distro_name, baseline.process_after_live.pid);
end

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);
end

function [helper_info, cleanup_list] = local_prepare_simple_calibration_helper(cfg)
python_script_path_win = [tempname, '_task22_simple_cal.py'];
launcher_script_path_win = [tempname, '_task22_simple_cal.ps1'];
output_json_path_win = [tempname, '_task22_simple_cal.json'];
output_log_path_win = [tempname, '_task22_simple_cal.log'];

python_script_path_wsl = uav.ardupilot.windows_to_wsl_path(python_script_path_win);
output_json_path_wsl = uav.ardupilot.windows_to_wsl_path(output_json_path_win);
output_log_path_wsl = uav.ardupilot.windows_to_wsl_path(output_log_path_win);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);

py_lines = strings(0, 1);
py_lines(end + 1, 1) = "from pymavlink import mavutil";
py_lines(end + 1, 1) = "import json";
py_lines(end + 1, 1) = "import time";
py_lines(end + 1, 1) = "from pathlib import Path";
py_lines(end + 1, 1) = "result = {'helper_completed': True, 'ack_result': None, 'failure_reason': '', 'output_excerpt': ''}";
py_lines(end + 1, 1) = "status_texts = []";
py_lines(end + 1, 1) = "try:";
py_lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=245, source_component=190)";
py_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
py_lines(end + 1, 1) = "    if hb is None:";
py_lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT перед простой калибровкой акселерометра.')";
py_lines(end + 1, 1) = "    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 4, 0, 0)";
py_lines(end + 1, 1) = "    deadline = time.time() + 25.0";
py_lines(end + 1, 1) = "    while time.time() < deadline:";
py_lines(end + 1, 1) = "        msg = master.recv_match(blocking=True, timeout=1)";
py_lines(end + 1, 1) = "        if msg is None:";
py_lines(end + 1, 1) = "            continue";
py_lines(end + 1, 1) = "        mtype = msg.get_type()";
py_lines(end + 1, 1) = "        if mtype == 'COMMAND_ACK' and int(msg.command) == int(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION):";
py_lines(end + 1, 1) = "            result['ack_result'] = int(msg.result)";
py_lines(end + 1, 1) = "        elif mtype == 'STATUSTEXT':";
py_lines(end + 1, 1) = "            text = str(getattr(msg, 'text', '')).strip()";
py_lines(end + 1, 1) = "            if text:";
py_lines(end + 1, 1) = "                status_texts.append(text)";
py_lines(end + 1, 1) = "    if status_texts:";
py_lines(end + 1, 1) = "        result['output_excerpt'] = ' | '.join(status_texts[-8:])";
py_lines(end + 1, 1) = "    if result['ack_result'] not in (0, None) and not result['failure_reason']:";
py_lines(end + 1, 1) = "        result['failure_reason'] = 'Команда калибровки отклонена с кодом {}'.format(result['ack_result'])";
py_lines(end + 1, 1) = "except Exception as exc:";
py_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
py_lines(end + 1, 1) = "Path(" + local_py(output_json_path_wsl) + ").write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')";
uav.ardupilot.write_utf8_text_file(python_script_path_win, strjoin(py_lines, newline) + newline);

launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$ArgumentList = @('-d','" + string(cfg.wsl_distro_name) + "','--','bash','-lc','sleep 5; " + python_command_wsl + " " + python_script_path_wsl + " >" + output_log_path_wsl + " 2>&1')";
launcher_lines(end + 1, 1) = "Start-Process -WindowStyle Hidden -FilePath 'wsl.exe' -ArgumentList $ArgumentList | Out-Null";
uav.ardupilot.write_utf8_text_file(launcher_script_path_win, strjoin(launcher_lines, newline) + newline);

helper_info = struct();
helper_info.launch_command = sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', char(launcher_script_path_win));
helper_info.output_json_path_win = string(output_json_path_win);
helper_info.output_log_path_win = string(output_log_path_win);
cleanup_list = {python_script_path_win, launcher_script_path_win, output_json_path_win, output_log_path_win};
end

function arm_response = local_run_arm_response(repo_root, case_id)
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
arm_mat_tmp = [tempname, '_task22_case_', lower(char(case_id)), '_arm.mat'];
arm_csv_tmp = [tempname, '_task22_case_', lower(char(case_id)), '_arm.csv'];
cleanup_obj = onCleanup(@() local_cleanup_temp({arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

assignin('base', 'ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
assignin('base', 'ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
assignin('base', 'ardupilot_live_backend_duration_s', 25.0);
assignin('base', 'ardupilot_arm_attempt_delay_s', 10.0);
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_arm_pwm_response_mat_path'', ' ...
    '''ardupilot_arm_pwm_response_csv_path'', ' ...
    '''ardupilot_live_backend_duration_s'', ' ...
    '''ardupilot_arm_attempt_delay_s'');'])); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
arm_response = evalin('base', 'ardupilot_arm_pwm_response');

copyfile(arm_mat_tmp, fullfile(reports_dir, sprintf('task_22_case_%s_arm.mat', lower(char(case_id)))), 'f');
copyfile(arm_csv_tmp, fullfile(reports_dir, sprintf('task_22_case_%s_arm.csv', lower(char(case_id)))), 'f');
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

function helper_result = local_read_helper_result(json_path_win, log_path_win)
deadline = tic;
while toc(deadline) < 35.0
    if isfile(json_path_win)
        break;
    end
    pause(0.2);
end

helper_result = struct('helper_completed', false, 'ack_result', nan, 'failure_reason', "", 'output_excerpt', "");
if isfile(json_path_win)
    decoded = jsondecode(fileread(json_path_win));
    helper_result.helper_completed = logical(local_get_field(decoded, 'helper_completed', false));
    helper_result.ack_result = double(local_get_field(decoded, 'ack_result', nan));
    helper_result.failure_reason = string(local_get_field(decoded, 'failure_reason', ""));
    helper_result.output_excerpt = string(local_get_field(decoded, 'output_excerpt', ""));
elseif isfile(log_path_win)
    helper_result.failure_reason = "Не сформирован JSON-результат простой калибровки.";
    helper_result.output_excerpt = local_tail_excerpt(fileread(log_path_win));
end
end

function value = local_get_field(data, field_name, default_value)
if isstruct(data) && isfield(data, field_name)
    value = data.(field_name);
else
    value = default_value;
end
end

function text_value = local_tail_excerpt(raw_text)
lines = splitlines(string(raw_text));
lines = strtrim(lines);
lines(lines == "") = [];
if isempty(lines)
    text_value = "";
else
    lines = lines(max(1, numel(lines)-7):end);
    text_value = strjoin(lines, " | ");
end
end

function case_defs = local_make_case_definitions(repo_root)
case_defs = [ ...
    struct('case_id', "A", 'case_title', "Текущий профиль TASK-21", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm')), ...
    struct('case_id', "B", 'case_title', "SCHED_LOOP_RATE = 250", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_250.parm')), ...
    struct('case_id', "C", 'case_title', "SCHED_LOOP_RATE = 200", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_200.parm')), ...
    struct('case_id', "D", 'case_title', "SCHED_LOOP_RATE = 100", 'parm_path', fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_100.parm'))];
end

function case_result = local_empty_case_result()
case_result = struct( ...
    'case_id', "", ...
    'case_title', "", ...
    'parm_path', "", ...
    'precalibration_ready', false, ...
    'actual_sched_loop_rate', nan, ...
    'actual_sim_rate_hz', nan, ...
    'actual_ins_use', nan, ...
    'actual_ins_use2', nan, ...
    'actual_ins_use3', nan, ...
    'actual_ins_enable_mask', nan, ...
    'actual_log_disarmed', nan, ...
    'actual_arming_check', nan, ...
    'gyro_rate_hz', nan, ...
    'min_required_gyro_rate_hz', nan, ...
    'valid_rx_count', 0, ...
    'response_tx_count', 0, ...
    'json_tx_count', 0, ...
    'last_frame_count', 0, ...
    'average_valid_rx_rate_hz', nan, ...
    'average_response_tx_rate_hz', nan, ...
    'average_exchange_period_s', nan, ...
    'p95_exchange_period_s', nan, ...
    'command_ack', nan, ...
    'arm_succeeded', false, ...
    'failure_reason', "", ...
    'status_text_tail', "", ...
    'motor_pwm_min_us', nan, ...
    'motor_pwm_max_us', nan, ...
    'motor_cmd_min_radps', nan, ...
    'motor_cmd_max_radps', nan, ...
    'arducopter_alive', false);
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

function rate_hz = local_extract_gyro_rate_hz(status_texts)
rate_hz = nan;
status_texts = string(status_texts(:));
for idx = 1:numel(status_texts)
    one_text = status_texts(idx);
    if contains(one_text, "Gyro")
        token = regexp(char(one_text), '(\d+(?:\.\d+)?)Hz', 'tokens', 'once');
        if ~isempty(token)
            rate_hz = str2double(token{1});
            return;
        end
    end
end
end

function reason_text = local_pick_first_failure_reason(arm_attempt)
reason_text = string(arm_attempt.failure_reason);
status_texts = string(arm_attempt.status_texts(:));
for idx = 1:numel(status_texts)
    if startsWith(strtrim(status_texts(idx)), "Arm:")
        reason_text = strtrim(status_texts(idx));
        return;
    end
end
if strlength(strtrim(reason_text)) == 0 && ...
        isfinite(double(arm_attempt.ack_result)) && ...
        double(arm_attempt.ack_result) ~= 0
    reason_text = "Команда взведения отклонена с кодом " + string(double(arm_attempt.ack_result)) + ".";
end
end

function text_value = local_tail_status_texts(status_texts, tail_count)
status_texts = string(status_texts(:));
status_texts = strtrim(status_texts);
status_texts(status_texts == "") = [];
if isempty(status_texts)
    text_value = "";
else
    tail_count = min(numel(status_texts), tail_count);
    text_value = strjoin(status_texts(end-tail_count+1:end), " | ");
end
end

function pwm_matrix = local_extract_pwm_matrix(sitl_output)
row_count = numel(sitl_output);
pwm_matrix = nan(row_count, 4);
for idx = 1:row_count
    if sitl_output(idx).valid
        pwm_matrix(idx, :) = double(sitl_output(idx).motor_pwm_us(:)).';
    end
end
end

function [min_value, max_value] = local_range_from_matrix(value_matrix)
values = double(value_matrix(:));
values = values(isfinite(values));
if isempty(values)
    min_value = nan;
    max_value = nan;
else
    min_value = min(values);
    max_value = max(values);
end
end

function best_case_id = local_pick_best_case(case_results)
scores = zeros(numel(case_results), 1);
for idx = 1:numel(case_results)
    scores(idx) = double(case_results(idx).arm_succeeded) * 1.0e6 ...
        + double(case_results(idx).response_tx_count) ...
        + 0.001 * double(case_results(idx).valid_rx_count);
end
[~, best_idx] = max(scores);
best_case_id = string(case_results(best_idx).case_id);
end

function text_value = local_make_summary_log(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-22: серия опытов по частоте гироскопа и SCHED_LOOP_RATE";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Предварительная простая калибровка: " + local_bool_text(result.precalibration.calibration_ready);
lines(end + 1, 1) = "ACK калибровки: " + string(result.precalibration.command_ack);
lines(end + 1, 1) = "Причина precalibration: " + string(result.precalibration.failure_reason);
for idx = 1:height(result.table)
    row = result.table(idx, :);
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "[" + row.case_id + "] " + row.case_title;
    lines(end + 1, 1) = "Файл параметров: " + row.parm_path;
    lines(end + 1, 1) = "Фактический SCHED_LOOP_RATE [Hz]: " + string(row.actual_sched_loop_rate);
    lines(end + 1, 1) = "Фактическая частота гироскопа [Hz]: " + string(row.gyro_rate_hz);
    lines(end + 1, 1) = "Требуемая минимальная частота [Hz]: " + string(row.min_required_gyro_rate_hz);
    lines(end + 1, 1) = "valid_rx_count: " + string(row.valid_rx_count);
    lines(end + 1, 1) = "response_tx_count: " + string(row.response_tx_count);
    lines(end + 1, 1) = "json_tx_count: " + string(row.json_tx_count);
    lines(end + 1, 1) = "last_frame_count: " + string(row.last_frame_count);
    lines(end + 1, 1) = "arm: " + local_bool_text(row.arm_succeeded);
    lines(end + 1, 1) = "COMMAND_ACK: " + string(row.command_ack);
    lines(end + 1, 1) = "Причина: " + string(row.failure_reason);
    lines(end + 1, 1) = "STATUSTEXT: " + string(row.status_text_tail);
    lines(end + 1, 1) = sprintf('motor_pwm_us range: [%.6f %.6f]', row.motor_pwm_min_us, row.motor_pwm_max_us);
    lines(end + 1, 1) = sprintf('motor_cmd_radps range: [%.6f %.6f]', row.motor_cmd_min_radps, row.motor_cmd_max_radps);
    lines(end + 1, 1) = "arducopter alive: " + local_bool_text(row.arducopter_alive);
end
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Лучший опыт: " + result.best_case_id;
lines(end + 1, 1) = "Arm подтвержден хотя бы в одном опыте: " + local_bool_text(result.arm_succeeded);
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
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

function flag = local_is_wsl_pid_alive(distro_name, pid_value)
flag = false;
if ~isfinite(pid_value) || pid_value <= 0
    return;
end
command_text = sprintf('wsl -d %s -- bash -lc "kill -0 %d >/dev/null 2>&1"', ...
    char(distro_name), round(pid_value));
[status_code, ~] = system(command_text);
flag = status_code == 0;
end
