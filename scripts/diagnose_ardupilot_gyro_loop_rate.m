%% DIAGNOSE_ARDUPILOT_GYRO_LOOP_RATE Диагностировать частоту гироскопа и главного цикла ArduPilot.
% Назначение:
%   Запускает рабочий JSON-режим `ArduPilot + MATLAB-модель`,
%   при необходимости выполняет штатную простую калибровку
%   акселерометра, затем собирает фактические параметры,
%   метрики обмена и результат попытки `arm`, чтобы зафиксировать
%   реальный блокер по частоте гироскопа.
%
% Входы:
%   none
%
% Выходы:
%   task_22_gyro_loop_rate_diagnostics - структура результата в base
%
% Единицы измерения:
%   частоты - Гц;
%   периоды - секунды;
%   ШИМ - микросекунды;
%   команды частоты вращения - рад/с.
%
% Допущения:
%   Используется профиль TASK-20 с одним активным ИНС:
%   `tools/ardupilot/wsl/task20_single_accel_enable_mask.parm`.

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
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');
if ~isfile(parm_path)
    error('uav:task22:diagnoseGyroLoop:MissingParm', ...
        'Не найден профиль TASK-20: %s', parm_path);
end

param_names = [ ...
    "SCHED_LOOP_RATE"; ...
    "SIM_RATE_HZ"; ...
    "INS_USE"; ...
    "INS_USE2"; ...
    "INS_USE3"; ...
    "INS_ENABLE_MASK"; ...
    "LOG_DISARMED"; ...
    "ARMING_CHECK"];

calibration = local_run_simple_calibration(repo_root, cfg);
baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_22_diag_gyro");

result = struct();
result.parm_path = string(parm_path);
result.calibration = calibration;
result.baseline = baseline;
result.fetch = struct();
result.arm_response = struct();
result.metrics = struct();
result.sched_loop_rate_hz = nan;
result.sim_rate_hz = nan;
result.gyro_rate_hz = nan;
result.min_required_gyro_rate_hz = nan;
result.max_sched_loop_rate_hz = nan;
result.first_failure_reason = string(baseline.first_failure_reason);
result.arducopter_alive = logical(baseline.process_after_live.is_alive);
result.log_path = string(fullfile(logs_dir, 'task_22_diagnose_gyro_loop_rate.txt'));
result.csv_path = string(fullfile(reports_dir, 'task_22_gyro_loop_rate_diagnostics.csv'));
result.mat_path = string(fullfile(reports_dir, 'task_22_gyro_loop_rate_diagnostics.mat'));

if baseline.baseline_restored
    fetch_result = uav.ardupilot.fetch_mavlink_params( ...
        cfg, ...
        "tcp:127.0.0.1:5763", ...
        'ParamNames', param_names, ...
        'Timeout_s', 25.0);

    arm_response = local_run_arm_response(repo_root, reports_dir);
    metrics = uav.ardupilot.summarize_live_backend_metrics(arm_response.live_result);
    gyro_rate_hz = local_extract_gyro_rate_hz(arm_response.arm_attempt.status_texts);
    sched_loop_rate_hz = local_param_value(fetch_result, "SCHED_LOOP_RATE");
    sim_rate_hz = local_param_value(fetch_result, "SIM_RATE_HZ");

    result.fetch = fetch_result;
    result.arm_response = arm_response;
    result.metrics = metrics;
    result.sched_loop_rate_hz = sched_loop_rate_hz;
    result.sim_rate_hz = sim_rate_hz;
    result.gyro_rate_hz = gyro_rate_hz;
    if isfinite(sched_loop_rate_hz)
        result.min_required_gyro_rate_hz = 1.8 * sched_loop_rate_hz;
    end
    if isfinite(gyro_rate_hz) && gyro_rate_hz > 0
        result.max_sched_loop_rate_hz = floor(gyro_rate_hz / 1.8);
    end
    result.first_failure_reason = local_pick_first_failure_reason(arm_response.arm_attempt);
    result.arducopter_alive = local_is_wsl_pid_alive(cfg.wsl_distro_name, baseline.process_after_live.pid);
else
    result.fetch.failure_reason = baseline.first_failure_reason;
end

diag_table = local_make_diag_table(result);
writetable(diag_table, char(result.csv_path));
save(char(result.mat_path), 'result');
uav.ardupilot.write_utf8_text_file(result.log_path, local_make_log_text(result));
assignin('base', 'task_22_gyro_loop_rate_diagnostics', result);

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);

fprintf('TASK-22: диагностика частоты гироскопа и главного цикла\n');
fprintf('  калибровка акселерометра подтверждена      : %s\n', local_bool_text(calibration.calibration_ready));
fprintf('  valid_rx_count                             : %d\n', local_metric(result.metrics, 'valid_rx_count'));
fprintf('  response_tx_count                          : %d\n', local_metric(result.metrics, 'response_tx_count'));
fprintf('  SCHED_LOOP_RATE [Hz]                       : %.0f\n', result.sched_loop_rate_hz);
fprintf('  фактическая частота гироскопа [Hz]         : %.0f\n', result.gyro_rate_hz);
fprintf('  требуемая минимальная частота [Hz]         : %.0f\n', result.min_required_gyro_rate_hz);
fprintf('  максимальный допустимый SCHED_LOOP_RATE    : %.0f\n', result.max_sched_loop_rate_hz);
fprintf('  причина отказа                             : %s\n', char(result.first_failure_reason));

function calibration = local_run_simple_calibration(repo_root, cfg)
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');

baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_22_diag_precal");
calibration = struct();
calibration.calibration_ready = false;
calibration.command_ack = nan;
calibration.status_text_tail = "";
calibration.failure_reason = string(baseline.first_failure_reason);
calibration.valid_rx_count = double(baseline.metrics.valid_rx_count);
calibration.response_tx_count = double(baseline.metrics.response_tx_count);

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

calibration.valid_rx_count = double(metrics.valid_rx_count);
calibration.response_tx_count = double(metrics.response_tx_count);
calibration.command_ack = double(helper_result.ack_result);
calibration.status_text_tail = string(helper_result.output_excerpt);
calibration.failure_reason = string(helper_result.failure_reason);
calibration.calibration_ready = helper_result.helper_completed && helper_result.ack_result == 0;

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
pause(2.0);

if calibration.calibration_ready
    post_baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_22_diag_postcal");
    calibration.valid_rx_count = double(post_baseline.metrics.valid_rx_count);
    calibration.response_tx_count = double(post_baseline.metrics.response_tx_count);
    calibration.failure_reason = string(post_baseline.first_failure_reason);
end

uav.ardupilot.write_utf8_text_file( ...
    fullfile(logs_dir, 'task_22_diagnose_gyro_loop_rate_calibration.txt'), ...
    local_make_calibration_text(calibration));
save(fullfile(reports_dir, 'task_22_diagnose_gyro_loop_rate_calibration.mat'), 'calibration');
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

function arm_response = local_run_arm_response(repo_root, reports_dir)
arm_mat_tmp = [tempname, '_task22_arm.mat'];
arm_csv_tmp = [tempname, '_task22_arm.csv'];
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

copyfile(arm_mat_tmp, fullfile(reports_dir, 'task_22_diagnose_gyro_loop_rate_arm.mat'), 'f');
copyfile(arm_csv_tmp, fullfile(reports_dir, 'task_22_diagnose_gyro_loop_rate_arm.csv'), 'f');
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

function value = local_param_value(fetch_result, param_name)
value = nan;
param_name = char(string(param_name));
if isfield(fetch_result, 'param_values') && isstruct(fetch_result.param_values) ...
        && isfield(fetch_result.param_values, param_name)
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
if strlength(strtrim(reason_text)) == 0 && isfinite(double(arm_attempt.ack_result))
    reason_text = "Команда взведения отклонена с кодом " + string(double(arm_attempt.ack_result)) + ".";
end
end

function diag_table = local_make_diag_table(result)
metrics = struct();
arm_attempt = struct();
if isfield(result, 'metrics') && isstruct(result.metrics)
    metrics = result.metrics;
end
if isfield(result, 'arm_response') && isstruct(result.arm_response) ...
        && isfield(result.arm_response, 'arm_attempt')
    arm_attempt = result.arm_response.arm_attempt;
end
diag_table = table( ...
    string(result.parm_path), ...
    logical(result.calibration.calibration_ready), ...
    double(result.calibration.command_ack), ...
    double(result.sched_loop_rate_hz), ...
    double(result.sim_rate_hz), ...
    double(result.gyro_rate_hz), ...
    double(result.min_required_gyro_rate_hz), ...
    double(result.max_sched_loop_rate_hz), ...
    local_metric(metrics, 'valid_rx_count'), ...
    local_metric(metrics, 'response_tx_count'), ...
    local_metric(metrics, 'json_tx_count'), ...
    local_metric(metrics, 'last_frame_count'), ...
    local_metric(metrics, 'valid_rx_rate_hz'), ...
    local_metric(metrics, 'response_tx_rate_hz'), ...
    local_metric(metrics, 'valid_rx_period_mean_s'), ...
    local_metric(metrics, 'valid_rx_period_p95_s'), ...
    local_metric(metrics, 'model_step_elapsed_mean_s'), ...
    logical(result.arducopter_alive), ...
    local_metric(arm_attempt, 'ack_result'), ...
    logical(local_metric(arm_attempt, 'arm_succeeded')), ...
    string(result.first_failure_reason), ...
    'VariableNames', { ...
        'parm_path', ...
        'accel_simple_calibration_ready', ...
        'accel_calibration_ack', ...
        'sched_loop_rate_hz', ...
        'sim_rate_hz', ...
        'gyro_rate_hz', ...
        'min_required_gyro_rate_hz', ...
        'max_sched_loop_rate_hz', ...
        'valid_rx_count', ...
        'response_tx_count', ...
        'json_tx_count', ...
        'last_frame_count', ...
        'valid_rx_rate_hz', ...
        'response_tx_rate_hz', ...
        'valid_rx_period_mean_s', ...
        'valid_rx_period_p95_s', ...
        'model_step_elapsed_mean_s', ...
        'arducopter_alive', ...
        'command_ack', ...
        'arm_succeeded', ...
        'failure_reason'});
end

function value = local_metric(source, field_name)
value = nan;
if isstruct(source) && isfield(source, field_name) && ~isempty(source.(field_name))
    raw_value = source.(field_name);
    if islogical(raw_value) && isscalar(raw_value)
        value = raw_value;
    else
        value = double(raw_value);
    end
end
end

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-22: диагностика частоты гироскопа и главного цикла";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль TASK-20: " + result.parm_path;
lines(end + 1, 1) = "Штатная простая калибровка выполнена: " + local_bool_text(result.calibration.calibration_ready);
lines(end + 1, 1) = "COMMAND_ACK калибровки: " + string(result.calibration.command_ack);
lines(end + 1, 1) = "Фактический SCHED_LOOP_RATE [Hz]: " + string(result.sched_loop_rate_hz);
lines(end + 1, 1) = "Фактический SIM_RATE_HZ [Hz]: " + string(result.sim_rate_hz);
lines(end + 1, 1) = "Фактическая частота гироскопа [Hz]: " + string(result.gyro_rate_hz);
lines(end + 1, 1) = "Требуемая минимальная частота [Hz]: " + string(result.min_required_gyro_rate_hz);
lines(end + 1, 1) = "Максимально допустимый SCHED_LOOP_RATE [Hz]: " + string(result.max_sched_loop_rate_hz);
lines(end + 1, 1) = "valid_rx_count: " + string(local_metric(result.metrics, 'valid_rx_count'));
lines(end + 1, 1) = "response_tx_count: " + string(local_metric(result.metrics, 'response_tx_count'));
lines(end + 1, 1) = "json_tx_count: " + string(local_metric(result.metrics, 'json_tx_count'));
lines(end + 1, 1) = "last_frame_count: " + string(local_metric(result.metrics, 'last_frame_count'));
lines(end + 1, 1) = "ArduPilot жив после прогона: " + local_bool_text(result.arducopter_alive);
lines(end + 1, 1) = "Первая причина отказа: " + string(result.first_failure_reason);
if isfield(result, 'arm_response') && isfield(result.arm_response, 'arm_attempt')
    status_texts = string(result.arm_response.arm_attempt.status_texts(:));
    if ~isempty(status_texts)
        lines(end + 1, 1) = "";
        lines(end + 1, 1) = "Последние STATUSTEXT:";
        for idx = max(1, numel(status_texts)-7):numel(status_texts)
            lines(end + 1, 1) = "  " + strtrim(status_texts(idx)); %#ok<AGROW>
        end
    end
end
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_make_calibration_text(calibration)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-22: предварительная простая калибровка акселерометра";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Калибровка подтверждена: " + local_bool_text(calibration.calibration_ready);
lines(end + 1, 1) = "COMMAND_ACK: " + string(calibration.command_ack);
lines(end + 1, 1) = "valid_rx_count: " + string(calibration.valid_rx_count);
lines(end + 1, 1) = "response_tx_count: " + string(calibration.response_tx_count);
lines(end + 1, 1) = "Причина: " + string(calibration.failure_reason);
lines(end + 1, 1) = "Хвост STATUSTEXT: " + string(calibration.status_text_tail);
text_value = strjoin(lines, newline) + newline;
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
