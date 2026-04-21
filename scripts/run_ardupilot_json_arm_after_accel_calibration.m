%% RUN_ARDUPILOT_JSON_ARM_AFTER_ACCEL_CALIBRATION Проверить arm после калибровки акселерометра.
% Назначение:
%   Повторяет практический прогон `ArduPilot JSON + MATLAB-модель` после
%   штатной калибровки акселерометра. Вариант A использует сохраненное
%   состояние после простой калибровки, вариант B - параметрический
%   профиль `task21_single_accel_calibrated.parm`, если он был создан.
%
% Входы:
%   none
%
% Выходы:
%   task_21_arm_after_accel_calibration - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   команды частоты вращения - рад/с.
%
% Допущения:
%   `run_ardupilot_json_accel_simple_calibration.m` уже выполнялся хотя бы
%   один раз и при необходимости сохранил калибровку в рабочем каталоге SITL.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

log_path = fullfile(logs_dir, 'task_21_arm_after_accel_calibration.txt');
csv_path = fullfile(reports_dir, 'task_21_arm_after_accel_calibration.csv');
mat_path = fullfile(reports_dir, 'task_21_arm_after_accel_calibration.mat');
base_parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');
calibrated_parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task21_single_accel_calibrated.parm');

if ~isfile(base_parm_path)
    error('uav:task21:armAfterCal:MissingBaseParm', ...
        'Не найден профиль TASK-20: %s', base_parm_path);
end

variants = [ ...
    struct('variant_name', "after_simple_calibration", 'parm_path', string(base_parm_path), 'profile_applied', false)];
if isfile(calibrated_parm_path)
    variants(end + 1) = struct('variant_name', "after_param_profile", 'parm_path', string(calibrated_parm_path), 'profile_applied', true); %#ok<AGROW>
end

rows = repmat(local_empty_row(), numel(variants), 1);
for idx = 1:numel(variants)
    rows(idx) = local_run_variant(repo_root, logs_dir, reports_dir, variants(idx));
    if rows(idx).arm_succeeded
        break;
    end
end

result = struct();
result.rows = rows;
result.table = struct2table(rows);
result.arm_succeeded = any([rows.arm_succeeded]);
result.selected_variant = "";
if result.arm_succeeded
    result.selected_variant = rows(find([rows.arm_succeeded], 1, 'first')).variant_name;
end

writetable(result.table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_21_arm_after_accel_calibration', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('TASK-21: проверка arm после калибровки акселерометра\n');
for idx = 1:numel(rows)
    fprintf('  %-28s arm=%s ack=%.0f valid_rx=%d response_tx=%d reason=%s\n', ...
        char(rows(idx).variant_name), ...
        local_bool_text(rows(idx).arm_succeeded), ...
        rows(idx).command_ack, ...
        rows(idx).valid_rx_count, ...
        rows(idx).response_tx_count, ...
        char(rows(idx).failure_reason));
end

function row = local_run_variant(repo_root, logs_dir, reports_dir, variant)
cfg = uav.ardupilot.default_json_config();
variant_char = char(variant.variant_name);
baseline_mat_tmp = [tempname, '_', variant_char, '_baseline.mat'];
baseline_csv_tmp = [tempname, '_', variant_char, '_baseline.csv'];
arm_mat_tmp = [tempname, '_', variant_char, '_arm.mat'];
arm_csv_tmp = [tempname, '_', variant_char, '_arm.csv'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp, arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

local_assign_base('ardupilot_task15_extra_defaults_win_path', char(variant.parm_path));
local_assign_base('ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, ['task_21_', variant_char, '_baseline.txt']));
local_assign_base('ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, ['task_21_', variant_char, '_wait.txt']));
local_assign_base('ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, ['task_21_', variant_char, '_handshake.txt']));
local_assign_base('ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, ['task_21_', variant_char, '_live.txt']));
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
local_assign_base('ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
local_assign_base('ardupilot_live_backend_duration_s', 40.0);
local_assign_base('ardupilot_arm_attempt_delay_s', 10.0);
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

row = local_empty_row();
row.variant_name = variant.variant_name;
row.parm_path = variant.parm_path;
row.profile_applied = variant.profile_applied;
row.valid_rx_count = double(baseline.metrics.valid_rx_count);
row.response_tx_count = double(baseline.metrics.response_tx_count);
row.last_frame_count = double(baseline.metrics.last_frame_count);
row.failure_reason = string(baseline.first_failure_reason);

if baseline.baseline_restored
    run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
    response = evalin('base', 'ardupilot_arm_pwm_response');
    row.valid_rx_count = double(response.live_result.valid_rx_count);
    row.response_tx_count = double(response.live_result.response_tx_count);
    row.last_frame_count = double(response.live_result.last_frame_count);
    row.command_ack = double(response.arm_attempt.ack_result);
    row.arm_succeeded = logical(response.arm_attempt.arm_succeeded);
    row.failure_reason = string(response.arm_attempt.failure_reason);
    finite_pwm = double(response.live_result.last_pwm_us(:));
    finite_pwm = finite_pwm(isfinite(finite_pwm));
    if ~isempty(finite_pwm)
        row.motor_pwm_min_us = min(finite_pwm);
        row.motor_pwm_max_us = max(finite_pwm);
    end
    finite_motor = double(response.live_result.last_motor_cmd_radps(:));
    finite_motor = finite_motor(isfinite(finite_motor));
    if ~isempty(finite_motor)
        row.motor_cmd_min_radps = min(finite_motor);
        row.motor_cmd_max_radps = max(finite_motor);
    end
    status_texts = string(response.arm_attempt.status_texts(:));
    if isempty(status_texts)
        row.status_text_tail = "";
    else
        row.status_text_tail = strjoin(status_texts(max(1, numel(status_texts)-4):end), " | ");
    end
else
    row.command_ack = nan;
    row.arm_succeeded = false;
    row.motor_pwm_min_us = nan;
    row.motor_pwm_max_us = nan;
    row.motor_cmd_min_radps = nan;
    row.motor_cmd_max_radps = nan;
    row.status_text_tail = "";
end

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);
end

function row = local_empty_row()
row = struct( ...
    'variant_name', "", ...
    'parm_path', "", ...
    'profile_applied', false, ...
    'valid_rx_count', 0, ...
    'response_tx_count', 0, ...
    'last_frame_count', 0, ...
    'command_ack', nan, ...
    'arm_succeeded', false, ...
    'failure_reason', "", ...
    'status_text_tail', "", ...
    'motor_pwm_min_us', nan, ...
    'motor_pwm_max_us', nan, ...
    'motor_cmd_min_radps', nan, ...
    'motor_cmd_max_radps', nan);
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

function text_value = local_make_log_text(result)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-21: повторная попытка arm после калибровки акселерометра";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "arm выполнен: " + local_bool_text(result.arm_succeeded);
lines(end + 1, 1) = "Выбранный вариант: " + local_empty_as_none(result.selected_variant);
for idx = 1:height(result.table)
    row = result.table(idx, :);
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "[" + row.variant_name + "]";
    lines(end + 1, 1) = "Профиль: " + row.parm_path;
    lines(end + 1, 1) = "valid_rx_count: " + string(row.valid_rx_count);
    lines(end + 1, 1) = "response_tx_count: " + string(row.response_tx_count);
    lines(end + 1, 1) = "last_frame_count: " + string(row.last_frame_count);
    lines(end + 1, 1) = "arm: " + local_bool_text(row.arm_succeeded);
    lines(end + 1, 1) = "COMMAND_ACK: " + string(row.command_ack);
    lines(end + 1, 1) = "Причина: " + string(row.failure_reason);
    lines(end + 1, 1) = "STATUSTEXT: " + string(row.status_text_tail);
    lines(end + 1, 1) = sprintf('motor_pwm_us range: [%.6f %.6f]', row.motor_pwm_min_us, row.motor_pwm_max_us);
    lines(end + 1, 1) = sprintf('motor_cmd_radps range: [%.6f %.6f]', row.motor_cmd_min_radps, row.motor_cmd_max_radps);
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

function text_value = local_empty_as_none(value)
value = string(value);
if strlength(value) == 0
    text_value = "нет";
else
    text_value = value;
end
end
