%% RUN_ARDUPILOT_JSON_ACCEL_CONVENTION_EXPERIMENTS Сравнить варианты accel_body для ArduPilot JSON.
% Назначение:
%   Для нескольких диагностических соглашений по imu.accel_body
%   восстанавливает устойчивый обмен TASK-15, выполняет попытку arm
%   и фиксирует устойчивость обмена, ACK, STATUSTEXT, диапазоны ШИМ,
%   команд частоты вращения винтов и диапазоны accel_body.
%
% Входы:
%   none
%
% Выходы:
%   task_19_accel_convention_experiments - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ускорение - м/с^2;
%   ШИМ - микросекунды;
%   команды винтов - рад/с
%
% Допущения:
%   Используется уже подготовленный ArduPilot в WSL и рабочий JSON-обмен TASK-15.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_19_accel_convention_experiments.txt');
csv_path = fullfile(reports_dir, 'task_19_accel_convention_experiments.csv');
mat_path = fullfile(reports_dir, 'task_19_accel_convention_experiments.mat');

cfg = uav.ardupilot.default_json_config();
host_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);
arm_delay_s = local_read_base_scalar('ardupilot_task19_arm_delay_s', 30.0);
live_duration_s = max(40.0, arm_delay_s + 10.0);

modes = [ ...
    "current"; ...
    "specific_force_frd"; ...
    "minus_specific_force_frd"; ...
    "linear_accel_body"; ...
    "zero_on_ground"];

case_results = repmat(local_empty_case_result(), numel(modes), 1);

for idx = 1:numel(modes)
    mode_name = modes(idx);
    override = struct( ...
        'json_accel_mode', mode_name, ...
        'json_prearm_hold_enabled', true, ...
        'json_prearm_pwm_threshold_us', 1005.0);

    prefix = "task_19_accel_" + replace(mode_name, "-", "_");
    baseline_diag_log = fullfile(logs_dir, char(prefix + "_baseline.txt"));
    baseline_wait_log = fullfile(logs_dir, char(prefix + "_wait.txt"));
    baseline_handshake_log = fullfile(logs_dir, char(prefix + "_handshake.txt"));
    baseline_live_log = fullfile(logs_dir, char(prefix + "_live_backend.txt"));

    baseline_mat_tmp = [tempname, '_task19_accel_baseline.mat'];
    baseline_csv_tmp = [tempname, '_task19_accel_baseline.csv'];
    arm_mat_tmp = [tempname, '_task19_accel_arm.mat'];
    arm_csv_tmp = [tempname, '_task19_accel_arm.csv'];
    cleanup_temp = onCleanup(@() local_cleanup_temp_files({ ...
        baseline_mat_tmp, baseline_csv_tmp, arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

    local_assign_base('ardupilot_json_cfg_override', override);
    local_assign_base('ardupilot_task15_baseline_diag_log_path', baseline_diag_log);
    local_assign_base('ardupilot_task15_baseline_wait_log_path', baseline_wait_log);
    local_assign_base('ardupilot_task15_baseline_handshake_log_path', baseline_handshake_log);
    local_assign_base('ardupilot_task15_baseline_live_log_path', baseline_live_log);
    local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
    local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
    local_assign_base('ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
    local_assign_base('ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
    local_assign_base('ardupilot_live_backend_duration_s', live_duration_s);
    local_assign_base('ardupilot_arm_attempt_delay_s', arm_delay_s);
    cleanup_base = onCleanup(@() evalin('base', [ ...
        'clear(' ...
        '''ardupilot_json_cfg_override'', ' ...
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

    case_result = local_empty_case_result();
    case_result.accel_mode = mode_name;
    case_result.baseline_valid_rx_count = double(baseline.metrics.valid_rx_count);
    case_result.baseline_response_tx_count = double(baseline.metrics.response_tx_count);
    case_result.baseline_json_tx_count = double(baseline.metrics.json_tx_count);
    case_result.baseline_last_frame_count = double(baseline.metrics.last_frame_count);
    case_result.baseline_restored = logical(baseline.baseline_restored);
    case_result.arducopter_alive_after_baseline = logical(baseline.process_after_live.is_alive);
    case_result.launch_command = string(baseline.launch_command);

    if baseline.baseline_restored
        run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
        response = evalin('base', 'ardupilot_arm_pwm_response');
        metrics = uav.ardupilot.summarize_live_backend_metrics(response.live_result);
        [accel_min_vec, accel_max_vec, accel_norm_min, accel_norm_max] = ...
            local_extract_accel_stats(response.live_result);

        case_result.valid_rx_count = double(metrics.valid_rx_count);
        case_result.json_tx_count = double(metrics.json_tx_count);
        case_result.response_tx_count = double(metrics.response_tx_count);
        case_result.last_frame_count = double(metrics.last_frame_count);
        case_result.valid_rx_rate_hz = double(metrics.valid_rx_rate_hz);
        case_result.response_tx_rate_hz = double(metrics.response_tx_rate_hz);
        case_result.mean_response_period_s = double(metrics.response_tx_period_mean_s);
        case_result.p95_response_period_s = double(metrics.response_tx_period_p95_s);
        case_result.ack_result = double(response.arm_attempt.ack_result);
        case_result.arm_succeeded = logical(response.arm_attempt.arm_succeeded);
        case_result.failure_reason = string(response.arm_attempt.failure_reason);
        case_result.status_texts = string(response.arm_attempt.status_texts(:));
        case_result.has_accels_inconsistent = any(contains(case_result.status_texts, "Accels inconsistent"));
        case_result.motor_pwm_after_min_us = local_min_finite(response.after.last_pwm_us);
        case_result.motor_pwm_after_max_us = local_max_finite(response.after.last_pwm_us);
        case_result.motor_cmd_after_min_radps = local_min_finite(response.after.last_motor_cmd_radps);
        case_result.motor_cmd_after_max_radps = local_max_finite(response.after.last_motor_cmd_radps);
        case_result.accel_x_min_mps2 = accel_min_vec(1);
        case_result.accel_y_min_mps2 = accel_min_vec(2);
        case_result.accel_z_min_mps2 = accel_min_vec(3);
        case_result.accel_x_max_mps2 = accel_max_vec(1);
        case_result.accel_y_max_mps2 = accel_max_vec(2);
        case_result.accel_z_max_mps2 = accel_max_vec(3);
        case_result.accel_norm_min_mps2 = accel_norm_min;
        case_result.accel_norm_max_mps2 = accel_norm_max;
        case_result.arducopter_alive_after_case = local_is_arducopter_alive(cfg.wsl_distro_name, baseline.process_after_launch.pid);

        if ~(case_result.valid_rx_count > 50 && case_result.response_tx_count > 50 && case_result.last_frame_count > 0)
            case_result.failure_reason = "Устойчивый обмен не подтвержден для текущего варианта accel_body.";
        end
    else
        case_result.failure_reason = string(baseline.first_failure_reason);
        case_result.status_texts = strings(0, 1);
    end

    case_results(idx) = case_result;
end

summary = struct();
summary.cases = case_results;
[summary.selected_mode, summary.selection_reason] = local_pick_selected_mode(case_results);
summary.arm_succeeded = any([case_results.arm_succeeded]);

save(mat_path, 'summary');
writetable(local_make_case_table(case_results), csv_path);
local_write_utf8_text(log_path, local_make_summary_log(summary, arm_delay_s));
assignin('base', 'task_19_accel_convention_experiments', summary);
local_stop_existing_sitl(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port);

fprintf('Опыты TASK-19 по соглашению accel_body\n');
fprintf('  выбранный вариант                     : %s\n', char(summary.selected_mode));
fprintf('  причина выбора                        : %s\n', char(summary.selection_reason));
fprintf('  успешное взведение                    : %s\n', local_bool_text(summary.arm_succeeded));

function case_result = local_empty_case_result()
%LOCAL_EMPTY_CASE_RESULT Построить пустую структуру одного опыта.

case_result = struct( ...
    'accel_mode', "", ...
    'baseline_restored', false, ...
    'baseline_valid_rx_count', 0, ...
    'baseline_response_tx_count', 0, ...
    'baseline_json_tx_count', 0, ...
    'baseline_last_frame_count', 0, ...
    'arducopter_alive_after_baseline', false, ...
    'launch_command', "", ...
    'valid_rx_count', 0, ...
    'json_tx_count', 0, ...
    'response_tx_count', 0, ...
    'last_frame_count', 0, ...
    'valid_rx_rate_hz', 0.0, ...
    'response_tx_rate_hz', 0.0, ...
    'mean_response_period_s', nan, ...
    'p95_response_period_s', nan, ...
    'ack_result', nan, ...
    'arm_succeeded', false, ...
    'failure_reason', "", ...
    'status_texts', strings(0, 1), ...
    'has_accels_inconsistent', false, ...
    'motor_pwm_after_min_us', nan, ...
    'motor_pwm_after_max_us', nan, ...
    'motor_cmd_after_min_radps', nan, ...
    'motor_cmd_after_max_radps', nan, ...
    'accel_x_min_mps2', nan, ...
    'accel_y_min_mps2', nan, ...
    'accel_z_min_mps2', nan, ...
    'accel_x_max_mps2', nan, ...
    'accel_y_max_mps2', nan, ...
    'accel_z_max_mps2', nan, ...
    'accel_norm_min_mps2', nan, ...
    'accel_norm_max_mps2', nan, ...
    'arducopter_alive_after_case', false);
end

function [selected_mode, selection_reason] = local_pick_selected_mode(case_results)
%LOCAL_PICK_SELECTED_MODE Выбрать лучший вариант accel_body.

selected_mode = "";
selection_reason = "";

stable_mask = arrayfun(@(item) item.valid_rx_count > 50 ...
    && item.response_tx_count > 50 ...
    && item.last_frame_count > 0 ...
    && item.arducopter_alive_after_case, case_results);

arm_mask = stable_mask & arrayfun(@(item) item.arm_succeeded ...
    && item.motor_pwm_after_max_us > 1000.0 ...
    && item.motor_cmd_after_max_radps > 0.0, case_results);
if any(arm_mask)
    winner = case_results(find(arm_mask, 1, 'first'));
    selected_mode = string(winner.accel_mode);
    selection_reason = "Выбран первый вариант, в котором arm подтвержден и появились ненулевые команды.";
    return;
end

no_accel_fault_mask = stable_mask & ~arrayfun(@(item) item.has_accels_inconsistent, case_results);
if any(no_accel_fault_mask)
    winner = case_results(find(no_accel_fault_mask, 1, 'first'));
    selected_mode = string(winner.accel_mode);
    selection_reason = "Arm не подтвержден, но выбран первый устойчивый вариант без сообщения Accels inconsistent.";
    return;
end

if any(stable_mask)
    winner = case_results(find(stable_mask, 1, 'first'));
    selected_mode = string(winner.accel_mode);
    selection_reason = "Arm не подтвержден; выбран первый устойчивый вариант обмена для дальнейшей диагностики.";
    return;
end

winner = case_results(1);
selected_mode = string(winner.accel_mode);
selection_reason = "Ни один вариант не дал устойчивого обмена; выбран первый опыт для отчета.";
end

function data_table = local_make_case_table(case_results)
%LOCAL_MAKE_CASE_TABLE Преобразовать массив результатов в таблицу CSV.

case_count = numel(case_results);

accel_mode = strings(case_count, 1);
baseline_restored = false(case_count, 1);
baseline_valid_rx_count = zeros(case_count, 1);
baseline_response_tx_count = zeros(case_count, 1);
baseline_json_tx_count = zeros(case_count, 1);
baseline_last_frame_count = zeros(case_count, 1);
valid_rx_count = zeros(case_count, 1);
json_tx_count = zeros(case_count, 1);
response_tx_count = zeros(case_count, 1);
last_frame_count = zeros(case_count, 1);
valid_rx_rate_hz = zeros(case_count, 1);
response_tx_rate_hz = zeros(case_count, 1);
mean_response_period_s = nan(case_count, 1);
p95_response_period_s = nan(case_count, 1);
ack_result = nan(case_count, 1);
arm_succeeded = false(case_count, 1);
failure_reason = strings(case_count, 1);
status_texts = strings(case_count, 1);
has_accels_inconsistent = false(case_count, 1);
motor_pwm_after_min_us = nan(case_count, 1);
motor_pwm_after_max_us = nan(case_count, 1);
motor_cmd_after_min_radps = nan(case_count, 1);
motor_cmd_after_max_radps = nan(case_count, 1);
accel_x_min_mps2 = nan(case_count, 1);
accel_y_min_mps2 = nan(case_count, 1);
accel_z_min_mps2 = nan(case_count, 1);
accel_x_max_mps2 = nan(case_count, 1);
accel_y_max_mps2 = nan(case_count, 1);
accel_z_max_mps2 = nan(case_count, 1);
accel_norm_min_mps2 = nan(case_count, 1);
accel_norm_max_mps2 = nan(case_count, 1);
arducopter_alive_after_case = false(case_count, 1);

for idx = 1:case_count
    item = case_results(idx);
    accel_mode(idx) = string(item.accel_mode);
    baseline_restored(idx) = logical(item.baseline_restored);
    baseline_valid_rx_count(idx) = double(item.baseline_valid_rx_count);
    baseline_response_tx_count(idx) = double(item.baseline_response_tx_count);
    baseline_json_tx_count(idx) = double(item.baseline_json_tx_count);
    baseline_last_frame_count(idx) = double(item.baseline_last_frame_count);
    valid_rx_count(idx) = double(item.valid_rx_count);
    json_tx_count(idx) = double(item.json_tx_count);
    response_tx_count(idx) = double(item.response_tx_count);
    last_frame_count(idx) = double(item.last_frame_count);
    valid_rx_rate_hz(idx) = double(item.valid_rx_rate_hz);
    response_tx_rate_hz(idx) = double(item.response_tx_rate_hz);
    mean_response_period_s(idx) = double(item.mean_response_period_s);
    p95_response_period_s(idx) = double(item.p95_response_period_s);
    ack_result(idx) = double(item.ack_result);
    arm_succeeded(idx) = logical(item.arm_succeeded);
    failure_reason(idx) = string(item.failure_reason);
    status_texts(idx) = strjoin(string(item.status_texts(:)).', " | ");
    has_accels_inconsistent(idx) = logical(item.has_accels_inconsistent);
    motor_pwm_after_min_us(idx) = double(item.motor_pwm_after_min_us);
    motor_pwm_after_max_us(idx) = double(item.motor_pwm_after_max_us);
    motor_cmd_after_min_radps(idx) = double(item.motor_cmd_after_min_radps);
    motor_cmd_after_max_radps(idx) = double(item.motor_cmd_after_max_radps);
    accel_x_min_mps2(idx) = double(item.accel_x_min_mps2);
    accel_y_min_mps2(idx) = double(item.accel_y_min_mps2);
    accel_z_min_mps2(idx) = double(item.accel_z_min_mps2);
    accel_x_max_mps2(idx) = double(item.accel_x_max_mps2);
    accel_y_max_mps2(idx) = double(item.accel_y_max_mps2);
    accel_z_max_mps2(idx) = double(item.accel_z_max_mps2);
    accel_norm_min_mps2(idx) = double(item.accel_norm_min_mps2);
    accel_norm_max_mps2(idx) = double(item.accel_norm_max_mps2);
    arducopter_alive_after_case(idx) = logical(item.arducopter_alive_after_case);
end

data_table = table( ...
    accel_mode, ...
    baseline_restored, ...
    baseline_valid_rx_count, ...
    baseline_response_tx_count, ...
    baseline_json_tx_count, ...
    baseline_last_frame_count, ...
    valid_rx_count, ...
    json_tx_count, ...
    response_tx_count, ...
    last_frame_count, ...
    valid_rx_rate_hz, ...
    response_tx_rate_hz, ...
    mean_response_period_s, ...
    p95_response_period_s, ...
    ack_result, ...
    arm_succeeded, ...
    failure_reason, ...
    status_texts, ...
    has_accels_inconsistent, ...
    motor_pwm_after_min_us, ...
    motor_pwm_after_max_us, ...
    motor_cmd_after_min_radps, ...
    motor_cmd_after_max_radps, ...
    accel_x_min_mps2, ...
    accel_y_min_mps2, ...
    accel_z_min_mps2, ...
    accel_x_max_mps2, ...
    accel_y_max_mps2, ...
    accel_z_max_mps2, ...
    accel_norm_min_mps2, ...
    accel_norm_max_mps2, ...
    arducopter_alive_after_case);
end

function text_value = local_make_summary_log(summary, arm_delay_s)
%LOCAL_MAKE_SUMMARY_LOG Сформировать текстовый журнал серии опытов.

lines = strings(0, 1);
lines(end + 1, 1) = "Опыты TASK-19 по соглашению accel_body";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Задержка перед arm [s]: " + sprintf('%.3f', arm_delay_s);
lines(end + 1, 1) = "Выбранный вариант: " + string(summary.selected_mode);
lines(end + 1, 1) = "Причина выбора: " + string(summary.selection_reason);
lines(end + 1, 1) = "";

for idx = 1:numel(summary.cases)
    item = summary.cases(idx);
    lines(end + 1, 1) = "Вариант: " + string(item.accel_mode);
    lines(end + 1, 1) = "  baseline restored: " + local_bool_text(item.baseline_restored);
    lines(end + 1, 1) = "  baseline valid_rx_count: " + string(item.baseline_valid_rx_count);
    lines(end + 1, 1) = "  baseline response_tx_count: " + string(item.baseline_response_tx_count);
    lines(end + 1, 1) = "  valid_rx_count: " + string(item.valid_rx_count);
    lines(end + 1, 1) = "  response_tx_count: " + string(item.response_tx_count);
    lines(end + 1, 1) = "  last_frame_count: " + string(item.last_frame_count);
    lines(end + 1, 1) = "  ACK: " + string(item.ack_result);
    lines(end + 1, 1) = "  arm succeeded: " + local_bool_text(item.arm_succeeded);
    lines(end + 1, 1) = "  has Accels inconsistent: " + local_bool_text(item.has_accels_inconsistent);
    lines(end + 1, 1) = "  motor_pwm_us range after arm: [" + sprintf('%.3f', item.motor_pwm_after_min_us) ...
        + ", " + sprintf('%.3f', item.motor_pwm_after_max_us) + "]";
    lines(end + 1, 1) = "  motor_cmd_radps range after arm: [" + sprintf('%.3f', item.motor_cmd_after_min_radps) ...
        + ", " + sprintf('%.3f', item.motor_cmd_after_max_radps) + "]";
    lines(end + 1, 1) = "  accel_norm range [m/s^2]: [" + sprintf('%.6f', item.accel_norm_min_mps2) ...
        + ", " + sprintf('%.6f', item.accel_norm_max_mps2) + "]";
    lines(end + 1, 1) = "  failure reason: " + string(item.failure_reason);
    if isempty(item.status_texts)
        lines(end + 1, 1) = "  STATUSTEXT: (нет)";
    else
        for st_idx = 1:numel(item.status_texts)
            lines(end + 1, 1) = "  STATUSTEXT: " + string(item.status_texts(st_idx));
        end
    end
    lines(end + 1, 1) = "";
end

text_value = strjoin(lines, newline) + newline;
end

function [accel_min_vec, accel_max_vec, accel_norm_min, accel_norm_max] = local_extract_accel_stats(live_result)
%LOCAL_EXTRACT_ACCEL_STATS Определить диапазоны accel_body по JSON-кадрам.

accel_rows = nan(numel(live_result.json_text), 3);
for idx = 1:numel(live_result.json_text)
    raw_text = strip(string(live_result.json_text(idx)));
    if strlength(raw_text) == 0
        continue;
    end
    try
        parsed = jsondecode(char(raw_text));
        if isfield(parsed, 'imu') && isfield(parsed.imu, 'accel_body')
            vec = double(parsed.imu.accel_body(:));
            if numel(vec) >= 3
                accel_rows(idx, :) = vec(1:3).';
            end
        end
    catch
    end
end

accel_min_vec = local_vector_column_extreme(accel_rows, @min);
accel_max_vec = local_vector_column_extreme(accel_rows, @max);
accel_norm = sqrt(sum(accel_rows.^2, 2));
accel_norm = accel_norm(isfinite(accel_norm));
if isempty(accel_norm)
    accel_norm_min = nan;
    accel_norm_max = nan;
else
    accel_norm_min = min(accel_norm);
    accel_norm_max = max(accel_norm);
end
end

function vec = local_vector_column_extreme(samples, op_handle)
%LOCAL_VECTOR_COLUMN_EXTREME Взять экстремум по каждому столбцу.

vec = nan(3, 1);
for col = 1:3
    column_values = samples(:, col);
    column_values = column_values(isfinite(column_values));
    if ~isempty(column_values)
        vec(col) = op_handle(column_values);
    end
end
end

function is_alive = local_is_arducopter_alive(distro_name, pid_value)
%LOCAL_IS_ARDUCOPTER_ALIVE Проверить, жив ли процесс arducopter в WSL.

is_alive = false;
if ~isfinite(pid_value) || pid_value <= 0
    return;
end

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "ps -p %d -o pid= 2>/dev/null || true"', ...
    char(distro_name), round(pid_value));
[~, output_text] = system(command_text);
is_alive = strlength(strtrim(string(output_text))) > 0;
end

function value = local_min_finite(samples)
%LOCAL_MIN_FINITE Минимум по конечным значениям.

samples = double(samples(:));
samples = samples(isfinite(samples));
if isempty(samples)
    value = nan;
else
    value = min(samples);
end
end

function value = local_max_finite(samples)
%LOCAL_MAX_FINITE Максимум по конечным значениям.

samples = double(samples(:));
samples = samples(isfinite(samples));
if isempty(samples)
    value = nan;
else
    value = max(samples);
end
end

function value = local_read_base_scalar(var_name, default_value)
%LOCAL_READ_BASE_SCALAR Прочитать числовое значение из base workspace.

value = double(default_value);
if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    candidate = evalin('base', var_name);
    validateattributes(candidate, {'numeric'}, ...
        {'real', 'scalar', 'finite', 'nonnegative'}, ...
        mfilename, var_name);
    value = double(candidate);
end
end

function local_assign_base(var_name, value)
%LOCAL_ASSIGN_BASE Поместить переменную в base workspace.

assignin('base', var_name, value);
end

function local_cleanup_temp_files(path_list)
%LOCAL_CLEANUP_TEMP_FILES Удалить временные файлы.

for idx = 1:numel(path_list)
    local_delete_if_exists(path_list{idx});
end
end

function local_delete_if_exists(path_value)
%LOCAL_DELETE_IF_EXISTS Удалить файл при наличии.

if exist(path_value, 'file') == 2
    delete(path_value);
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

folder_path = fileparts(char(path_value));
if strlength(string(folder_path)) > 0 && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(char(path_value), 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task19:accelexp:OpenFile', ...
        'Не удалось открыть файл %s.', string(path_value));
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function local_stop_existing_sitl(distro_name, host_ip, mavlink_port)
%LOCAL_STOP_EXISTING_SITL Остановить экземпляр arducopter текущего стенда.

pattern = sprintf( ...
    'build/sitl/bin/arducopter.*JSON:%s.*udpclient:%s:%d', ...
    char(host_ip), ...
    char(host_ip), ...
    mavlink_port);
system(sprintf( ...
    'wsl -d %s -- bash -lc "pkill -f ''%s'' >/dev/null 2>&1 || true"', ...
    char(distro_name), pattern)); %#ok<NASGU>
pause(1.0);
end

function host_ip = local_resolve_windows_host_ip(distro_name, fallback_ip)
%LOCAL_RESOLVE_WINDOWS_HOST_IP Определить адрес Windows-хоста для WSL2.

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

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русский текст.

if logical(flag_value)
    text_value = 'да';
else
    text_value = 'нет';
end
end
