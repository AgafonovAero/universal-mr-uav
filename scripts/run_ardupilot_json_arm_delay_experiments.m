%% RUN_ARDUPILOT_JSON_ARM_DELAY_EXPERIMENTS Проверить влияние задержки перед arm.
% Назначение:
%   После восстановления устойчивого обмена TASK-15 выполняет серию попыток
%   arm с задержками 5, 15, 30 и 60 секунд для выбранного варианта accel_body.
%
% Входы:
%   none
%
% Выходы:
%   task_19_arm_delay_experiments - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ШИМ - микросекунды;
%   команды винтов - рад/с
%
% Допущения:
%   Используется уже работоспособный JSON-обмен TASK-15 и подготовленный ArduPilot.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_19_arm_delay_experiments.txt');
csv_path = fullfile(reports_dir, 'task_19_arm_delay_experiments.csv');
mat_path = fullfile(reports_dir, 'task_19_arm_delay_experiments.mat');

cfg = uav.ardupilot.default_json_config();
[selected_mode, selection_source] = local_pick_selected_mode(repo_root, cfg);
delays_s = [5.0; 15.0; 30.0; 60.0];
host_ip = local_resolve_windows_host_ip(cfg.wsl_distro_name, cfg.udp_remote_ip);

case_results = repmat(local_empty_case_result(), numel(delays_s), 1);

for idx = 1:numel(delays_s)
    delay_s = delays_s(idx);
    override = struct( ...
        'json_accel_mode', selected_mode, ...
        'json_prearm_hold_enabled', true, ...
        'json_prearm_pwm_threshold_us', 1005.0);

    prefix = sprintf('task_19_arm_delay_%02ds', round(delay_s));
    baseline_diag_log = fullfile(logs_dir, [prefix, '_baseline.txt']);
    baseline_wait_log = fullfile(logs_dir, [prefix, '_wait.txt']);
    baseline_handshake_log = fullfile(logs_dir, [prefix, '_handshake.txt']);
    baseline_live_log = fullfile(logs_dir, [prefix, '_live_backend.txt']);
    baseline_mat_tmp = [tempname, '_task19_delay_baseline.mat'];
    baseline_csv_tmp = [tempname, '_task19_delay_baseline.csv'];
    arm_mat_tmp = [tempname, '_task19_delay_arm.mat'];
    arm_csv_tmp = [tempname, '_task19_delay_arm.csv'];
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
    local_assign_base('ardupilot_live_backend_duration_s', max(40.0, delay_s + 10.0));
    local_assign_base('ardupilot_arm_attempt_delay_s', delay_s);
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
    case_result.delay_s = delay_s;
    case_result.accel_mode = selected_mode;
    case_result.selection_source = selection_source;
    case_result.baseline_restored = logical(baseline.baseline_restored);
    case_result.baseline_valid_rx_count = double(baseline.metrics.valid_rx_count);
    case_result.baseline_response_tx_count = double(baseline.metrics.response_tx_count);
    case_result.baseline_last_frame_count = double(baseline.metrics.last_frame_count);
    case_result.arducopter_alive_after_baseline = logical(baseline.process_after_live.is_alive);

    if baseline.baseline_restored
        run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
        response = evalin('base', 'ardupilot_arm_pwm_response');
        metrics = uav.ardupilot.summarize_live_backend_metrics(response.live_result);

        case_result.valid_rx_count = double(metrics.valid_rx_count);
        case_result.json_tx_count = double(metrics.json_tx_count);
        case_result.response_tx_count = double(metrics.response_tx_count);
        case_result.last_frame_count = double(metrics.last_frame_count);
        case_result.valid_rx_rate_hz = double(metrics.valid_rx_rate_hz);
        case_result.response_tx_rate_hz = double(metrics.response_tx_rate_hz);
        case_result.mean_response_period_s = double(metrics.response_tx_period_mean_s);
        case_result.p95_response_period_s = double(metrics.response_tx_period_p95_s);
        case_result.arm_succeeded = logical(response.arm_attempt.arm_succeeded);
        case_result.ack_result = double(response.arm_attempt.ack_result);
        case_result.failure_reason = string(response.arm_attempt.failure_reason);
        case_result.status_texts = string(response.arm_attempt.status_texts(:));
        case_result.motor_pwm_after_min_us = local_min_finite(response.after.last_pwm_us);
        case_result.motor_pwm_after_max_us = local_max_finite(response.after.last_pwm_us);
        case_result.motor_cmd_after_min_radps = local_min_finite(response.after.last_motor_cmd_radps);
        case_result.motor_cmd_after_max_radps = local_max_finite(response.after.last_motor_cmd_radps);
        case_result.arducopter_alive_after_case = local_is_arducopter_alive(cfg.wsl_distro_name, baseline.process_after_launch.pid);
    else
        case_result.failure_reason = string(baseline.first_failure_reason);
    end

    case_results(idx) = case_result;
end

summary = struct();
summary.accel_mode = selected_mode;
summary.selection_source = selection_source;
summary.cases = case_results;
summary.arm_succeeded = any([case_results.arm_succeeded]);
summary.best_delay_s = local_pick_best_delay(case_results);

save(mat_path, 'summary');
writetable(local_make_case_table(case_results), csv_path);
local_write_utf8_text(log_path, local_make_summary_log(summary));
assignin('base', 'task_19_arm_delay_experiments', summary);
local_stop_existing_sitl(cfg.wsl_distro_name, host_ip, cfg.mavlink_udp_port);

fprintf('Опыты TASK-19 по задержке перед arm\n');
fprintf('  accel_mode                             : %s\n', char(selected_mode));
fprintf('  best_delay_s                           : %.0f\n', summary.best_delay_s);
fprintf('  arm succeeded                          : %s\n', local_bool_text(summary.arm_succeeded));

function case_result = local_empty_case_result()
%LOCAL_EMPTY_CASE_RESULT Построить пустую структуру одного опыта.

case_result = struct( ...
    'delay_s', 0.0, ...
    'accel_mode', "", ...
    'selection_source', "", ...
    'baseline_restored', false, ...
    'baseline_valid_rx_count', 0, ...
    'baseline_response_tx_count', 0, ...
    'baseline_last_frame_count', 0, ...
    'arducopter_alive_after_baseline', false, ...
    'valid_rx_count', 0, ...
    'json_tx_count', 0, ...
    'response_tx_count', 0, ...
    'last_frame_count', 0, ...
    'valid_rx_rate_hz', 0.0, ...
    'response_tx_rate_hz', 0.0, ...
    'mean_response_period_s', nan, ...
    'p95_response_period_s', nan, ...
    'arm_succeeded', false, ...
    'ack_result', nan, ...
    'failure_reason', "", ...
    'status_texts', strings(0, 1), ...
    'motor_pwm_after_min_us', nan, ...
    'motor_pwm_after_max_us', nan, ...
    'motor_cmd_after_min_radps', nan, ...
    'motor_cmd_after_max_radps', nan, ...
    'arducopter_alive_after_case', false);
end

function [selected_mode, selection_source] = local_pick_selected_mode(repo_root, cfg)
%LOCAL_PICK_SELECTED_MODE Выбрать accel_mode из предыдущей серии опытов.

selected_mode = string(cfg.json_accel_mode);
selection_source = "cfg.json_accel_mode";

mat_path = fullfile(repo_root, 'artifacts', 'reports', 'task_19_accel_convention_experiments.mat');
if isfile(mat_path)
    loaded = load(mat_path);
    if isfield(loaded, 'summary') && isfield(loaded.summary, 'selected_mode')
        selected_mode = string(loaded.summary.selected_mode);
        selection_source = "task_19_accel_convention_experiments.mat";
    end
end
end

function best_delay_s = local_pick_best_delay(case_results)
%LOCAL_PICK_BEST_DELAY Выбрать лучшую задержку по фактическому результату.

best_delay_s = nan;
success_mask = arrayfun(@(item) item.arm_succeeded ...
    && item.motor_pwm_after_max_us > 1000.0 ...
    && item.motor_cmd_after_max_radps > 0.0, case_results);
if any(success_mask)
    best_delay_s = case_results(find(success_mask, 1, 'first')).delay_s;
    return;
end

stable_mask = arrayfun(@(item) item.valid_rx_count > 50 ...
    && item.response_tx_count > 50 ...
    && item.last_frame_count > 0, case_results);
if any(stable_mask)
    best_delay_s = case_results(find(stable_mask, 1, 'first')).delay_s;
end
end

function data_table = local_make_case_table(case_results)
%LOCAL_MAKE_CASE_TABLE Построить CSV-таблицу результатов по задержкам.

case_count = numel(case_results);

delay_s = zeros(case_count, 1);
accel_mode = strings(case_count, 1);
selection_source = strings(case_count, 1);
baseline_restored = false(case_count, 1);
baseline_valid_rx_count = zeros(case_count, 1);
baseline_response_tx_count = zeros(case_count, 1);
baseline_last_frame_count = zeros(case_count, 1);
valid_rx_count = zeros(case_count, 1);
json_tx_count = zeros(case_count, 1);
response_tx_count = zeros(case_count, 1);
last_frame_count = zeros(case_count, 1);
valid_rx_rate_hz = zeros(case_count, 1);
response_tx_rate_hz = zeros(case_count, 1);
mean_response_period_s = nan(case_count, 1);
p95_response_period_s = nan(case_count, 1);
arm_succeeded = false(case_count, 1);
ack_result = nan(case_count, 1);
failure_reason = strings(case_count, 1);
status_texts = strings(case_count, 1);
motor_pwm_after_min_us = nan(case_count, 1);
motor_pwm_after_max_us = nan(case_count, 1);
motor_cmd_after_min_radps = nan(case_count, 1);
motor_cmd_after_max_radps = nan(case_count, 1);
arducopter_alive_after_case = false(case_count, 1);

for idx = 1:case_count
    item = case_results(idx);
    delay_s(idx) = double(item.delay_s);
    accel_mode(idx) = string(item.accel_mode);
    selection_source(idx) = string(item.selection_source);
    baseline_restored(idx) = logical(item.baseline_restored);
    baseline_valid_rx_count(idx) = double(item.baseline_valid_rx_count);
    baseline_response_tx_count(idx) = double(item.baseline_response_tx_count);
    baseline_last_frame_count(idx) = double(item.baseline_last_frame_count);
    valid_rx_count(idx) = double(item.valid_rx_count);
    json_tx_count(idx) = double(item.json_tx_count);
    response_tx_count(idx) = double(item.response_tx_count);
    last_frame_count(idx) = double(item.last_frame_count);
    valid_rx_rate_hz(idx) = double(item.valid_rx_rate_hz);
    response_tx_rate_hz(idx) = double(item.response_tx_rate_hz);
    mean_response_period_s(idx) = double(item.mean_response_period_s);
    p95_response_period_s(idx) = double(item.p95_response_period_s);
    arm_succeeded(idx) = logical(item.arm_succeeded);
    ack_result(idx) = double(item.ack_result);
    failure_reason(idx) = string(item.failure_reason);
    status_texts(idx) = strjoin(string(item.status_texts(:)).', " | ");
    motor_pwm_after_min_us(idx) = double(item.motor_pwm_after_min_us);
    motor_pwm_after_max_us(idx) = double(item.motor_pwm_after_max_us);
    motor_cmd_after_min_radps(idx) = double(item.motor_cmd_after_min_radps);
    motor_cmd_after_max_radps(idx) = double(item.motor_cmd_after_max_radps);
    arducopter_alive_after_case(idx) = logical(item.arducopter_alive_after_case);
end

data_table = table( ...
    delay_s, ...
    accel_mode, ...
    selection_source, ...
    baseline_restored, ...
    baseline_valid_rx_count, ...
    baseline_response_tx_count, ...
    baseline_last_frame_count, ...
    valid_rx_count, ...
    json_tx_count, ...
    response_tx_count, ...
    last_frame_count, ...
    valid_rx_rate_hz, ...
    response_tx_rate_hz, ...
    mean_response_period_s, ...
    p95_response_period_s, ...
    arm_succeeded, ...
    ack_result, ...
    failure_reason, ...
    status_texts, ...
    motor_pwm_after_min_us, ...
    motor_pwm_after_max_us, ...
    motor_cmd_after_min_radps, ...
    motor_cmd_after_max_radps, ...
    arducopter_alive_after_case);
end

function text_value = local_make_summary_log(summary)
%LOCAL_MAKE_SUMMARY_LOG Сформировать текстовый журнал опытов по задержке.

lines = strings(0, 1);
lines(end + 1, 1) = "Опыты TASK-19 по задержке перед arm";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "accel_mode: " + string(summary.accel_mode);
lines(end + 1, 1) = "Источник выбора accel_mode: " + string(summary.selection_source);
lines(end + 1, 1) = "Лучшее время задержки [s]: " + string(summary.best_delay_s);
lines(end + 1, 1) = "";

for idx = 1:numel(summary.cases)
    item = summary.cases(idx);
    lines(end + 1, 1) = "Задержка [s]: " + sprintf('%.0f', item.delay_s);
    lines(end + 1, 1) = "  baseline restored: " + local_bool_text(item.baseline_restored);
    lines(end + 1, 1) = "  valid_rx_count: " + string(item.valid_rx_count);
    lines(end + 1, 1) = "  response_tx_count: " + string(item.response_tx_count);
    lines(end + 1, 1) = "  last_frame_count: " + string(item.last_frame_count);
    lines(end + 1, 1) = "  ACK: " + string(item.ack_result);
    lines(end + 1, 1) = "  arm succeeded: " + local_bool_text(item.arm_succeeded);
    lines(end + 1, 1) = "  motor_pwm_us after arm: [" + sprintf('%.3f', item.motor_pwm_after_min_us) ...
        + ", " + sprintf('%.3f', item.motor_pwm_after_max_us) + "]";
    lines(end + 1, 1) = "  motor_cmd_radps after arm: [" + sprintf('%.3f', item.motor_cmd_after_min_radps) ...
        + ", " + sprintf('%.3f', item.motor_cmd_after_max_radps) + "]";
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
    error('uav:task19:armdelay:OpenFile', ...
        'Не удалось открыть файл %s.', string(path_value));
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русский текст.

if logical(flag_value)
    text_value = 'да';
else
    text_value = 'нет';
end
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
