%% DIAGNOSE_ARDUPILOT_JSON_IMU_CONSISTENCY Диагностика согласованности ИНС для ArduPilot JSON.
% Назначение:
%   Поднимает рабочий стенд ArduPilot JSON/UDP на базе устойчивого обмена TASK-15,
%   сохраняет первые 200 реально отправленных JSON-кадров, выполняет попытку
%   взведения и формирует инженерную сводку по инерциальным данным, кватерниону,
%   статусным сообщениям и ACK.
%
% Входы:
%   none
%
% Выходы:
%   task_19_json_imu_consistency - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ускорение - м/с^2;
%   угловая скорость - рад/с
%
% Допущения:
%   Внешняя среда ArduPilot в WSL уже подготовлена по TASK-15/TASK-18.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_19_json_imu_consistency.txt');
jsonl_path = fullfile(reports_dir, 'task_19_first_json_frames.jsonl');
csv_path = fullfile(reports_dir, 'task_19_json_imu_consistency.csv');
mat_path = fullfile(reports_dir, 'task_19_json_imu_consistency.mat');

baseline_diag_log = fullfile(logs_dir, 'task_19_json_imu_consistency_baseline.txt');
baseline_wait_log = fullfile(logs_dir, 'task_19_json_imu_consistency_wait.txt');
baseline_handshake_log = fullfile(logs_dir, 'task_19_json_imu_consistency_handshake.txt');
baseline_live_log = fullfile(logs_dir, 'task_19_json_imu_consistency_live_backend.txt');

baseline_mat_tmp = [tempname, '_task19_baseline.mat'];
baseline_csv_tmp = [tempname, '_task19_baseline.csv'];
arm_mat_tmp = [tempname, '_task19_arm.mat'];
arm_csv_tmp = [tempname, '_task19_arm.csv'];

cleanup_temp = onCleanup(@() local_cleanup_temp_files({ ...
    baseline_mat_tmp, baseline_csv_tmp, arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
cfg_override = local_make_cfg_override(cfg);

local_assign_base('ardupilot_json_cfg_override', cfg_override);
local_assign_base('ardupilot_task15_baseline_diag_log_path', baseline_diag_log);
local_assign_base('ardupilot_task15_baseline_wait_log_path', baseline_wait_log);
local_assign_base('ardupilot_task15_baseline_handshake_log_path', baseline_handshake_log);
local_assign_base('ardupilot_task15_baseline_live_log_path', baseline_live_log);
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
local_assign_base('ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
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

if ~baseline.baseline_restored
    result = struct();
    result.cfg_override = cfg_override;
    result.baseline = baseline;
    result.baseline_restored = false;
    result.failure_reason = string(baseline.first_failure_reason);
    save(mat_path, 'result');
    writetable(table(string(baseline.first_failure_reason), ...
        'VariableNames', {'failure_reason'}), csv_path);
    local_write_utf8_text(jsonl_path, "");
    local_write_utf8_text(log_path, ...
        "Диагностика TASK-19 не выполнена: базовый JSON-обмен TASK-15 не восстановлен." + newline ...
        + "Причина: " + string(baseline.first_failure_reason) + newline);
    assignin('base', 'task_19_json_imu_consistency', result);
    error('uav:task19:imuconsistency:BaselineFailed', ...
        'Базовый обмен TASK-15 не восстановлен: %s', ...
        char(string(baseline.first_failure_reason)));
end

arm_delay_s = local_read_base_scalar('ardupilot_task19_arm_delay_s', 30.0);
live_duration_s = max(40.0, arm_delay_s + 10.0);
local_assign_base('ardupilot_live_backend_duration_s', live_duration_s);
local_assign_base('ardupilot_arm_attempt_delay_s', arm_delay_s);

run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
response = evalin('base', 'ardupilot_arm_pwm_response');
metrics = uav.ardupilot.summarize_live_backend_metrics(response.live_result);

[frame_table, jsonl_text, frame_summary] = local_extract_first_frames( ...
    response.live_result, params, cfg_override, 200);
writetable(frame_table, csv_path);
local_write_utf8_text(jsonl_path, jsonl_text);

result = struct();
result.cfg_override = cfg_override;
result.baseline = baseline;
result.baseline_restored = true;
result.metrics = metrics;
result.arm_attempt = response.arm_attempt;
result.before = response.before;
result.after = response.after;
result.live_result = response.live_result;
result.frame_table = frame_table;
result.frame_summary = frame_summary;
save(mat_path, 'result');

local_write_utf8_text(log_path, local_make_log_text(result, arm_delay_s, jsonl_path, csv_path));
assignin('base', 'task_19_json_imu_consistency', result);
local_stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);

fprintf('Диагностика JSON-ИНС TASK-19\n');
fprintf('  baseline restored                      : %s\n', local_bool_text(baseline.baseline_restored));
fprintf('  valid_rx_count                         : %d\n', metrics.valid_rx_count);
fprintf('  response_tx_count                      : %d\n', metrics.response_tx_count);
fprintf('  ACK                                    : %.0f\n', response.arm_attempt.ack_result);
fprintf('  arm succeeded                          : %s\n', local_bool_text(response.arm_attempt.arm_succeeded));
fprintf('  failure reason                         : %s\n', char(string(response.arm_attempt.failure_reason)));
fprintf('  first 200 JSON frames saved            : %d\n', height(frame_table));

function cfg_override = local_make_cfg_override(cfg)
%LOCAL_MAKE_CFG_OVERRIDE Сформировать конфигурацию диагностики JSON-ИНС.

cfg_override = struct( ...
    'json_accel_mode', string(cfg.json_accel_mode), ...
    'json_prearm_hold_enabled', true, ...
    'json_prearm_pwm_threshold_us', 1005.0);

if evalin('base', 'exist(''ardupilot_json_cfg_override'', ''var'')')
    candidate = evalin('base', 'ardupilot_json_cfg_override');
    if isstruct(candidate) && isscalar(candidate)
        cfg_override = local_overlay_struct(cfg_override, candidate);
    end
end
end

function [frame_table, jsonl_text, summary] = local_extract_first_frames(live_result, params, cfg, sample_limit)
%LOCAL_EXTRACT_FIRST_FRAMES Сформировать таблицу и JSONL первых переданных кадров.

sample_limit = min(sample_limit, numel(live_result.json_text));

frame_index = nan(sample_limit, 1);
time_s = nan(sample_limit, 1);
json_timestamp_s = nan(sample_limit, 1);
frame_count = nan(sample_limit, 1);
exchange_status = strings(sample_limit, 1);
prearm_hold_active = false(sample_limit, 1);
required_fields_present = false(sample_limit, 1);
parse_ok = false(sample_limit, 1);

gyro_x = nan(sample_limit, 1);
gyro_y = nan(sample_limit, 1);
gyro_z = nan(sample_limit, 1);
accel_x = nan(sample_limit, 1);
accel_y = nan(sample_limit, 1);
accel_z = nan(sample_limit, 1);
position_n = nan(sample_limit, 1);
position_e = nan(sample_limit, 1);
position_d = nan(sample_limit, 1);
velocity_n = nan(sample_limit, 1);
velocity_e = nan(sample_limit, 1);
velocity_d = nan(sample_limit, 1);
quat_w = nan(sample_limit, 1);
quat_x = nan(sample_limit, 1);
quat_y = nan(sample_limit, 1);
quat_z = nan(sample_limit, 1);
quat_norm = nan(sample_limit, 1);
accel_norm = nan(sample_limit, 1);
accel_align_to_minus_gravity = nan(sample_limit, 1);
source_accel_x = nan(sample_limit, 1);
source_accel_y = nan(sample_limit, 1);
source_accel_z = nan(sample_limit, 1);
linear_accel_x = nan(sample_limit, 1);
linear_accel_y = nan(sample_limit, 1);
linear_accel_z = nan(sample_limit, 1);
gravity_body_x = nan(sample_limit, 1);
gravity_body_y = nan(sample_limit, 1);
gravity_body_z = nan(sample_limit, 1);
ground_candidate = false(sample_limit, 1);
all_finite = false(sample_limit, 1);
accel_mode = strings(sample_limit, 1);
accel_explanation = strings(sample_limit, 1);

jsonl_lines = strings(0, 1);

for idx = 1:sample_limit
    frame_index(idx) = idx;
    time_s(idx) = double(live_result.time_s(idx));
    exchange_status(idx) = string(live_result.exchange_status(idx));
    prearm_hold_active(idx) = logical(live_result.prearm_hold_hist(idx));
    if idx <= numel(live_result.sitl_output)
        frame_count(idx) = double(live_result.sitl_output(idx).frame_count);
    end

    raw_text = string(live_result.json_text(idx));
    raw_trim = strip(raw_text);
    if strlength(raw_trim) == 0
        continue;
    end

    try
        parsed = jsondecode(char(raw_trim));
        parse_ok(idx) = true;
        jsonl_lines(end + 1, 1) = raw_trim; %#ok<AGROW>

        required_fields_present(idx) = local_has_required_fields(parsed);
        json_timestamp_s(idx) = local_scalar_or_nan(parsed, 'timestamp');

        if isfield(parsed, 'imu')
            gyro_vec = local_vector_or_nan(parsed.imu, 'gyro', 3);
            accel_vec = local_vector_or_nan(parsed.imu, 'accel_body', 3);
        else
            gyro_vec = nan(3, 1);
            accel_vec = nan(3, 1);
        end

        pos_vec = local_vector_or_nan(parsed, 'position', 3);
        vel_vec = local_vector_or_nan(parsed, 'velocity', 3);
        quat_vec = local_vector_or_nan(parsed, 'quaternion', 4);

        gyro_x(idx) = gyro_vec(1);
        gyro_y(idx) = gyro_vec(2);
        gyro_z(idx) = gyro_vec(3);
        accel_x(idx) = accel_vec(1);
        accel_y(idx) = accel_vec(2);
        accel_z(idx) = accel_vec(3);
        position_n(idx) = pos_vec(1);
        position_e(idx) = pos_vec(2);
        position_d(idx) = pos_vec(3);
        velocity_n(idx) = vel_vec(1);
        velocity_e(idx) = vel_vec(2);
        velocity_d(idx) = vel_vec(3);
        quat_w(idx) = quat_vec(1);
        quat_x(idx) = quat_vec(2);
        quat_y(idx) = quat_vec(3);
        quat_z(idx) = quat_vec(4);
        quat_norm(idx) = norm(quat_vec);
        accel_norm(idx) = norm(accel_vec);

        state_k = live_result.state(idx);
        sensors_k = live_result.sensors(idx);
        [~, accel_diag_k] = uav.ardupilot.convert_imu_accel_for_json( ...
            state_k, sensors_k, time_s(idx), params, cfg);
        source_accel_x(idx) = accel_diag_k.accel_current_mps2(1);
        source_accel_y(idx) = accel_diag_k.accel_current_mps2(2);
        source_accel_z(idx) = accel_diag_k.accel_current_mps2(3);
        linear_accel_x(idx) = accel_diag_k.linear_accel_body_mps2(1);
        linear_accel_y(idx) = accel_diag_k.linear_accel_body_mps2(2);
        linear_accel_z(idx) = accel_diag_k.linear_accel_body_mps2(3);
        gravity_body_x(idx) = accel_diag_k.gravity_body_mps2(1);
        gravity_body_y(idx) = accel_diag_k.gravity_body_mps2(2);
        gravity_body_z(idx) = accel_diag_k.gravity_body_mps2(3);
        ground_candidate(idx) = logical(accel_diag_k.ground_candidate);
        accel_mode(idx) = string(accel_diag_k.mode);
        accel_explanation(idx) = string(accel_diag_k.explanation);

        minus_gravity = -double(accel_diag_k.gravity_body_mps2(:));
        accel_align_to_minus_gravity(idx) = local_alignment_cosine(accel_vec, minus_gravity);
        all_finite(idx) = all(isfinite([ ...
            json_timestamp_s(idx), ...
            gyro_vec(:).', ...
            accel_vec(:).', ...
            pos_vec(:).', ...
            vel_vec(:).', ...
            quat_vec(:).']));
    catch
        parse_ok(idx) = false;
    end
end

frame_table = table( ...
    frame_index, ...
    time_s, ...
    json_timestamp_s, ...
    frame_count, ...
    exchange_status, ...
    prearm_hold_active, ...
    parse_ok, ...
    required_fields_present, ...
    gyro_x, gyro_y, gyro_z, ...
    accel_x, accel_y, accel_z, ...
    position_n, position_e, position_d, ...
    velocity_n, velocity_e, velocity_d, ...
    quat_w, quat_x, quat_y, quat_z, ...
    quat_norm, ...
    accel_norm, ...
    accel_align_to_minus_gravity, ...
    source_accel_x, source_accel_y, source_accel_z, ...
    linear_accel_x, linear_accel_y, linear_accel_z, ...
    gravity_body_x, gravity_body_y, gravity_body_z, ...
    ground_candidate, ...
    all_finite, ...
    accel_mode, ...
    accel_explanation);

jsonl_text = strjoin(jsonl_lines, newline);
if strlength(jsonl_text) > 0
    jsonl_text = jsonl_text + newline;
end

timestamps = json_timestamp_s(parse_ok);
timestamps = timestamps(isfinite(timestamps));
summary = struct();
summary.sample_count = height(frame_table);
summary.parsed_count = sum(parse_ok);
summary.required_fields_all_present = all(required_fields_present(parse_ok));
summary.timestamps_strictly_increasing = all(diff(timestamps) > 0);
summary.quaternion_norm_min = local_nanmin(quat_norm);
summary.quaternion_norm_max = local_nanmax(quat_norm);
summary.accel_norm_min = local_nanmin(accel_norm);
summary.accel_norm_max = local_nanmax(accel_norm);
summary.all_numeric_fields_finite = all(all_finite(parse_ok));
summary.missing_required_frame_indices = frame_index(parse_ok & ~required_fields_present);
end

function log_text = local_make_log_text(result, arm_delay_s, jsonl_path, csv_path)
%LOCAL_MAKE_LOG_TEXT Сформировать текстовый журнал диагностики.

lines = strings(0, 1);
lines(end + 1, 1) = "Диагностика согласованности ИНС ArduPilot JSON";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Режим accel_body: " + string(result.cfg_override.json_accel_mode);
lines(end + 1, 1) = "Удержание модели до arm: " + local_bool_text(result.cfg_override.json_prearm_hold_enabled);
lines(end + 1, 1) = "Задержка перед arm [s]: " + sprintf('%.3f', arm_delay_s);
lines(end + 1, 1) = "baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "baseline json_tx_count: " + string(result.baseline.metrics.json_tx_count);
lines(end + 1, 1) = "baseline last_frame_count: " + string(result.baseline.metrics.last_frame_count);
lines(end + 1, 1) = "arducopter жив после baseline: " + local_bool_text(result.baseline.process_after_live.is_alive);
lines(end + 1, 1) = "valid_rx_count во время arm-прогона: " + string(result.metrics.valid_rx_count);
lines(end + 1, 1) = "response_tx_count во время arm-прогона: " + string(result.metrics.response_tx_count);
lines(end + 1, 1) = "last_frame_count во время arm-прогона: " + string(result.metrics.last_frame_count);
lines(end + 1, 1) = "arm выполнен: " + local_bool_text(result.arm_attempt.arm_succeeded);
lines(end + 1, 1) = "COMMAND_ACK: " + string(result.arm_attempt.ack_result);
lines(end + 1, 1) = "Причина отказа: " + string(result.arm_attempt.failure_reason);
lines(end + 1, 1) = "Обязательные поля присутствуют во всех разобранных кадрах: " + local_bool_text(result.frame_summary.required_fields_all_present);
lines(end + 1, 1) = "timestamp строго возрастает: " + local_bool_text(result.frame_summary.timestamps_strictly_increasing);
lines(end + 1, 1) = "Все числовые поля конечны: " + local_bool_text(result.frame_summary.all_numeric_fields_finite);
lines(end + 1, 1) = "Норма quaternion, min: " + sprintf('%.6f', result.frame_summary.quaternion_norm_min);
lines(end + 1, 1) = "Норма quaternion, max: " + sprintf('%.6f', result.frame_summary.quaternion_norm_max);
lines(end + 1, 1) = "Норма accel_body, min [m/s^2]: " + sprintf('%.6f', result.frame_summary.accel_norm_min);
lines(end + 1, 1) = "Норма accel_body, max [m/s^2]: " + sprintf('%.6f', result.frame_summary.accel_norm_max);
lines(end + 1, 1) = "Файл JSONL первых кадров: " + string(jsonl_path);
lines(end + 1, 1) = "CSV-таблица диагностики: " + string(csv_path);
lines(end + 1, 1) = "";
lines(end + 1, 1) = "STATUSTEXT:";
if isempty(result.arm_attempt.status_texts)
    lines(end + 1, 1) = "  (нет сообщений)";
else
    for idx = 1:numel(result.arm_attempt.status_texts)
        lines(end + 1, 1) = "  " + string(result.arm_attempt.status_texts(idx));
    end
end

log_text = strjoin(lines, newline) + newline;
end

function has_required = local_has_required_fields(parsed)
%LOCAL_HAS_REQUIRED_FIELDS Проверить обязательные поля JSON ArduPilot.

has_required = isfield(parsed, 'timestamp') ...
    && isfield(parsed, 'imu') ...
    && isfield(parsed.imu, 'gyro') ...
    && isfield(parsed.imu, 'accel_body') ...
    && isfield(parsed, 'position') ...
    && isfield(parsed, 'velocity') ...
    && (isfield(parsed, 'quaternion') || isfield(parsed, 'attitude'));
end

function vec = local_vector_or_nan(data, field_name, expected_len)
%LOCAL_VECTOR_OR_NAN Безопасно прочитать числовой вектор.

vec = nan(expected_len, 1);
if isfield(data, field_name) && ~isempty(data.(field_name))
    candidate = double(data.(field_name));
    candidate = candidate(:);
    if numel(candidate) >= expected_len
        vec = candidate(1:expected_len);
    end
end
end

function value = local_scalar_or_nan(data, field_name)
%LOCAL_SCALAR_OR_NAN Безопасно прочитать скаляр.

value = nan;
if isfield(data, field_name) && ~isempty(data.(field_name))
    value = double(data.(field_name));
end
end

function value = local_alignment_cosine(vec_a, vec_b)
%LOCAL_ALIGNMENT_COSINE Косинус угла между двумя векторами.

vec_a = double(vec_a(:));
vec_b = double(vec_b(:));
if ~all(isfinite(vec_a)) || ~all(isfinite(vec_b))
    value = nan;
    return;
end

norm_a = norm(vec_a);
norm_b = norm(vec_b);
if norm_a <= 0.0 || norm_b <= 0.0
    value = nan;
    return;
end

value = dot(vec_a, vec_b) / (norm_a * norm_b);
end

function value = local_nanmin(samples)
%LOCAL_NANMIN Минимум по конечным значениям.

samples = double(samples(:));
samples = samples(isfinite(samples));
if isempty(samples)
    value = nan;
else
    value = min(samples);
end
end

function value = local_nanmax(samples)
%LOCAL_NANMAX Максимум по конечным значениям.

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

function merged = local_overlay_struct(base_struct, overlay_struct)
%LOCAL_OVERLAY_STRUCT Наложить поля диагностического переопределения.

merged = base_struct;
field_names = fieldnames(overlay_struct);
for idx = 1:numel(field_names)
    field_name = field_names{idx};
    merged.(field_name) = overlay_struct.(field_name);
end
end

function local_assign_base(var_name, value)
%LOCAL_ASSIGN_BASE Поместить переменную в base workspace.

assignin('base', var_name, value);
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русский текст.

if logical(flag_value)
    text_value = 'да';
else
    text_value = 'нет';
end
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
    error('uav:task19:imuconsistency:OpenFile', ...
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
