%% DIAGNOSE_ARDUPILOT_MOTOR_OUTPUT_PARAMS Считать фактические параметры выходов ArduPilot.
% Назначение:
%   Запускает рабочий режим `ArduPilot JSON + MATLAB-модель` с профилем
%   TASK-22, подтверждает устойчивый обмен и через `MAVLink` считывает
%   фактические параметры моторных выходов, ограничений ШИМ и главного
%   цикла.
%
% Входы:
%   none
%
% Выходы:
%   task_23_motor_output_params - структура результата в base workspace
%
% Единицы измерения:
%   длительность импульсов ШИМ - микросекунды;
%   частоты - герцы.
%
% Допущения:
%   Используется рабочий профиль `task22_sched_loop_200.parm`, в котором
%   уже сохранены исправления TASK-20...TASK-22.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
log_path = fullfile(logs_dir, 'task_23_motor_output_params.txt');
csv_path = fullfile(reports_dir, 'task_23_motor_output_params.csv');
mat_path = fullfile(reports_dir, 'task_23_motor_output_params.mat');
parm_path = local_select_parm_path(repo_root);

local_prepare_parent(log_path);
local_prepare_parent(csv_path);

cfg = uav.ardupilot.default_json_config();
cleanup_stop = onCleanup(@() uav.ardupilot.stop_existing_sitl( ...
    cfg.wsl_distro_name, ...
    string(cfg.udp_remote_ip), ...
    cfg.mavlink_udp_port)); %#ok<NASGU>

param_names = [ ...
    "MOT_PWM_MIN"; ...
    "MOT_PWM_MAX"; ...
    "MOT_PWM_TYPE"; ...
    "MOT_SPIN_ARM"; ...
    "MOT_SPIN_MIN"; ...
    "MOT_SPIN_MAX"; ...
    "MOT_THST_HOVER"; ...
    "MOT_THST_EXPO"; ...
    "MOT_SAFE_DISARM"; ...
    "MOT_SAFE_TIME"; ...
    "SERVO1_FUNCTION"; ...
    "SERVO2_FUNCTION"; ...
    "SERVO3_FUNCTION"; ...
    "SERVO4_FUNCTION"; ...
    "SERVO1_MIN"; ...
    "SERVO1_MAX"; ...
    "SERVO2_MIN"; ...
    "SERVO2_MAX"; ...
    "SERVO3_MIN"; ...
    "SERVO3_MAX"; ...
    "SERVO4_MIN"; ...
    "SERVO4_MAX"; ...
    "ARMING_CHECK"; ...
    "SCHED_LOOP_RATE"];

baseline = local_run_baseline(repo_root, logs_dir, parm_path, "task_23_motor_params");

result = struct();
result.parm_path = string(parm_path);
result.baseline = baseline;
result.fetch_result = struct();
result.param_table = table();
result.param_row = local_empty_param_row();

if baseline.baseline_restored
    fetch_result = uav.ardupilot.fetch_mavlink_params( ...
        cfg, ...
        "tcp:127.0.0.1:5763", ...
        'ParamNames', param_names, ...
        'Timeout_s', 25.0);
    row = local_param_row_from_fetch(fetch_result);
    result.fetch_result = fetch_result;
    result.param_row = row;
    result.param_table = struct2table(row);
else
    row = local_empty_param_row();
    row.failure_reason = string(baseline.first_failure_reason);
    result.param_row = row;
    result.param_table = struct2table(row);
end

writetable(result.param_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_23_motor_output_params', result);
log_text = local_make_log_text(result);
if ismissing(log_text) || strlength(strtrim(log_text)) == 0
    log_text = local_make_fallback_log_text(result);
end
uav.ardupilot.write_utf8_text_file(log_path, log_text);
local_ensure_nonempty_log(log_path, log_text, result);

fprintf('TASK-23: параметры выходов ArduPilot\n');
fprintf('  baseline restored                     : %s\n', local_bool_text(baseline.baseline_restored));
fprintf('  valid_rx_count                        : %d\n', baseline.metrics.valid_rx_count);
fprintf('  response_tx_count                     : %d\n', baseline.metrics.response_tx_count);
fprintf('  SCHED_LOOP_RATE [Hz]                  : %.0f\n', result.param_row.SCHED_LOOP_RATE);
fprintf('  MOT_PWM_MIN/MAX [us]                  : %.0f / %.0f\n', result.param_row.MOT_PWM_MIN, result.param_row.MOT_PWM_MAX);
fprintf('  MOT_SPIN_ARM / MOT_SPIN_MIN / HOVER   : %.6f / %.6f / %.6f\n', ...
    result.param_row.MOT_SPIN_ARM, result.param_row.MOT_SPIN_MIN, result.param_row.MOT_THST_HOVER);

function parm_path = local_select_parm_path(repo_root)
%LOCAL_SELECT_PARM_PATH Выбрать активный профиль TASK-23.

parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task22_sched_loop_200.parm');
if evalin('base', 'exist(''task23_active_parm_path'', ''var'')')
    candidate = string(evalin('base', 'task23_active_parm_path'));
    if strlength(candidate) > 0
        parm_path = char(candidate);
    end
end

if ~isfile(parm_path)
    error('uav:task23:motorParams:MissingParm', ...
        'Не найден профиль запуска TASK-23: %s', parm_path);
end
end

function baseline = local_run_baseline(repo_root, logs_dir, parm_path, tag)
%LOCAL_RUN_BASELINE Запустить устойчивый JSON/UDP-обмен на рабочем профиле.

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

function row = local_param_row_from_fetch(fetch_result)
%LOCAL_PARAM_ROW_FROM_FETCH Сформировать строку параметров из MAVLink-ответа.

row = local_empty_param_row();
row.heartbeat_received = logical(fetch_result.heartbeat_received);
row.fetch_status_code = double(fetch_result.status_code);
row.failure_reason = string(fetch_result.failure_reason);
row.missing_param_count = double(numel(fetch_result.missing_names));

field_names = local_param_field_names();
for idx = 1:numel(field_names)
    field_name = field_names(idx);
    row.(field_name) = local_param_value(fetch_result.param_values, field_name);
end
end

function names = local_param_field_names()
%LOCAL_PARAM_FIELD_NAMES Вернуть имена числовых полей отчета.

names = [ ...
    "MOT_PWM_MIN"; "MOT_PWM_MAX"; "MOT_PWM_TYPE"; ...
    "MOT_SPIN_ARM"; "MOT_SPIN_MIN"; "MOT_SPIN_MAX"; ...
    "MOT_THST_HOVER"; "MOT_THST_EXPO"; ...
    "MOT_SAFE_DISARM"; "MOT_SAFE_TIME"; ...
    "SERVO1_FUNCTION"; "SERVO2_FUNCTION"; "SERVO3_FUNCTION"; "SERVO4_FUNCTION"; ...
    "SERVO1_MIN"; "SERVO1_MAX"; "SERVO2_MIN"; "SERVO2_MAX"; ...
    "SERVO3_MIN"; "SERVO3_MAX"; "SERVO4_MIN"; "SERVO4_MAX"; ...
    "ARMING_CHECK"; "SCHED_LOOP_RATE"];
end

function row = local_empty_param_row()
%LOCAL_EMPTY_PARAM_ROW Создать пустую строку отчета.

row = struct();
row.heartbeat_received = false;
row.fetch_status_code = nan;
row.failure_reason = "";
row.missing_param_count = nan;
field_names = local_param_field_names();
for idx = 1:numel(field_names)
    row.(char(field_names(idx))) = nan;
end
end

function value = local_param_value(data, name)
%LOCAL_PARAM_VALUE Прочитать значение параметра из структуры.

field_name = char(name);
if isstruct(data) && isfield(data, field_name) && ~isempty(data.(field_name))
    value = double(data.(field_name));
else
    value = nan;
end
end

function text_value = local_make_log_text(result)
%LOCAL_MAKE_LOG_TEXT Сформировать текстовый журнал сценария.

baseline = result.baseline;
row = result.param_row;
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-23: фактические параметры выходов ArduPilot";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль запуска: " + string(result.parm_path);
lines(end + 1, 1) = "Baseline restored: " + local_bool_text(baseline.baseline_restored);
lines(end + 1, 1) = "baseline valid_rx_count: " + string(baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "baseline response_tx_count: " + string(baseline.metrics.response_tx_count);
lines(end + 1, 1) = "baseline last_frame_count: " + string(baseline.metrics.last_frame_count);
lines(end + 1, 1) = "arducopter alive after baseline: " + local_bool_text(baseline.process_after_live.is_alive);
lines(end + 1, 1) = "fetch heartbeat received: " + local_bool_text(row.heartbeat_received);
lines(end + 1, 1) = "fetch status code: " + string(row.fetch_status_code);
lines(end + 1, 1) = "fetch failure reason: " + local_empty_as_none(row.failure_reason);
lines(end + 1, 1) = "missing parameter count: " + string(row.missing_param_count);
lines(end + 1, 1) = "";
field_names = local_param_field_names();
for idx = 1:numel(field_names)
    field_name = char(field_names(idx));
    lines(end + 1, 1) = string(field_name) + " = " + string(row.(field_name)); %#ok<AGROW>
end
text_value = strjoin(lines, newline) + newline;
end

function local_prepare_parent(path_value)
%LOCAL_PREPARE_PARENT Создать каталог для файла.

parent_dir = fileparts(path_value);
if strlength(string(parent_dir)) > 0 && ~isfolder(parent_dir)
    mkdir(parent_dir);
end
end

function text_value = local_make_fallback_log_text(result)
baseline = result.baseline;
row = result.param_row;
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-23: резервный журнал параметров выходов ArduPilot";
lines(end + 1, 1) = "baseline restored: " + local_bool_text(baseline.baseline_restored);
lines(end + 1, 1) = "valid_rx_count: " + string(baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "response_tx_count: " + string(baseline.metrics.response_tx_count);
lines(end + 1, 1) = "MOT_PWM_MIN: " + string(row.MOT_PWM_MIN);
lines(end + 1, 1) = "MOT_PWM_MAX: " + string(row.MOT_PWM_MAX);
lines(end + 1, 1) = "MOT_SPIN_ARM: " + string(row.MOT_SPIN_ARM);
lines(end + 1, 1) = "MOT_SPIN_MIN: " + string(row.MOT_SPIN_MIN);
lines(end + 1, 1) = "MOT_THST_HOVER: " + string(row.MOT_THST_HOVER);
lines(end + 1, 1) = "SCHED_LOOP_RATE: " + string(row.SCHED_LOOP_RATE);
text_value = strjoin(lines, newline) + newline;
end

function local_ensure_nonempty_log(log_path, log_text, result)
%LOCAL_ENSURE_NONEMPTY_LOG Повторно записать журнал, если файл остался пустым.

info = dir(log_path);
if ~isempty(info) && info.bytes > 0
    return;
end

text_value = string(log_text);
text_value(ismissing(text_value)) = "";
if strlength(strtrim(text_value)) == 0
    text_value = local_make_fallback_log_text(result);
end

fid = fopen(log_path, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task23:motorParams:EmptyLogRewriteFailed', ...
        'Не удалось повторно открыть журнал %s после нулевой записи.', log_path);
end

cleanup_fid = onCleanup(@() fclose(fid)); %#ok<NASGU>
payload = unicode2native(char(text_value), 'UTF-8');
written = fwrite(fid, payload, 'uint8');
if written == 0 && ~isempty(payload)
    error('uav:task23:motorParams:EmptyLogRewriteFailed', ...
        'Не удалось повторно записать непустой журнал %s.', log_path);
end
end

function local_cleanup_temp(file_list)
%LOCAL_CLEANUP_TEMP Удалить временные файлы.

for idx = 1:numel(file_list)
    if isfile(file_list{idx})
        delete(file_list{idx});
    end
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Представить логический признак по-русски.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function text_value = local_empty_as_none(value)
%LOCAL_EMPTY_AS_NONE Вернуть строку "нет", если значение пусто.

value = string(value);
if strlength(strtrim(value)) == 0
    text_value = "нет";
else
    text_value = value;
end
end
