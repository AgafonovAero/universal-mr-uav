%% RUN_ARDUPILOT_ARM_PWM_RESPONSE Сравнить ШИМ до и после попытки взведения.
% Назначение:
%   Выполняет 20-секундный прогон обмена ArduPilot JSON/UDP и через
%   заданную задержку запускает асинхронную попытку взведения ArduPilot
%   через pymavlink внутри WSL. После завершения прогона формирует
%   отдельные сводки для интервалов до и после попытки взведения.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_arm_pwm_response - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время - секунды;
%   длительность импульсов ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   ArduPilot SITL уже запущен и доступен по TCP-порту 5762 для выдачи
%   MAVLink HEARTBEAT и приема команды взведения.

repo_root = fileparts(fileparts(mfilename('fullpath')));
live_backend_script = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

cfg = uav.ardupilot.default_json_config();
[mat_report_path, csv_report_path] = local_get_report_paths(reports_dir);
[arm_helper_win, arm_result_win, arm_helper_log_win, arm_launcher_win, arm_launch_command] = local_prepare_async_arm_helper(cfg);

cleanup_helper = onCleanup(@() local_delete_if_exists(arm_helper_win)); %#ok<NASGU>
cleanup_result = onCleanup(@() local_delete_if_exists(arm_result_win)); %#ok<NASGU>
cleanup_helper_log = onCleanup(@() local_delete_if_exists(arm_helper_log_win)); %#ok<NASGU>
cleanup_launcher = onCleanup(@() local_delete_if_exists(arm_launcher_win)); %#ok<NASGU>

evalin('base', 'clear(''ardupilot_arm_attempt'');');

[launch_status, launch_output] = system(arm_launch_command); %#ok<ASGLU>

local_apply_default_override('ardupilot_live_backend_duration_s', 20.0);
local_apply_default_override('ardupilot_live_backend_update_rate_hz', cfg.update_rate_hz);
run(live_backend_script);
live_result = evalin('base', 'ardupilot_json_udp_live_backend');

arm_result = local_read_async_arm_result(arm_result_win, arm_helper_log_win, launch_status, launch_output);
assignin('base', 'ardupilot_arm_attempt', arm_result);

before_result = local_extract_phase_summary(live_result, [0.0, 10.0]);
after_result = local_extract_phase_summary(live_result, [10.0, inf]);

response = struct();
response.live_result = live_result;
response.before = before_result;
response.arm_attempt = arm_result;
response.after = after_result;
response.delta_pwm_us = local_safe_delta(before_result.last_pwm_us, after_result.last_pwm_us);
response.delta_motor_cmd_radps = local_safe_delta(before_result.last_motor_cmd_radps, after_result.last_motor_cmd_radps);
response.arm_launch_command = string(arm_launch_command);
response.mat_report_path = string(mat_report_path);
response.csv_report_path = string(csv_report_path);

save(mat_report_path, 'response');
writetable(local_make_csv_table(live_result), csv_report_path);

assignin('base', 'ardupilot_arm_pwm_response', response);

fprintf('Сравнение ШИМ до и после попытки взведения\n');
fprintf('  взведение выполнено                   : %s\n', local_bool_text(arm_result.arm_succeeded));
fprintf('  valid_rx_count до взведения           : %d\n', before_result.valid_rx_count);
fprintf('  valid_rx_count после взведения        : %d\n', after_result.valid_rx_count);
fprintf('  последние ШИМ до взведения [us]       : [%s]\n', local_format_vector(before_result.last_pwm_us));
fprintf('  последние ШИМ после взведения [us]    : [%s]\n', local_format_vector(after_result.last_pwm_us));
fprintf('  приращение ШИМ [us]                   : [%s]\n', local_format_vector(response.delta_pwm_us));
fprintf('  причина                               : %s\n', char(arm_result.failure_reason));

function local_apply_default_override(var_name, default_value)
%LOCAL_APPLY_DEFAULT_OVERRIDE Установить значение по умолчанию в base workspace.

if ~evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    assignin('base', var_name, default_value);
end
end

function [mat_report_path, csv_report_path] = local_get_report_paths(reports_dir)
%LOCAL_GET_REPORT_PATHS Определить пути сохранения результатов.

mat_report_path = fullfile(reports_dir, 'task_16_arm_pwm_response.mat');
csv_report_path = fullfile(reports_dir, 'task_16_arm_pwm_response.csv');

if evalin('base', 'exist(''ardupilot_arm_pwm_response_mat_path'', ''var'')')
    mat_report_path = char(evalin('base', 'ardupilot_arm_pwm_response_mat_path'));
end

if evalin('base', 'exist(''ardupilot_arm_pwm_response_csv_path'', ''var'')')
    csv_report_path = char(evalin('base', 'ardupilot_arm_pwm_response_csv_path'));
end
end

function [helper_path_win, result_path_win, helper_log_win, launcher_path_win, launch_command] = local_prepare_async_arm_helper(cfg)
%LOCAL_PREPARE_ASYNC_ARM_HELPER Подготовить Python-помощник для взведения.

helper_path_win = [tempname, '.py'];
launcher_path_win = [tempname, '.ps1'];
result_path_win = [tempname, '.json'];
helper_log_win = [tempname, '.log'];

helper_path_wsl = local_windows_to_wsl_path(helper_path_win);
result_path_wsl = local_windows_to_wsl_path(result_path_win);
helper_log_wsl = local_windows_to_wsl_path(helper_log_win);
arm_attempt_delay_s = local_read_arm_delay_override(10.0);
python_command_wsl = local_resolve_wsl_python(cfg);

python_lines = strings(0, 1);
python_lines(end + 1, 1) = "from pymavlink import mavutil";
python_lines(end + 1, 1) = "import json";
python_lines(end + 1, 1) = "import time";
python_lines(end + 1, 1) = "result = {";
python_lines(end + 1, 1) = "    'attempted': True,";
python_lines(end + 1, 1) = "    'method': 'pymavlink-async',";
python_lines(end + 1, 1) = "    'command_text': 'python3 async arm helper over tcp:127.0.0.1:5763',";
python_lines(end + 1, 1) = "    'heartbeat_received': False,";
python_lines(end + 1, 1) = "    'arm_succeeded': False,";
python_lines(end + 1, 1) = "    'ack_result': None,";
python_lines(end + 1, 1) = "    'status_texts': [],";
python_lines(end + 1, 1) = "    'failure_reason': '',";
python_lines(end + 1, 1) = "    'system_id': 0,";
python_lines(end + 1, 1) = "    'component_id': 0,";
python_lines(end + 1, 1) = "    'elapsed_s': 0.0";
python_lines(end + 1, 1) = "}";
python_lines(end + 1, 1) = "start = time.time()";
python_lines(end + 1, 1) = "try:";
python_lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=245, source_component=190)";
python_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
python_lines(end + 1, 1) = "    if hb is None:";
python_lines(end + 1, 1) = "        result['failure_reason'] = 'Не получен HEARTBEAT на TCP-порту 5763.'";
python_lines(end + 1, 1) = "    else:";
python_lines(end + 1, 1) = "        result['heartbeat_received'] = True";
python_lines(end + 1, 1) = "        result['system_id'] = int(hb.get_srcSystem())";
python_lines(end + 1, 1) = "        result['component_id'] = int(hb.get_srcComponent())";
python_lines(end + 1, 1) = "        try:";
python_lines(end + 1, 1) = "            master.set_mode_apm('STABILIZE')";
python_lines(end + 1, 1) = "        except Exception:";
python_lines(end + 1, 1) = "            pass";
python_lines(end + 1, 1) = "        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)";
python_lines(end + 1, 1) = "        deadline = time.time() + 10.0";
python_lines(end + 1, 1) = "        while time.time() < deadline:";
python_lines(end + 1, 1) = "            msg = master.recv_match(blocking=True, timeout=1)";
python_lines(end + 1, 1) = "            if msg is None:";
python_lines(end + 1, 1) = "                continue";
python_lines(end + 1, 1) = "            msg_type = msg.get_type()";
python_lines(end + 1, 1) = "            if msg_type == 'COMMAND_ACK' and int(msg.command) == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):";
python_lines(end + 1, 1) = "                result['ack_result'] = int(msg.result)";
python_lines(end + 1, 1) = "            elif msg_type == 'STATUSTEXT':";
python_lines(end + 1, 1) = "                text_value = getattr(msg, 'text', '')";
python_lines(end + 1, 1) = "                if text_value is not None:";
python_lines(end + 1, 1) = "                    result['status_texts'].append(str(text_value).strip())";
python_lines(end + 1, 1) = "            elif msg_type == 'HEARTBEAT':";
python_lines(end + 1, 1) = "                is_armed = bool(int(msg.base_mode) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))";
python_lines(end + 1, 1) = "                if is_armed:";
python_lines(end + 1, 1) = "                    result['arm_succeeded'] = True";
python_lines(end + 1, 1) = "                    break";
python_lines(end + 1, 1) = "        arm_status = [text for text in result['status_texts'] if str(text).startswith('Arm:')]";
python_lines(end + 1, 1) = "        if (not result['arm_succeeded']) and arm_status:";
python_lines(end + 1, 1) = "            result['failure_reason'] = arm_status[-1]";
python_lines(end + 1, 1) = "        elif (not result['arm_succeeded']) and (result['ack_result'] is not None):";
python_lines(end + 1, 1) = "            result['failure_reason'] = f""Команда взведения отклонена с кодом {result['ack_result']}.""";
python_lines(end + 1, 1) = "        elif (not result['arm_succeeded']) and result['status_texts']:";
python_lines(end + 1, 1) = "            result['failure_reason'] = result['status_texts'][-1]";
python_lines(end + 1, 1) = "        elif not result['arm_succeeded']:";
python_lines(end + 1, 1) = "            result['failure_reason'] = 'ArduPilot не подтвердил состояние взведения.'";
python_lines(end + 1, 1) = "except Exception as exc:";
python_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
python_lines(end + 1, 1) = "result['elapsed_s'] = time.time() - start";
python_lines(end + 1, 1) = "with open(r'" + replace(result_path_wsl, "\", "\\") + "', 'w', encoding='utf-8') as fid:";
python_lines(end + 1, 1) = "    json.dump(result, fid, ensure_ascii=False)";

local_write_utf8_text(helper_path_win, strjoin(python_lines, newline) + newline);
local_delete_if_exists(result_path_win);
local_delete_if_exists(helper_log_win);

launcher_lines = strings(0, 1);
launcher_lines(end + 1, 1) = "$ArgumentList = @(";
launcher_lines(end + 1, 1) = "    '-d',";
launcher_lines(end + 1, 1) = "    '" + string(cfg.wsl_distro_name) + "',";
launcher_lines(end + 1, 1) = "    '--',";
launcher_lines(end + 1, 1) = "    'bash',";
launcher_lines(end + 1, 1) = "    '-lc',";
launcher_lines(end + 1, 1) = "    'sleep " + sprintf('%.3f', arm_attempt_delay_s) + "; " + python_command_wsl + " " + helper_path_wsl + " >" + helper_log_wsl + " 2>&1'";
launcher_lines(end + 1, 1) = ")";
launcher_lines(end + 1, 1) = "Start-Process -WindowStyle Hidden -FilePath 'wsl.exe' -ArgumentList $ArgumentList | Out-Null";
local_write_utf8_text(launcher_path_win, strjoin(launcher_lines, newline) + newline);

launch_command = sprintf( ...
    'powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', ...
    char(launcher_path_win));
end

function arm_result = local_read_async_arm_result(result_path_win, helper_log_win, launch_status, launch_output)
%LOCAL_READ_ASYNC_ARM_RESULT Прочитать результат внешней попытки взведения.

deadline = tic;
while toc(deadline) < 25.0
    if isfile(result_path_win)
        break;
    end
    pause(0.2);
end

if isfile(result_path_win)
    raw_text = fileread(result_path_win);
    parsed = jsondecode(raw_text);

    arm_result = struct();
    arm_result.attempted = logical(parsed.attempted);
    arm_result.method = string(parsed.method);
    arm_result.command_text = string(parsed.command_text);
    arm_result.arm_succeeded = logical(parsed.arm_succeeded);
    arm_result.heartbeat_received = logical(parsed.heartbeat_received);
    arm_result.ack_result = local_read_optional_number(parsed, 'ack_result', nan);
    arm_result.status_texts = strings(0, 1);
    if isfield(parsed, 'status_texts') && ~isempty(parsed.status_texts)
        arm_result.status_texts = string(parsed.status_texts(:));
    end
    arm_result.failure_reason = string(parsed.failure_reason);
    arm_result.system_id = double(parsed.system_id);
    arm_result.component_id = double(parsed.component_id);
    arm_result.elapsed_s = double(parsed.elapsed_s);
    arm_result.status_code = 0;
    arm_result.raw_output = string(raw_text);
else
    helper_log_text = "";
    if isfile(helper_log_win)
        helper_log_text = string(fileread(helper_log_win));
    end

    arm_result = struct();
    arm_result.attempted = launch_status == 0;
    arm_result.method = "pymavlink-async";
    arm_result.command_text = "";
    arm_result.arm_succeeded = false;
    arm_result.heartbeat_received = false;
    arm_result.ack_result = nan;
    arm_result.status_texts = strings(0, 1);
    if strlength(strtrim(helper_log_text)) > 0
        arm_result.failure_reason = "Не удалось получить JSON-результат внешней попытки взведения; см. raw_output.";
    else
        arm_result.failure_reason = "Не удалось получить результат внешней попытки взведения.";
    end
    arm_result.system_id = 0;
    arm_result.component_id = 0;
    arm_result.elapsed_s = 0.0;
    arm_result.status_code = double(launch_status);
    arm_result.raw_output = string(launch_output) + newline + helper_log_text;
end
end

function value = local_read_optional_number(data, field_name, default_value)
%LOCAL_READ_OPTIONAL_NUMBER Прочитать необязательное числовое поле.

value = default_value;
if isfield(data, field_name) && ~isempty(data.(field_name))
    value = double(data.(field_name));
end
end

function phase_summary = local_extract_phase_summary(live_result, interval_s)
%LOCAL_EXTRACT_PHASE_SUMMARY Сформировать сводку по временной фазе.

time_s = local_get_field(live_result, 'time_s', zeros(0, 1));
motor_cmd_radps = local_get_field(live_result, 'motor_cmd_radps', zeros(0, 4));
sitl_output = local_get_field( ...
    live_result, ...
    'sitl_output', ...
    repmat(struct('valid', false, 'motor_pwm_us', nan(4, 1)), 0, 1));

mask = time_s >= interval_s(1) & time_s < interval_s(2);

phase_summary = struct();
phase_summary.executed = any(mask);
phase_summary.time_s = time_s(mask);
phase_summary.motor_cmd_radps = motor_cmd_radps(mask, :);
phase_summary.sitl_output = sitl_output(mask);
phase_summary.valid_rx_count = sum(arrayfun(@(s) double(s.valid), phase_summary.sitl_output));
phase_summary.last_pwm_us = nan(4, 1);
phase_summary.last_motor_cmd_radps = nan(4, 1);

if any(mask)
    valid_idx = find(arrayfun(@(s) s.valid, phase_summary.sitl_output), 1, 'last');
    if ~isempty(valid_idx)
        phase_summary.last_pwm_us = double(phase_summary.sitl_output(valid_idx).motor_pwm_us(:));
    end

    finite_rows = find(all(isfinite(phase_summary.motor_cmd_radps), 2), 1, 'last');
    if ~isempty(finite_rows)
        phase_summary.last_motor_cmd_radps = double(phase_summary.motor_cmd_radps(finite_rows, :).');
    end
end
end

function delay_s = local_read_arm_delay_override(default_value)
%LOCAL_READ_ARM_DELAY_OVERRIDE Прочитать необязательное значение задержки взведения.

delay_s = default_value;

if evalin('base', 'exist(''ardupilot_arm_attempt_delay_s'', ''var'')')
    candidate_value = evalin('base', 'ardupilot_arm_attempt_delay_s');
    validateattributes(candidate_value, {'numeric'}, ...
        {'real', 'scalar', 'finite', 'nonnegative'}, ...
        mfilename, 'ardupilot_arm_attempt_delay_s');
    delay_s = double(candidate_value);
end
end

function python_command_wsl = local_resolve_wsl_python(cfg)
%LOCAL_RESOLVE_WSL_PYTHON Определить интерпретатор Python с pymavlink внутри WSL.

preferred_command = "/home/oaleg/venv-ardupilot/bin/python3";
probe_command = sprintf( ...
    'wsl -d %s -- bash -lc ''if [ -x %s ]; then printf %s; else command -v python3; fi''', ...
    char(cfg.wsl_distro_name), ...
    preferred_command, ...
    preferred_command);
[status_code, raw_output] = system(probe_command);
raw_output = strtrim(string(raw_output));

if status_code == 0 && strlength(raw_output) > 0
    python_command_wsl = raw_output;
else
    python_command_wsl = "python3";
end
end

function value = local_get_field(struct_value, field_name, default_value)
%LOCAL_GET_FIELD Безопасно прочитать поле структуры.

if isstruct(struct_value) && isfield(struct_value, field_name)
    value = struct_value.(field_name);
else
    value = default_value;
end
end

function delta_value = local_safe_delta(before_value, after_value)
%LOCAL_SAFE_DELTA Вычислить приращение между значениями до и после.

delta_value = nan(size(before_value));
if all(isfinite(before_value)) && all(isfinite(after_value))
    delta_value = double(after_value) - double(before_value);
end
end

function csv_table = local_make_csv_table(live_result)
%LOCAL_MAKE_CSV_TABLE Сформировать таблицу CSV с фазой прогона.

if ~isstruct(live_result) || ...
        ~isfield(live_result, 'time_s') || ...
        ~isfield(live_result, 'sitl_output') || ...
        ~isfield(live_result, 'motor_cmd_radps')
    csv_table = table( ...
        strings(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        zeros(0, 1), ...
        strings(0, 1), ...
        'VariableNames', { ...
            'phase', 'time_s', 'frame_count', ...
            'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
            'motor_cmd_1_radps', 'motor_cmd_2_radps', ...
            'motor_cmd_3_radps', 'motor_cmd_4_radps', ...
            'exchange_status'});
    return;
end

n_rows = numel(live_result.time_s);
phase_labels = repmat("before_arm", n_rows, 1);
phase_labels(live_result.time_s >= 10.0) = "after_arm";

frame_count = arrayfun(@(s) double(s.frame_count), live_result.sitl_output);
pwm_matrix = nan(n_rows, 4);
for idx = 1:n_rows
    if live_result.sitl_output(idx).valid
        pwm_matrix(idx, :) = double(live_result.sitl_output(idx).motor_pwm_us(:)).';
    end
end

csv_table = table( ...
    phase_labels, ...
    live_result.time_s(:), ...
    frame_count(:), ...
    pwm_matrix(:, 1), ...
    pwm_matrix(:, 2), ...
    pwm_matrix(:, 3), ...
    pwm_matrix(:, 4), ...
    live_result.motor_cmd_radps(:, 1), ...
    live_result.motor_cmd_radps(:, 2), ...
    live_result.motor_cmd_radps(:, 3), ...
    live_result.motor_cmd_radps(:, 4), ...
    string(local_get_field(live_result, 'exchange_status', strings(n_rows, 1))), ...
    'VariableNames', { ...
        'phase', 'time_s', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_cmd_1_radps', 'motor_cmd_2_radps', ...
        'motor_cmd_3_radps', 'motor_cmd_4_radps', ...
        'exchange_status'});
end

function wsl_path = local_windows_to_wsl_path(path_value)
%LOCAL_WINDOWS_TO_WSL_PATH Преобразовать путь Windows в путь WSL.

path_value = string(path_value);
full_path = string(java.io.File(char(path_value)).getCanonicalPath());
drive_letter = lower(extractBefore(full_path, ":\"));
tail_value = extractAfter(full_path, ":\");
tail_value = replace(tail_value, "\", "/");

if startsWith(tail_value, "/")
    tail_value = extractAfter(tail_value, 1);
end

wsl_path = "/mnt/" + drive_letter + "/" + tail_value;
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить UTF-8 текст без BOM.

folder_path = fileparts(path_value);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task17:run_ardupilot_arm_pwm_response:OpenFile', ...
        'Не удалось открыть файл %s.', path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function local_delete_if_exists(path_value)
%LOCAL_DELETE_IF_EXISTS Удалить временный файл, если он существует.

if isfile(path_value)
    delete(path_value);
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать строку вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end
