%% TRY_ARDUPILOT_ARM Попытаться выполнить взведение ArduPilot через pymavlink.
% Назначение:
%   Выполняет автоматическую попытку взведения ArduPilot по локальному
%   каналу TCP внутри WSL и сохраняет структурированный результат с
%   причиной отказа, если взведение не состоялось.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_arm_attempt - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время - секунды
%
% Допущения:
%   ArduPilot SITL уже запущен и предоставляет TCP-порт MAVLink 5762.

cfg = uav.ardupilot.default_json_config();
python_path_win = [tempname, '.py'];
python_path_wsl = local_windows_to_wsl_path(python_path_win);

python_lines = strings(0, 1);
python_lines(end + 1, 1) = "from pymavlink import mavutil";
python_lines(end + 1, 1) = "import json";
python_lines(end + 1, 1) = "import time";
python_lines(end + 1, 1) = "result = {";
python_lines(end + 1, 1) = "    'attempted': True,";
python_lines(end + 1, 1) = "    'method': 'pymavlink',";
python_lines(end + 1, 1) = "    'command_text': 'python3 try_ardupilot_arm helper over tcp:127.0.0.1:5762',";
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
python_lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=245, source_component=190)";
python_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=15)";
python_lines(end + 1, 1) = "    if hb is None:";
python_lines(end + 1, 1) = "        result['failure_reason'] = 'Не получен HEARTBEAT на TCP-порту 5762.'";
python_lines(end + 1, 1) = "    else:";
python_lines(end + 1, 1) = "        result['heartbeat_received'] = True";
python_lines(end + 1, 1) = "        result['system_id'] = int(hb.get_srcSystem())";
python_lines(end + 1, 1) = "        result['component_id'] = int(hb.get_srcComponent())";
python_lines(end + 1, 1) = "        try:";
python_lines(end + 1, 1) = "            master.set_mode_apm('STABILIZE')";
python_lines(end + 1, 1) = "        except Exception:";
python_lines(end + 1, 1) = "            pass";
python_lines(end + 1, 1) = "        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)";
python_lines(end + 1, 1) = "        deadline = time.time() + 15.0";
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
python_lines(end + 1, 1) = "        if not result['arm_succeeded'] and result['status_texts']:";
python_lines(end + 1, 1) = "            result['failure_reason'] = result['status_texts'][-1]";
python_lines(end + 1, 1) = "        elif not result['arm_succeeded'] and result['ack_result'] is not None:";
python_lines(end + 1, 1) = "            result['failure_reason'] = f""Команда взведения отклонена с кодом {result['ack_result']}.""";
python_lines(end + 1, 1) = "        elif not result['arm_succeeded']:";
python_lines(end + 1, 1) = "            result['failure_reason'] = 'ArduPilot не подтвердил состояние взведения.'";
python_lines(end + 1, 1) = "except Exception as exc:";
python_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
python_lines(end + 1, 1) = "result['elapsed_s'] = time.time() - start";
python_lines(end + 1, 1) = "print(json.dumps(result, ensure_ascii=False))";

local_write_utf8_text(python_path_win, strjoin(python_lines, newline) + newline);
cleanup_obj = onCleanup(@() local_delete_if_exists(python_path_win)); %#ok<NASGU>

command_text = sprintf( ...
    'wsl -d %s -- bash -lc "python3 %s"', ...
    char(cfg.wsl_distro_name), ...
    char(python_path_wsl));
[status_code, output_text] = system(command_text);
output_text = string(strtrim(output_text));

result = struct();
result.attempted = true;
result.method = "pymavlink";
result.command_text = string(command_text);
result.arm_succeeded = false;
result.heartbeat_received = false;
result.ack_result = nan;
result.status_texts = strings(0, 1);
result.failure_reason = "";
result.system_id = 0;
result.component_id = 0;
result.elapsed_s = 0.0;
result.status_code = double(status_code);
result.raw_output = output_text;

if status_code == 0 && startsWith(output_text, "{")
    parsed = jsondecode(char(output_text));
    result.arm_succeeded = logical(parsed.arm_succeeded);
    result.heartbeat_received = logical(parsed.heartbeat_received);

    if isempty(parsed.ack_result)
        result.ack_result = nan;
    else
        result.ack_result = double(parsed.ack_result);
    end

    result.failure_reason = string(parsed.failure_reason);
    result.system_id = double(parsed.system_id);
    result.component_id = double(parsed.component_id);
    result.elapsed_s = double(parsed.elapsed_s);

    if isfield(parsed, 'status_texts') && ~isempty(parsed.status_texts)
        result.status_texts = string(parsed.status_texts(:));
    end
else
    result.failure_reason = "Не удалось получить структурированный ответ от pymavlink.";
end

assignin('base', 'ardupilot_arm_attempt', result);
local_save_result(result);

fprintf('Попытка взведения ArduPilot\n');
fprintf('  метод                                : %s\n', char(result.method));
fprintf('  HEARTBEAT по TCP 5762 получен         : %s\n', local_bool_text(result.heartbeat_received));
fprintf('  взведение выполнено                   : %s\n', local_bool_text(result.arm_succeeded));
if isfinite(result.ack_result)
    fprintf('  код подтверждения команды             : %.0f\n', result.ack_result);
end
fprintf('  время ожидания [s]                    : %.3f\n', result.elapsed_s);
fprintf('  причина                               : %s\n', char(result.failure_reason));
if ~isempty(result.status_texts)
    fprintf('  последние сообщения ArduPilot:\n');
    for idx = 1:numel(result.status_texts)
        fprintf('    %s\n', char(result.status_texts(idx)));
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
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:ardupilot:try_ardupilot_arm:OpenFile', ...
        'Не удалось открыть временный файл %s.', path_value);
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

function local_save_result(result)
%LOCAL_SAVE_RESULT Сохранить результат попытки взведения в reports.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

arm_result = result; %#ok<NASGU>
save(fullfile(reports_dir, 'task_16_arm_attempt.mat'), 'arm_result');
end
