function result = try_arm_internal_sitl(cfg)
%TRY_ARM_INTERNAL_SITL Подтвердить взведение штатного ArduPilot SITL.
% Назначение:
%   Подключается к внутреннему SITL ArduPilot по MAVLink, переводит
%   аппарат в режим GUIDED, выполняет команду взведения и фиксирует
%   итоговый ACK и последние STATUSTEXT.
%
% Входы:
%   cfg - структура конфигурации ArduPilot/WSL
%
% Выходы:
%   result - структура:
%       arm_succeeded  - логический признак факта взведения;
%       ack_result     - код COMMAND_ACK для arm, если получен;
%       failure_reason - текст первой причины отказа;
%       status_texts   - последние сообщения STATUSTEXT;
%       status_code    - код завершения вспомогательного Python-сценария;
%       raw_output     - текстовый вывод WSL-команды.
%
% Единицы измерения:
%   время - секунды.
%
% Допущения:
%   Внутренний SITL уже запущен и слушает tcp:127.0.0.1:5760.

arguments
    cfg (1,1) struct
end

python_script_path_win = [tempname, '_task21_internal_arm.py'];
output_json_path_win = [tempname, '_task21_internal_arm.json'];
cleanup_py = onCleanup(@() local_delete_if_exists(python_script_path_win)); %#ok<NASGU>
cleanup_json = onCleanup(@() local_delete_if_exists(output_json_path_win)); %#ok<NASGU>

python_script_path_wsl = uav.ardupilot.windows_to_wsl_path(python_script_path_win);
output_json_path_wsl = uav.ardupilot.windows_to_wsl_path(output_json_path_win);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);

py_lines = strings(0, 1);
py_lines(end + 1, 1) = "from pymavlink import mavutil";
py_lines(end + 1, 1) = "import json";
py_lines(end + 1, 1) = "import time";
py_lines(end + 1, 1) = "from pathlib import Path";
py_lines(end + 1, 1) = "output_path = Path(" + local_py(output_json_path_wsl) + ")";
py_lines(end + 1, 1) = "result = {'arm_succeeded': False, 'ack_result': None, 'failure_reason': '', 'status_texts': []}";
py_lines(end + 1, 1) = "status_texts = []";
py_lines(end + 1, 1) = "try:";
py_lines(end + 1, 1) = "    master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=245, source_component=190)";
py_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=20)";
py_lines(end + 1, 1) = "    if hb is None:";
py_lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT по tcp:127.0.0.1:5760 для внутреннего SITL.')";
py_lines(end + 1, 1) = "    time.sleep(20.0)";
py_lines(end + 1, 1) = "    try:";
py_lines(end + 1, 1) = "        master.set_mode_apm('GUIDED')";
py_lines(end + 1, 1) = "    except Exception:";
py_lines(end + 1, 1) = "        master.set_mode('GUIDED')";
py_lines(end + 1, 1) = "    time.sleep(2.0)";
py_lines(end + 1, 1) = "    master.arducopter_arm()";
py_lines(end + 1, 1) = "    deadline = time.time() + 20.0";
py_lines(end + 1, 1) = "    while time.time() < deadline:";
py_lines(end + 1, 1) = "        if master.motors_armed():";
py_lines(end + 1, 1) = "            result['arm_succeeded'] = True";
py_lines(end + 1, 1) = "            break";
py_lines(end + 1, 1) = "        msg = master.recv_match(blocking=True, timeout=1)";
py_lines(end + 1, 1) = "        if msg is None:";
py_lines(end + 1, 1) = "            continue";
py_lines(end + 1, 1) = "        mtype = msg.get_type()";
py_lines(end + 1, 1) = "        if mtype == 'COMMAND_ACK' and int(getattr(msg, 'command', -1)) == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):";
py_lines(end + 1, 1) = "            result['ack_result'] = int(getattr(msg, 'result', -1))";
py_lines(end + 1, 1) = "        elif mtype == 'STATUSTEXT':";
py_lines(end + 1, 1) = "            text = str(getattr(msg, 'text', '')).strip()";
py_lines(end + 1, 1) = "            if text:";
py_lines(end + 1, 1) = "                status_texts.append(text)";
py_lines(end + 1, 1) = "    if status_texts:";
py_lines(end + 1, 1) = "        result['status_texts'] = status_texts[-8:]";
py_lines(end + 1, 1) = "    if not result['arm_succeeded'] and not result['failure_reason']:";
py_lines(end + 1, 1) = "        result['failure_reason'] = ' | '.join(status_texts[-5:]) if status_texts else 'Взведение не подтверждено по тайм-ауту.'";
py_lines(end + 1, 1) = "except Exception as exc:";
py_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
py_lines(end + 1, 1) = "output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";

uav.ardupilot.write_utf8_text_file(python_script_path_win, strjoin(py_lines, newline) + newline);
[status_code, output_text] = system(sprintf('wsl.exe -d %s -- bash -lc ''%s %s''', ...
    char(cfg.wsl_distro_name), ...
    char(python_command_wsl), ...
    char(python_script_path_wsl)));

if isfile(output_json_path_win)
    result = jsondecode(fileread(output_json_path_win));
else
    result = struct();
    result.arm_succeeded = false;
    result.ack_result = nan;
    result.failure_reason = "Не сформирован JSON-результат pymavlink для внутреннего SITL.";
    result.status_texts = strings(0, 1);
end

result.raw_output = string(output_text);
result.status_code = double(status_code);
if ~isfield(result, 'status_texts') || isempty(result.status_texts)
    result.status_texts = strings(0, 1);
else
    result.status_texts = string(result.status_texts(:));
end
if ~isfield(result, 'ack_result') || isempty(result.ack_result)
    result.ack_result = nan;
else
    result.ack_result = double(result.ack_result);
end
if ~isfield(result, 'failure_reason') || isempty(result.failure_reason)
    result.failure_reason = "";
else
    result.failure_reason = string(result.failure_reason);
end
result.arm_succeeded = logical(result.arm_succeeded);
end

function literal = local_py(text_value)
literal = "'" + replace(string(text_value), "'", "\\'") + "'";
end

function local_delete_if_exists(path_text)
path_text = char(string(path_text));
if isfile(path_text)
    delete(path_text);
end
end
