function result = fetch_mavlink_params(cfg, connection_string, varargin)
%FETCH_MAVLINK_PARAMS Считать параметры ArduPilot через MAVLink и WSL Python.
% Назначение:
%   Поднимает временный Python-сценарий внутри выбранного дистрибутива WSL,
%   подключается к ArduPilot по заданной строке соединения MAVLink и
%   считывает либо полный набор параметров, либо указанный список имен.
%
% Входы:
%   cfg               - структура конфигурации ArduPilot JSON/WSL
%   connection_string - строка соединения MAVLink, например "tcp:127.0.0.1:5763"
%
% Параметры:
%   'ParamNames' - строковый массив имен параметров для адресного чтения
%   'FetchAll'   - логический флаг чтения полного списка параметров
%   'Timeout_s'  - предельное время чтения, с
%
% Выходы:
%   result - структура:
%       heartbeat_received - получен ли HEARTBEAT;
%       failure_reason     - текст отказа;
%       param_values       - структура name -> value;
%       param_names        - строковый массив считанных имен;
%       missing_names      - имена, которых не удалось получить адресно;
%       status_code        - код завершения WSL Python;
%       output_text        - текстовый вывод команды.
%
% Единицы измерения:
%   время - секунды;
%   значения параметров - в тех единицах, которые использует ArduPilot.
%
% Допущения:
%   Внутри WSL доступен Python с pymavlink, а ArduPilot уже слушает
%   connection_string.

arguments
    cfg (1,1) struct
    connection_string (1,1) string
end

arguments (Repeating)
    varargin
end

parser = inputParser();
parser.addParameter('ParamNames', strings(0, 1));
parser.addParameter('FetchAll', []);
parser.addParameter('Timeout_s', 25.0);
parser.addParameter('Runtime', "wsl");
parser.parse(varargin{:});

param_names = string(parser.Results.ParamNames(:));
fetch_all = parser.Results.FetchAll;
timeout_s = double(parser.Results.Timeout_s);
runtime_name = lower(string(parser.Results.Runtime));

if isempty(fetch_all)
    fetch_all = isempty(param_names);
end
fetch_all = logical(fetch_all);

if fetch_all
    requested_names = strings(0, 1);
else
    requested_names = unique(param_names, 'stable');
end

python_tmp_path = [tempname, '_fetch_mavlink_params.py'];
json_tmp_path = [tempname, '_fetch_mavlink_params.json'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({python_tmp_path, json_tmp_path})); %#ok<NASGU>

switch runtime_name
    case "wsl"
        python_tmp_path_runtime = uav.ardupilot.windows_to_wsl_path(python_tmp_path);
        json_tmp_path_runtime = uav.ardupilot.windows_to_wsl_path(json_tmp_path);
        python_command = uav.ardupilot.resolve_wsl_python(cfg);
    case "windows"
        python_tmp_path_runtime = string(python_tmp_path);
        json_tmp_path_runtime = string(json_tmp_path);
        python_command = "python";
    otherwise
        error('uav:ardupilot:fetchMavlinkParams:BadRuntime', ...
            'Неподдерживаемое окружение выполнения fetch_mavlink_params: %s', ...
            char(runtime_name));
end

python_lines = strings(0, 1);
python_lines(end + 1, 1) = "from pymavlink import mavutil";
python_lines(end + 1, 1) = "import json";
python_lines(end + 1, 1) = "import math";
python_lines(end + 1, 1) = "import time";
python_lines(end + 1, 1) = "from pathlib import Path";
python_lines(end + 1, 1) = "OUTPUT_PATH = Path(" + local_py(json_tmp_path_runtime) + ")";
python_lines(end + 1, 1) = "CONNECTION_STRING = " + local_py(connection_string);
python_lines(end + 1, 1) = "FETCH_ALL = " + local_py_bool(fetch_all);
python_lines(end + 1, 1) = "TIMEOUT_S = " + sprintf('%.6f', timeout_s);
python_lines(end + 1, 1) = "PARAM_NAMES = [";
for idx = 1:numel(requested_names)
    python_lines(end + 1, 1) = "    " + local_py(requested_names(idx)) + ",";
end
python_lines(end + 1, 1) = "]";
python_lines(end + 1, 1) = "result = {";
python_lines(end + 1, 1) = "    'heartbeat_received': False,";
python_lines(end + 1, 1) = "    'failure_reason': '',";
python_lines(end + 1, 1) = "    'param_values': {},";
python_lines(end + 1, 1) = "    'requested_names': PARAM_NAMES,";
python_lines(end + 1, 1) = "    'expected_param_count': None";
python_lines(end + 1, 1) = "}";
python_lines(end + 1, 1) = "start_t = time.time()";
python_lines(end + 1, 1) = "last_new_t = start_t";
python_lines(end + 1, 1) = "try:";
python_lines(end + 1, 1) = "    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=245, source_component=190)";
python_lines(end + 1, 1) = "    hb = master.wait_heartbeat(timeout=10)";
python_lines(end + 1, 1) = "    if hb is None:";
python_lines(end + 1, 1) = "        raise RuntimeError('Не получен HEARTBEAT по каналу ' + CONNECTION_STRING)";
python_lines(end + 1, 1) = "    result['heartbeat_received'] = True";
python_lines(end + 1, 1) = "    if FETCH_ALL:";
python_lines(end + 1, 1) = "        master.param_fetch_all()";
python_lines(end + 1, 1) = "    else:";
python_lines(end + 1, 1) = "        for name in PARAM_NAMES:";
python_lines(end + 1, 1) = "            try:";
python_lines(end + 1, 1) = "                master.param_fetch_one(name)";
python_lines(end + 1, 1) = "            except Exception:";
python_lines(end + 1, 1) = "                pass";
python_lines(end + 1, 1) = "    while time.time() - start_t < TIMEOUT_S:";
python_lines(end + 1, 1) = "        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)";
python_lines(end + 1, 1) = "        if msg is None:";
python_lines(end + 1, 1) = "            if result['param_values'] and time.time() - last_new_t > 3.0:";
python_lines(end + 1, 1) = "                break";
python_lines(end + 1, 1) = "            continue";
python_lines(end + 1, 1) = "        name = str(getattr(msg, 'param_id', '')).rstrip('\\x00')";
python_lines(end + 1, 1) = "        if not name:";
python_lines(end + 1, 1) = "            continue";
python_lines(end + 1, 1) = "        value = float(getattr(msg, 'param_value', float('nan')))";
python_lines(end + 1, 1) = "        result['param_values'][name] = value if math.isfinite(value) else None";
python_lines(end + 1, 1) = "        count = int(getattr(msg, 'param_count', -1))";
python_lines(end + 1, 1) = "        if count >= 0:";
python_lines(end + 1, 1) = "            result['expected_param_count'] = count";
python_lines(end + 1, 1) = "        last_new_t = time.time()";
python_lines(end + 1, 1) = "        if (not FETCH_ALL) and len(result['param_values']) >= len(PARAM_NAMES):";
python_lines(end + 1, 1) = "            break";
python_lines(end + 1, 1) = "        if FETCH_ALL and result['expected_param_count'] is not None and len(result['param_values']) >= result['expected_param_count']:";
python_lines(end + 1, 1) = "            break";
python_lines(end + 1, 1) = "except Exception as exc:";
python_lines(end + 1, 1) = "    result['failure_reason'] = str(exc)";
python_lines(end + 1, 1) = "OUTPUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')";

uav.ardupilot.write_utf8_text_file(python_tmp_path, strjoin(python_lines, newline) + newline);

switch runtime_name
    case "wsl"
        command = sprintf('wsl -d %s -- bash -lc ''%s %s''', ...
            char(cfg.wsl_distro_name), ...
            char(python_command), ...
            char(python_tmp_path_runtime));
    case "windows"
        command = sprintf('python "%s"', char(python_tmp_path_runtime));
end
[status_code, output_text] = system(command);

if ~isfile(json_tmp_path)
    error('uav:ardupilot:fetchMavlinkParams:NoResultFile', ...
        'Не сформирован JSON-результат чтения параметров. Код=%d. Вывод=%s', ...
        status_code, char(string(output_text)));
end

decoded = jsondecode(fileread(json_tmp_path));
param_names_read = string(fieldnames(decoded.param_values));
missing_names = strings(0, 1);
if ~fetch_all && ~isempty(requested_names)
    missing_names = requested_names(~ismember(requested_names, param_names_read));
end

result = struct();
result.heartbeat_received = isfield(decoded, 'heartbeat_received') && logical(decoded.heartbeat_received);
result.failure_reason = string(local_read_json_field(decoded, 'failure_reason', ""));
result.param_values = decoded.param_values;
result.param_names = sort(param_names_read);
result.requested_names = requested_names;
result.missing_names = missing_names;
result.expected_param_count = double(local_read_json_field(decoded, 'expected_param_count', nan));
result.status_code = double(status_code);
result.output_text = string(output_text);
result.connection_string = connection_string;
result.fetch_all = fetch_all;
result.runtime = runtime_name;
end

function value = local_read_json_field(data, field_name, default_value)
if isfield(data, field_name) && ~isempty(data.(field_name))
    value = data.(field_name);
else
    value = default_value;
end
end

function literal = local_py(text_value)
escaped = replace(string(text_value), "\", "\\");
escaped = replace(escaped, "'", "\\'");
literal = "'" + escaped + "'";
end

function literal = local_py_bool(flag_value)
if flag_value
    literal = "True";
else
    literal = "False";
end
end

function local_cleanup_temp(file_list)
for idx = 1:numel(file_list)
    if isfile(file_list{idx})
        delete(file_list{idx});
    end
end
end
