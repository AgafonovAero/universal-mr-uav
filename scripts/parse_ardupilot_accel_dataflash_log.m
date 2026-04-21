%% PARSE_ARDUPILOT_ACCEL_DATAFLASH_LOG Извлечь акселерометры из DataFlash.
% Назначение:
%   Читает сохраненный `DataFlash`-журнал `ArduPilot`, извлекает записи
%   акселерометров и параметры `INS_*`, оценивает активные экземпляры ИНС и
%   максимальное расхождение между ними перед попыткой взведения.
%
% Входы:
%   none
%
% Выходы:
%   task_20_accel_instances - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   ускорение - в тех единицах, в которых оно сохранено самим ArduPilot в
%   DataFlash;
%   модуль ускорения и различия - те же единицы
%
% Допущения:
%   Внутри `WSL` доступен `pymavlink`, а файл
%   `artifacts/reports/task_20_ardupilot_dataflash.bin` уже сохранен.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

bin_path = fullfile(reports_dir, 'task_20_ardupilot_dataflash.bin');
log_path = fullfile(logs_dir, 'task_20_parse_dataflash_accel_log.txt');
csv_path = fullfile(reports_dir, 'task_20_accel_instances.csv');
mat_path = fullfile(reports_dir, 'task_20_accel_instances.mat');
json_tmp_path = [tempname, '_task20_dataflash_summary.json'];
python_tmp_path = [tempname, '_task20_parse_dataflash.py'];

cleanup_tmp = onCleanup(@() local_cleanup_temp({json_tmp_path, python_tmp_path})); %#ok<NASGU>

if ~isfile(bin_path)
    error('uav:task20:parseDataflash:MissingBin', ...
        'Не найден DataFlash-журнал: %s', bin_path);
end

cfg = uav.ardupilot.default_json_config();
[status_code, output_text] = local_run_parser(cfg, bin_path, csv_path, json_tmp_path, python_tmp_path);
summary = local_read_summary_json(json_tmp_path, status_code, output_text);
accel_table = readtable(csv_path, 'TextType', 'string');

result = struct();
result.summary = summary;
result.accel_table = accel_table;
result.bin_path = string(bin_path);
result.csv_path = string(csv_path);
result.log_path = string(log_path);

save(mat_path, 'result');
assignin('base', 'task_20_accel_instances', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('Разбор DataFlash-журнала TASK-20\n');
fprintf('  active instances                       : %s\n', char(join(string(summary.active_instances), ", ")));
fprintf('  max accel difference                   : %.6f\n', double(summary.max_instance_difference));
fprintf('  prearm max accel difference            : %.6f\n', double(summary.prearm_window_max_difference));
fprintf('  accel rows                             : %d\n', height(accel_table));

function [status_code, output_text] = local_run_parser(cfg, bin_path, csv_path, json_path, python_path)
python_path_wsl = uav.ardupilot.windows_to_wsl_path(python_path);
bin_path_wsl = uav.ardupilot.windows_to_wsl_path(bin_path);
csv_path_wsl = uav.ardupilot.windows_to_wsl_path(csv_path);
json_path_wsl = uav.ardupilot.windows_to_wsl_path(json_path);
python_command_wsl = uav.ardupilot.resolve_wsl_python(cfg);

lines = strings(0, 1);
lines(end + 1, 1) = "from pymavlink import DFReader";
lines(end + 1, 1) = "import csv";
lines(end + 1, 1) = "import json";
lines(end + 1, 1) = "import math";
lines(end + 1, 1) = "from collections import defaultdict";
lines(end + 1, 1) = "from pathlib import Path";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "BIN_PATH = Path(" + local_py(bin_path_wsl) + ")";
lines(end + 1, 1) = "CSV_PATH = Path(" + local_py(csv_path_wsl) + ")";
lines(end + 1, 1) = "JSON_PATH = Path(" + local_py(json_path_wsl) + ")";
lines(end + 1, 1) = "rows = []";
lines(end + 1, 1) = "params = {}";
lines(end + 1, 1) = "messages = []";
lines(end + 1, 1) = "message_types_present = set()";
lines(end + 1, 1) = "grouped = defaultdict(list)";
lines(end + 1, 1) = "prearm_time_us = None";
lines(end + 1, 1) = "reader = DFReader.DFReader_binary(str(BIN_PATH))";
lines(end + 1, 1) = "while True:";
lines(end + 1, 1) = "    msg = reader.recv_msg()";
lines(end + 1, 1) = "    if msg is None:";
lines(end + 1, 1) = "        break";
lines(end + 1, 1) = "    msg_type = msg.get_type()";
lines(end + 1, 1) = "    message_types_present.add(msg_type)";
lines(end + 1, 1) = "    if msg_type == 'PARM':";
lines(end + 1, 1) = "        name = str(getattr(msg, 'Name', '')).rstrip('\\x00')";
lines(end + 1, 1) = "        if name.startswith('INS_') or name.startswith('SIM_') or name in ('ARMING_CHECK','LOG_DISARMED','SCHED_LOOP_RATE'):";
lines(end + 1, 1) = "            params[name] = float(getattr(msg, 'Value', float('nan')))";
lines(end + 1, 1) = "    elif msg_type == 'MSG':";
lines(end + 1, 1) = "        text = str(getattr(msg, 'Message', '')).strip()";
lines(end + 1, 1) = "        if text:";
lines(end + 1, 1) = "            messages.append({'TimeUS': int(getattr(msg, 'TimeUS', 0)), 'text': text})";
lines(end + 1, 1) = "            if prearm_time_us is None and ('Arm:' in text or 'PreArm:' in text):";
lines(end + 1, 1) = "                prearm_time_us = int(getattr(msg, 'TimeUS', 0))";
lines(end + 1, 1) = "    elif msg_type in ('IMU', 'IMU2', 'IMU3', 'ACC'):";
lines(end + 1, 1) = "        time_us = int(getattr(msg, 'TimeUS', 0))";
lines(end + 1, 1) = "        inst = int(getattr(msg, 'I', 0)) if hasattr(msg, 'I') else {'IMU':0,'IMU2':1,'IMU3':2}.get(msg_type, 0)";
lines(end + 1, 1) = "        ax = float(getattr(msg, 'AccX', float('nan')))";
lines(end + 1, 1) = "        ay = float(getattr(msg, 'AccY', float('nan')))";
lines(end + 1, 1) = "        az = float(getattr(msg, 'AccZ', float('nan')))";
lines(end + 1, 1) = "        norm = math.sqrt(ax*ax + ay*ay + az*az) if all(math.isfinite(v) for v in (ax,ay,az)) else float('nan')";
lines(end + 1, 1) = "        row = {'time_us': time_us, 'time_s': time_us*1.0e-6, 'instance_id': inst, 'msg_type': msg_type, 'AccX': ax, 'AccY': ay, 'AccZ': az, 'accel_norm': norm}";
lines(end + 1, 1) = "        rows.append(row)";
lines(end + 1, 1) = "        grouped[(msg_type, time_us)].append(row)";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "max_diff = 0.0";
lines(end + 1, 1) = "prearm_window_max_diff = 0.0";
lines(end + 1, 1) = "prearm_window_start_us = max((prearm_time_us or 0) - 5_000_000, 0)";
lines(end + 1, 1) = "for key, group in grouped.items():";
lines(end + 1, 1) = "    diff = float('nan')";
lines(end + 1, 1) = "    if len(group) >= 2:";
lines(end + 1, 1) = "        diff = 0.0";
lines(end + 1, 1) = "        for i in range(len(group)):";
lines(end + 1, 1) = "            for j in range(i+1, len(group)):";
lines(end + 1, 1) = "                dx = group[i]['AccX'] - group[j]['AccX']";
lines(end + 1, 1) = "                dy = group[i]['AccY'] - group[j]['AccY']";
lines(end + 1, 1) = "                dz = group[i]['AccZ'] - group[j]['AccZ']";
lines(end + 1, 1) = "                cur = math.sqrt(dx*dx + dy*dy + dz*dz)";
lines(end + 1, 1) = "                if cur > diff:";
lines(end + 1, 1) = "                    diff = cur";
lines(end + 1, 1) = "        max_diff = max(max_diff, diff)";
lines(end + 1, 1) = "        if prearm_time_us is not None and prearm_window_start_us <= group[0]['time_us'] <= prearm_time_us:";
lines(end + 1, 1) = "            prearm_window_max_diff = max(prearm_window_max_diff, diff)";
lines(end + 1, 1) = "    for row in group:";
lines(end + 1, 1) = "        row['max_pairwise_difference'] = diff";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "CSV_PATH.parent.mkdir(parents=True, exist_ok=True)";
lines(end + 1, 1) = "with CSV_PATH.open('w', newline='', encoding='utf-8') as fid:";
lines(end + 1, 1) = "    writer = csv.DictWriter(fid, fieldnames=['time_s','time_us','msg_type','instance_id','AccX','AccY','AccZ','accel_norm','max_pairwise_difference'])";
lines(end + 1, 1) = "    writer.writeheader()";
lines(end + 1, 1) = "    for row in rows:";
lines(end + 1, 1) = "        writer.writerow(row)";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "summary = {";
lines(end + 1, 1) = "    'active_instances': sorted({int(row['instance_id']) for row in rows}),";
lines(end + 1, 1) = "    'message_types_present': sorted(message_types_present),";
lines(end + 1, 1) = "    'accel_row_count': len(rows),";
lines(end + 1, 1) = "    'max_instance_difference': max_diff,";
lines(end + 1, 1) = "    'prearm_time_us': prearm_time_us,";
lines(end + 1, 1) = "    'prearm_window_max_difference': prearm_window_max_diff,";
lines(end + 1, 1) = "    'params': params,";
lines(end + 1, 1) = "    'messages': messages[-40:]";
lines(end + 1, 1) = "}";
lines(end + 1, 1) = "JSON_PATH.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')";

uav.ardupilot.write_utf8_text_file(python_path, strjoin(lines, newline) + newline);
command = sprintf('wsl -d %s -- bash -lc ''%s %s''', ...
    char(cfg.wsl_distro_name), ...
    char(python_command_wsl), ...
    char(python_path_wsl));
[status_code, output_text] = system(command);
end

function summary = local_read_summary_json(json_path, status_code, output_text)
if ~isfile(json_path)
    error('uav:task20:parseDataflash:PythonFailed', ...
        'Не удалось получить JSON-сводку разбора DataFlash. Код=%d. Вывод=%s', ...
        status_code, char(string(output_text)));
end

decoded = jsondecode(fileread(json_path));
summary = struct();
summary.status_code = double(status_code);
summary.output_text = string(output_text);
summary.active_instances = double(decoded.active_instances(:)).';
summary.message_types_present = string(decoded.message_types_present(:));
summary.accel_row_count = double(decoded.accel_row_count);
summary.max_instance_difference = double(decoded.max_instance_difference);
summary.prearm_time_us = local_read_optional_number(decoded, 'prearm_time_us', nan);
summary.prearm_window_max_difference = double(decoded.prearm_window_max_difference);
summary.params = decoded.params;
summary.messages = decoded.messages;
end

function value = local_read_optional_number(data, field_name, default_value)
if isfield(data, field_name) && ~isempty(data.(field_name))
    value = double(data.(field_name));
else
    value = default_value;
end
end

function text_value = local_make_log_text(result)
summary = result.summary;
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-20: разбор DataFlash-журнала ArduPilot по акселерометрам";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "BIN: " + result.bin_path;
lines(end + 1, 1) = "CSV: " + result.csv_path;
lines(end + 1, 1) = "Active instances: " + join(string(summary.active_instances), ", ");
lines(end + 1, 1) = "Message types present: " + join(summary.message_types_present, ", ");
lines(end + 1, 1) = "Accel row count: " + string(summary.accel_row_count);
lines(end + 1, 1) = "Max accel difference: " + sprintf('%.6f', summary.max_instance_difference);
lines(end + 1, 1) = "Prearm window max difference: " + sprintf('%.6f', summary.prearm_window_max_difference);
lines(end + 1, 1) = "Selected INS params:";
param_names = sort(string(fieldnames(summary.params)));
for idx = 1:numel(param_names)
    pname = param_names(idx);
    lines(end + 1, 1) = "  " + pname + " = " + string(summary.params.(char(pname)));
end
lines(end + 1, 1) = "Последние MSG:";
if isempty(summary.messages)
    lines(end + 1, 1) = "  MSG не найдены.";
else
    for idx = 1:numel(summary.messages)
        lines(end + 1, 1) = "  [" + string(summary.messages(idx).TimeUS) + "] " + string(summary.messages(idx).text);
    end
end
text_value = strjoin(lines, newline) + newline;
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
