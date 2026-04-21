%% DIAGNOSE_ARDUPILOT_ACCEL_CALIBRATION_PARAMS Диагностика параметров калибровки акселерометра.
% Назначение:
%   Поднимает устойчивый режим `ArduPilot JSON + MATLAB-модель` с профилем
%   TASK-20, считывает фактические параметры калибровки акселерометра через
%   MAVLink после загрузки и фиксирует признаки отсутствующей 3D-калибровки.
%
% Входы:
%   none
%
% Выходы:
%   task_21_accel_calibration_params - структура результата в base workspace
%
% Единицы измерения:
%   время - секунды;
%   параметры ArduPilot - в собственных единицах ArduPilot.
%
% Допущения:
%   `ArduPilot` собран внутри `WSL`, а базовый обмен TASK-20 остается
%   работоспособным с профилем одного акселерометра.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

log_path = fullfile(logs_dir, 'task_21_diagnose_accel_calibration_params.txt');
csv_path = fullfile(reports_dir, 'task_21_accel_calibration_params.csv');
mat_path = fullfile(reports_dir, 'task_21_accel_calibration_params.mat');
parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_single_accel_enable_mask.parm');

if ~isfile(parm_path)
    error('uav:task21:diagAccelCal:MissingParm', ...
        'Не найден профиль TASK-20: %s', parm_path);
end

cfg = uav.ardupilot.default_json_config();
baseline_mat_tmp = [tempname, '_task21_diag_accel.mat'];
baseline_csv_tmp = [tempname, '_task21_diag_accel.csv'];
cleanup_tmp = onCleanup(@() local_cleanup_temp({baseline_mat_tmp, baseline_csv_tmp})); %#ok<NASGU>

local_assign_base('ardupilot_task15_extra_defaults_win_path', parm_path);
local_assign_base('ardupilot_task15_baseline_diag_log_path', fullfile(logs_dir, 'task_21_diag_accel_baseline.txt'));
local_assign_base('ardupilot_task15_baseline_wait_log_path', fullfile(logs_dir, 'task_21_diag_accel_wait.txt'));
local_assign_base('ardupilot_task15_baseline_handshake_log_path', fullfile(logs_dir, 'task_21_diag_accel_handshake.txt'));
local_assign_base('ardupilot_task15_baseline_live_log_path', fullfile(logs_dir, 'task_21_diag_accel_live.txt'));
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_live_backend_duration_s', 25.0);
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
    uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(cfg.udp_remote_ip), cfg.mavlink_udp_port);
    error('uav:task21:diagAccelCal:BaselineFailed', ...
        'Не удалось восстановить базовый обмен перед диагностикой параметров. Причина: %s', ...
        char(string(baseline.first_failure_reason)));
end

fetch_result = uav.ardupilot.fetch_mavlink_params(cfg, "tcp:127.0.0.1:5763", 'FetchAll', true, 'Timeout_s', 25.0);
uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);

[param_table, missing_names, suspicious_names] = local_make_param_table(fetch_result);

result = struct();
result.parm_path = string(parm_path);
result.baseline = baseline;
result.fetch_result = fetch_result;
result.param_table = param_table;
result.missing_names = missing_names;
result.suspicious_names = suspicious_names;
result.required_fields_present = ~any(ismember(["INS_ENABLE_MASK","INS_USE","INS_USE2","INS_USE3","AHRS_ORIENTATION","ARMING_CHECK","LOG_DISARMED"], missing_names));
result.signs_calibration_not_done = local_detect_not_calibrated(param_table);

writetable(param_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_21_accel_calibration_params', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('TASK-21: диагностика параметров калибровки акселерометра\n');
fprintf('  baseline restored                      : %s\n', local_bool_text(baseline.baseline_restored));
fprintf('  valid_rx_count                         : %d\n', baseline.metrics.valid_rx_count);
fprintf('  response_tx_count                      : %d\n', baseline.metrics.response_tx_count);
fprintf('  fetched params                         : %d\n', height(param_table));
fprintf('  missing requested names                : %s\n', char(local_join_or_none(missing_names)));
fprintf('  calibration signs missing              : %s\n', local_bool_text(result.signs_calibration_not_done));

function [param_table, missing_names, suspicious_names] = local_make_param_table(fetch_result)
expected_names = [ ...
    "INS_ENABLE_MASK"; "INS_USE"; "INS_USE2"; "INS_USE3"; ...
    "INS_ACC_ID"; "INS_ACC2_ID"; "INS_ACC3_ID"; ...
    "INS_ACCOFFS_X"; "INS_ACCOFFS_Y"; "INS_ACCOFFS_Z"; ...
    "INS_ACCSCAL_X"; "INS_ACCSCAL_Y"; "INS_ACCSCAL_Z"; ...
    "INS_ACC2OFFS_X"; "INS_ACC2OFFS_Y"; "INS_ACC2OFFS_Z"; ...
    "INS_ACC2SCAL_X"; "INS_ACC2SCAL_Y"; "INS_ACC2SCAL_Z"; ...
    "AHRS_ORIENTATION"; "ARMING_CHECK"; "LOG_DISARMED"];

actual_names = string(fieldnames(fetch_result.param_values));
selected_names = actual_names(startsWith(actual_names, "INS_ACC") ...
    | startsWith(actual_names, "INS_USE") ...
    | startsWith(actual_names, "INS_ENABLE") ...
    | startsWith(actual_names, "AHRS_") ...
    | startsWith(actual_names, "ARMING_") ...
    | startsWith(actual_names, "LOG_"));
selected_names = union(selected_names, expected_names, 'stable');

rows = repmat(struct( ...
    'param_name', "", ...
    'param_value', nan, ...
    'is_present', false, ...
    'is_expected', false, ...
    'status_note', ""), numel(selected_names), 1);

for idx = 1:numel(selected_names)
    name = selected_names(idx);
    rows(idx).param_name = name;
    rows(idx).is_expected = any(expected_names == name);
    if isfield(fetch_result.param_values, name)
        rows(idx).is_present = true;
        value = fetch_result.param_values.(char(name));
        if isempty(value)
            rows(idx).param_value = nan;
        else
            rows(idx).param_value = double(value);
        end
    end
    rows(idx).status_note = local_param_note(name, rows(idx).param_value, rows(idx).is_present);
end

param_table = struct2table(rows);
param_table = sortrows(param_table, {'is_expected', 'param_name'}, {'descend', 'ascend'});
missing_names = param_table.param_name(param_table.is_expected & ~param_table.is_present);
 suspicious_names = param_table.param_name(param_table.status_note ~= "");
end

function flag = local_detect_not_calibrated(param_table)
flag = false;
if any(param_table.is_expected & ~param_table.is_present)
    flag = true;
    return;
end

mask = startsWith(param_table.param_name, "INS_ACCOFFS_") | startsWith(param_table.param_name, "INS_ACCSCAL_");
values = param_table.param_value(mask);
if any(~isfinite(values)) || any(values == 0)
    flag = true;
    return;
end

temp_rows = startsWith(param_table.param_name, "INS_ACC") & contains(param_table.param_name, "CALTEMP");
if any(temp_rows)
    temps = param_table.param_value(temp_rows);
    if any(temps <= -299)
        flag = true;
    end
end
end

function note = local_param_note(name, value, is_present)
if ~is_present
    note = "параметр не найден";
    return;
end

if ~isfinite(value)
    note = "значение не является конечным";
    return;
end

if startsWith(name, "INS_ACCSCAL_") && abs(value) < 1.0e-9
    note = "масштаб акселерометра равен нулю";
    return;
end

if startsWith(name, "INS_ACCOFFS_") && abs(value) < 1.0e-12
    note = "смещение акселерометра нулевое";
    return;
end

if contains(name, "CALTEMP") && value <= -299
    note = "температурная калибровка не выполнялась";
    return;
end

if endsWith(name, "_ID") && value == 0
    note = "экземпляр датчика не активирован";
    return;
end

note = "";
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
lines(end + 1, 1) = "TASK-21: диагностика параметров калибровки акселерометра";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Профиль TASK-20: " + result.parm_path;
lines(end + 1, 1) = "Baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "Baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "Baseline last_frame_count: " + string(result.baseline.metrics.last_frame_count);
lines(end + 1, 1) = "Параметры считаны: " + string(height(result.param_table));
lines(end + 1, 1) = "Heartbeat по tcp:5763: " + local_bool_text(result.fetch_result.heartbeat_received);
lines(end + 1, 1) = "Признаки отсутствующей 3D-калибровки: " + local_bool_text(result.signs_calibration_not_done);
lines(end + 1, 1) = "Отсутствующие параметры: " + local_join_or_none(result.missing_names);
lines(end + 1, 1) = "Подозрительные параметры: " + local_join_or_none(result.suspicious_names);
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Параметры:";
for idx = 1:height(result.param_table)
    row = result.param_table(idx, :);
    note = string(row.status_note);
    if strlength(note) == 0
        note = "-";
    end
    lines(end + 1, 1) = "  " + row.param_name + " = " + string(row.param_value) + " [" + note + "]";
end
if strlength(result.fetch_result.failure_reason) > 0
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "Ошибка чтения параметров: " + result.fetch_result.failure_reason;
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

function text_value = local_join_or_none(values)
values = string(values(:));
values(values == "") = [];
if isempty(values)
    text_value = "нет";
else
    text_value = strjoin(values, ", ");
end
end
