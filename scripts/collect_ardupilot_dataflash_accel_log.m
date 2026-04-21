%% COLLECT_ARDUPILOT_DATAFLASH_ACCEL_LOG Собрать DataFlash-журнал ArduPilot.
% Назначение:
%   Запускает режим `ArduPilot JSON + MATLAB-модель`, включает запись
%   журнала в разоруженном состоянии, выдерживает не менее 60 секунд
%   устойчивого обмена, выполняет попытку взведения и копирует последний
%   DataFlash-журнал `ArduPilot` из `WSL` в артефакты репозитория.
%
% Входы:
%   none
%
% Выходы:
%   task_20_collect_dataflash_accel_log - структура результата в base
%   workspace
%
% Единицы измерения:
%   время - секунды;
%   частота - герцы;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   `ArduPilot` установлен в `WSL`, а `pymavlink` доступен в виртуальной
%   среде `/home/oaleg/venv-ardupilot/bin/python3`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_20_collect_dataflash_accel_log.txt');
bin_copy_path = fullfile(reports_dir, 'task_20_ardupilot_dataflash.bin');
baseline_diag_log = fullfile(logs_dir, 'task_20_dataflash_baseline.txt');
baseline_wait_log = fullfile(logs_dir, 'task_20_dataflash_wait.txt');
baseline_handshake_log = fullfile(logs_dir, 'task_20_dataflash_handshake.txt');
baseline_live_log = fullfile(logs_dir, 'task_20_dataflash_live_backend.txt');
baseline_mat_tmp = [tempname, '_task20_dataflash_baseline.mat'];
baseline_csv_tmp = [tempname, '_task20_dataflash_baseline.csv'];
arm_mat_tmp = [tempname, '_task20_dataflash_arm.mat'];
arm_csv_tmp = [tempname, '_task20_dataflash_arm.csv'];

cfg = uav.ardupilot.default_json_config();
logging_parm_path = fullfile(repo_root, 'tools', 'ardupilot', 'wsl', 'task20_dataflash_logging.parm');

if ~isfile(logging_parm_path)
    error('uav:task20:dataflash:MissingParm', ...
        'Не найден файл параметров журналирования: %s', ...
        logging_parm_path);
end

cleanup_tmp = onCleanup(@() local_cleanup_temp({ ...
    baseline_mat_tmp, baseline_csv_tmp, arm_mat_tmp, arm_csv_tmp})); %#ok<NASGU>

local_assign_base('ardupilot_task15_extra_defaults_win_path', logging_parm_path);
local_assign_base('ardupilot_task15_baseline_diag_log_path', baseline_diag_log);
local_assign_base('ardupilot_task15_baseline_wait_log_path', baseline_wait_log);
local_assign_base('ardupilot_task15_baseline_handshake_log_path', baseline_handshake_log);
local_assign_base('ardupilot_task15_baseline_live_log_path', baseline_live_log);
local_assign_base('ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
local_assign_base('ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
local_assign_base('ardupilot_arm_pwm_response_mat_path', arm_mat_tmp);
local_assign_base('ardupilot_arm_pwm_response_csv_path', arm_csv_tmp);
local_assign_base('ardupilot_live_backend_duration_s', 60.0);
local_assign_base('ardupilot_arm_attempt_delay_s', 30.0);
cleanup_base = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_task15_extra_defaults_win_path'', ' ...
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

preexisting_logs = local_list_dataflash_logs(cfg);

run(fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m'));
baseline = evalin('base', 'task_17_task15_baseline_exchange');

response = struct();
if baseline.baseline_restored
    local_assign_base('ardupilot_live_backend_duration_s', 60.0);
    local_assign_base('ardupilot_arm_attempt_delay_s', 30.0);
    run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
    response = evalin('base', 'ardupilot_arm_pwm_response');
end

uav.ardupilot.stop_existing_sitl(cfg.wsl_distro_name, string(baseline.cfg.udp_remote_ip), cfg.mavlink_udp_port);
pause(2.0);

[copy_ok, latest_wsl_path, candidate_list, copy_output] = local_copy_latest_dataflash(cfg, bin_copy_path);

result = struct();
result.logging_parm_path = string(logging_parm_path);
result.preexisting_logs = preexisting_logs;
result.baseline = baseline;
result.arm_response = response;
result.dataflash_found = logical(copy_ok);
result.dataflash_wsl_path = string(latest_wsl_path);
result.dataflash_copy_path = string(bin_copy_path);
result.dataflash_candidate_list = candidate_list;
result.dataflash_copy_output = string(copy_output);
result.first_failure_reason = local_pick_failure_reason(baseline, response, copy_ok, latest_wsl_path);

assignin('base', 'task_20_collect_dataflash_accel_log', result);
save(fullfile(reports_dir, 'task_20_collect_dataflash_accel_log.mat'), 'result');
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('Сбор DataFlash-журнала TASK-20\n');
fprintf('  baseline restored                      : %s\n', local_bool_text(baseline.baseline_restored));
fprintf('  arm attempted                          : %s\n', local_bool_text(~isempty(fieldnames(response))));
fprintf('  dataflash found                        : %s\n', local_bool_text(copy_ok));
fprintf('  dataflash path                         : %s\n', char(string(latest_wsl_path)));
fprintf('  first failure reason                   : %s\n', char(result.first_failure_reason));

if ~copy_ok
    error('uav:task20:dataflash:LogNotFound', ...
        'DataFlash-журнал не найден. %s', char(result.first_failure_reason));
end

function local_assign_base(name, value)
assignin('base', name, value);
end

function listing = local_list_dataflash_logs(cfg)
listing = strings(0, 1);
command = sprintf([ ...
    'wsl -d %s -- bash -lc ', ...
    '"cd %s/ArduCopter && ls -1t logs/*.BIN logs/*.bin 2>/dev/null | head -n 20"'], ...
    char(cfg.wsl_distro_name), ...
    char(cfg.ardupilot_root));
[~, output_text] = system(command);
items = splitlines(string(output_text));
items = strtrim(items);
items(items == "") = [];
listing = items;
end

function [copy_ok, latest_wsl_path, candidate_list, output_text] = local_copy_latest_dataflash(cfg, win_target_path)
copy_ok = false;
latest_wsl_path = "";
candidate_list = local_list_dataflash_logs(cfg);
output_text = "";

if isempty(candidate_list)
    return;
end

latest_wsl_path = candidate_list(1);
ardupilot_root_wsl = string(cfg.ardupilot_root);
if startsWith(ardupilot_root_wsl, "~/")
    ardupilot_root_wsl = "/home/oaleg/" + extractAfter(ardupilot_root_wsl, 2);
end
if ~startsWith(latest_wsl_path, "/")
    latest_wsl_path = ardupilot_root_wsl + "/ArduCopter/" + latest_wsl_path;
elseif startsWith(latest_wsl_path, "~/")
    latest_wsl_path = "/home/oaleg/" + extractAfter(latest_wsl_path, 2);
end
target_wsl_path = uav.ardupilot.windows_to_wsl_path(win_target_path);
target_dir_wsl = extractBefore(target_wsl_path, "/task_20_ardupilot_dataflash.bin");
command = sprintf([ ...
    'wsl -d %s -- bash -lc ', ...
    '"mkdir -p ''%s'' && cp ''%s'' ''%s''"'], ...
    char(cfg.wsl_distro_name), ...
    char(target_dir_wsl), ...
    char(latest_wsl_path), ...
    char(target_wsl_path));
[status_code, output_text] = system(command);
copy_ok = status_code == 0 && isfile(win_target_path);
end

function text_value = local_pick_failure_reason(baseline, response, copy_ok, latest_wsl_path)
if ~baseline.baseline_restored
    text_value = string(baseline.first_failure_reason);
    return;
end

if isempty(fieldnames(response))
    text_value = "Попытка arm не была выполнена после baseline-прогона.";
    return;
end

if ~copy_ok
    if strlength(string(latest_wsl_path)) > 0
        text_value = "Не удалось скопировать DataFlash-журнал из " + string(latest_wsl_path) + ".";
    else
        text_value = "DataFlash-журнал не найден в каталоге logs ArduPilot.";
    end
    return;
end

if isfield(response, 'arm_attempt') && isfield(response.arm_attempt, 'failure_reason')
    text_value = string(response.arm_attempt.failure_reason);
else
    text_value = "";
end
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
lines(end + 1, 1) = "TASK-20: сбор DataFlash-журнала по акселерометрам ArduPilot";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Файл параметров журналирования: " + string(result.logging_parm_path);
lines(end + 1, 1) = "Baseline restored: " + local_bool_text(result.baseline.baseline_restored);
lines(end + 1, 1) = "Baseline valid_rx_count: " + string(result.baseline.metrics.valid_rx_count);
lines(end + 1, 1) = "Baseline response_tx_count: " + string(result.baseline.metrics.response_tx_count);
lines(end + 1, 1) = "Baseline last_frame_count: " + string(result.baseline.metrics.last_frame_count);
if ~isempty(fieldnames(result.arm_response))
    lines(end + 1, 1) = "Arm attempted: да";
    lines(end + 1, 1) = "Arm succeeded: " + local_bool_text(result.arm_response.arm_attempt.arm_succeeded);
    lines(end + 1, 1) = "COMMAND_ACK: " + string(result.arm_response.arm_attempt.ack_result);
    lines(end + 1, 1) = "Failure reason: " + string(result.arm_response.arm_attempt.failure_reason);
else
    lines(end + 1, 1) = "Arm attempted: нет";
end
lines(end + 1, 1) = "DataFlash found: " + local_bool_text(result.dataflash_found);
lines(end + 1, 1) = "DataFlash WSL path: " + string(result.dataflash_wsl_path);
lines(end + 1, 1) = "DataFlash copy path: " + string(result.dataflash_copy_path);
lines(end + 1, 1) = "First failure reason: " + string(result.first_failure_reason);
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Последние найденные журналы ArduPilot:";
if isempty(result.dataflash_candidate_list)
    lines(end + 1, 1) = "  журналы не найдены";
else
    lines = [lines; "  " + result.dataflash_candidate_list(:)]; %#ok<AGROW>
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
