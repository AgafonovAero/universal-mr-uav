%% RUN_ARDUPILOT_JSON_MODEL_DEMO Запустить JSON-режим ArduPilot с MATLAB-моделью.
% Назначение:
%   Использует восстановленный базовый обмен TASK-15 как основу для
%   практического режима `ArduPilot SITL + MATLAB-модель + Mission Planner`.
%   Сценарий:
%   - поднимает или переиспользует `Mission Planner`;
%   - восстанавливает устойчивый `JSON/UDP`-обмен;
%   - выполняет попытку взведения после устойчивого обмена;
%   - сохраняет журналы, таблицу и рисунки.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_json_model_demo - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время - секунды;
%   высота - метры;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   Используются существующие средства TASK-15/TASK-17 без изменения
%   математической модели движения и без изменения моделей Simulink.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

local_prepare_parent(fullfile(logs_dir, 'dummy.txt'));
local_prepare_parent(fullfile(reports_dir, 'dummy.txt'));
local_prepare_parent(fullfile(figures_dir, 'dummy.txt'));

log_start = fullfile(logs_dir, 'task_18_json_model_start.txt');
log_exchange = fullfile(logs_dir, 'task_18_json_model_exchange.txt');
log_arm = fullfile(logs_dir, 'task_18_json_model_arm_attempt.txt');
log_statustext = fullfile(logs_dir, 'task_18_json_model_statustext.txt');
report_mat = fullfile(reports_dir, 'task_18_json_model_demo.mat');
report_csv = fullfile(reports_dir, 'task_18_json_model_demo.csv');
baseline_wait_log = fullfile(logs_dir, 'task_18_json_model_wait_for_packet.txt');
baseline_handshake_log = fullfile(logs_dir, 'task_18_json_model_handshake.txt');
baseline_live_log = fullfile(logs_dir, 'task_18_json_model_live_backend.txt');
baseline_debug_log = fullfile(logs_dir, 'task_18_json_model_baseline_exchange.txt');
baseline_mat_tmp = fullfile(tempdir, 'task_18_json_model_baseline_exchange.mat');
baseline_csv_tmp = fullfile(tempdir, 'task_18_json_model_baseline_exchange.csv');
arm_response_mat_tmp = fullfile(tempdir, 'task_18_json_model_arm_pwm_response.mat');
arm_response_csv_tmp = fullfile(tempdir, 'task_18_json_model_arm_pwm_response.csv');

fig_exchange = fullfile(figures_dir, 'task_18_json_exchange_counts.png');
fig_pwm = fullfile(figures_dir, 'task_18_json_pwm_channels.png');
fig_motor = fullfile(figures_dir, 'task_18_json_motor_commands.png');
fig_alt = fullfile(figures_dir, 'task_18_json_altitude.png');
fig_att = fullfile(figures_dir, 'task_18_json_attitude.png');

mp_status = local_launch_mission_planner();
local_write_utf8_text(log_start, "Состояние Mission Planner перед JSON-режимом" + newline + mp_status + newline);

baseline_script = fullfile(repo_root, 'scripts', 'run_ardupilot_task15_baseline_exchange.m');
assignin('base', 'ardupilot_task15_baseline_diag_log_path', baseline_debug_log);
assignin('base', 'ardupilot_task15_baseline_wait_log_path', baseline_wait_log);
assignin('base', 'ardupilot_task15_baseline_handshake_log_path', baseline_handshake_log);
assignin('base', 'ardupilot_task15_baseline_live_log_path', baseline_live_log);
assignin('base', 'ardupilot_task15_baseline_mat_report_path', baseline_mat_tmp);
assignin('base', 'ardupilot_task15_baseline_csv_report_path', baseline_csv_tmp);
assignin('base', 'ardupilot_arm_pwm_response_mat_path', arm_response_mat_tmp);
assignin('base', 'ardupilot_arm_pwm_response_csv_path', arm_response_csv_tmp);
cleanup_overrides = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_task15_baseline_diag_log_path'', ' ...
    '''ardupilot_task15_baseline_wait_log_path'', ' ...
    '''ardupilot_task15_baseline_handshake_log_path'', ' ...
    '''ardupilot_task15_baseline_live_log_path'', ' ...
    '''ardupilot_task15_baseline_mat_report_path'', ' ...
    '''ardupilot_task15_baseline_csv_report_path'', ' ...
    '''ardupilot_arm_pwm_response_mat_path'', ' ...
    '''ardupilot_arm_pwm_response_csv_path'');'])); %#ok<NASGU>
run(baseline_script);
baseline = evalin('base', 'task_17_task15_baseline_exchange');
baseline_metrics = baseline.metrics;

if ~baseline.baseline_restored
    result = struct();
    result.mission_planner_status = mp_status;
    result.baseline = baseline;
    result.baseline_metrics = baseline_metrics;
    result.json_metrics = baseline_metrics;
    result.arm_attempt = struct();
    result.arm_attempt.arm_succeeded = false;
    result.arm_attempt.failure_reason = string(baseline.first_failure_reason);
    result.arm_attempt.status_texts = strings(0, 1);
    result.arm_attempt.ack_result = nan;
    result.success = false;
    save(report_mat, 'result');
    local_write_utf8_text(log_exchange, local_make_exchange_log(baseline, baseline_metrics));
    local_write_utf8_text(log_arm, "Попытка arm не выполнялась: baseline JSON-обмен не восстановлен." + newline ...
        + "Причина: " + string(baseline.first_failure_reason) + newline);
    local_write_utf8_text(log_statustext, "STATUSTEXT не собирались, потому что baseline не восстановлен." + newline);
    writetable(table(string(baseline.first_failure_reason), 'VariableNames', {'failure_reason'}), report_csv);
    assignin('base', 'ardupilot_json_model_demo', result);
    error('uav:task18:jsondemo:BaselineFailed', ...
        'JSON baseline не восстановлен: %s', ...
        char(string(baseline.first_failure_reason)));
end

assignin('base', 'ardupilot_live_backend_duration_s', 40.0);
assignin('base', 'ardupilot_arm_attempt_delay_s', 30.0);
cleanup_overrides = onCleanup(@() evalin('base', [ ...
    'clear(' ...
    '''ardupilot_live_backend_duration_s'', ' ...
    '''ardupilot_arm_attempt_delay_s'');'])); %#ok<NASGU>

run(fullfile(repo_root, 'scripts', 'run_ardupilot_arm_pwm_response.m'));
response = evalin('base', 'ardupilot_arm_pwm_response');
json_metrics = uav.ardupilot.summarize_live_backend_metrics(response.live_result);
demo_table = local_make_demo_table(response.live_result);
writetable(demo_table, report_csv);

result = struct();
result.mission_planner_status = mp_status;
result.baseline = baseline;
result.baseline_metrics = baseline_metrics;
result.response = response;
result.json_metrics = json_metrics;
result.arm_attempt = response.arm_attempt;
result.before = response.before;
result.after = response.after;
result.success = json_metrics.valid_rx_count > 50 && json_metrics.response_tx_count > 50;

save(report_mat, 'result');
assignin('base', 'ardupilot_json_model_demo', result);

local_write_utf8_text(log_exchange, local_make_exchange_log(baseline, json_metrics));
local_write_utf8_text(log_arm, local_make_arm_log(response.arm_attempt, response.before, response.after));
local_write_utf8_text(log_statustext, local_make_statustext_log(response.arm_attempt));

local_save_figure(fig_exchange, @() local_plot_exchange_counts(response.live_result));
local_save_figure(fig_pwm, @() local_plot_pwm_channels(response.live_result));
local_save_figure(fig_motor, @() local_plot_motor_commands(response.live_result));
local_save_figure(fig_alt, @() local_plot_altitude(response.live_result));
local_save_figure(fig_att, @() local_plot_attitude(response.live_result));

fprintf('JSON-режим ArduPilot SITL с MATLAB-моделью\n');
fprintf('  baseline valid_rx_count                 : %d\n', baseline_metrics.valid_rx_count);
fprintf('  baseline response_tx_count              : %d\n', baseline_metrics.response_tx_count);
fprintf('  JSON valid_rx_count                     : %d\n', json_metrics.valid_rx_count);
fprintf('  JSON response_tx_count                  : %d\n', json_metrics.response_tx_count);
fprintf('  Mission Planner запущен                 : %s\n', local_bool_text(contains(mp_status, "Mission Planner запущен: да")));
fprintf('  попытка arm выполнена                   : да\n');
fprintf('  arm выполнен                            : %s\n', local_bool_text(response.arm_attempt.arm_succeeded));
fprintf('  код ACK                                 : %.0f\n', response.arm_attempt.ack_result);
fprintf('  причина                                 : %s\n', char(response.arm_attempt.failure_reason));

function mp_status = local_launch_mission_planner()
%LOCAL_LAUNCH_MISSION_PLANNER Запустить или переиспользовать Mission Planner.

process_command = ['powershell -NoProfile -NonInteractive -Command "' ...
    '$mp = Get-Process -Name MissionPlanner -ErrorAction SilentlyContinue | Select-Object -First 1; ' ...
    'if ($null -eq $mp) { ' ...
    '$candidates = @(' ...
    '''C:\Program Files (x86)\Mission Planner\MissionPlanner.exe'', ' ...
    '''C:\Program Files\Mission Planner\MissionPlanner.exe'', ' ...
    '''$env:LOCALAPPDATA\Mission Planner\MissionPlanner.exe''); ' ...
    '$path = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1; ' ...
    'if ($path) { $mp = Start-Process -FilePath $path -PassThru; Start-Sleep -Seconds 3 } }; ' ...
    '$udp = Get-NetUDPEndpoint -ErrorAction SilentlyContinue | Where-Object { $_.LocalPort -eq 14550 } | Select-Object -First 1; ' ...
    'Write-Output (' ...
    '''Mission Planner запущен: '' + $(if ($mp) { ''да'' } else { ''нет'' })); ' ...
    'if ($mp) { Write-Output (''PID: '' + $mp.Id) }; ' ...
    'Write-Output (''Порт UDP 14550 занят: '' + $(if ($udp) { ''да'' } else { ''нет'' })); ' ...
    'if ($udp) { Write-Output (''PID владельца порта 14550: '' + $udp.OwningProcess) }"'];
[~, output_text] = system(process_command);
mp_status = string(strtrim(output_text));
end

function table_value = local_make_demo_table(live_result)
%LOCAL_MAKE_DEMO_TABLE Сформировать табличный отчет JSON-режима.

time_s = double(live_result.time_s(:));
sample_count = numel(time_s);

pwm_us = nan(sample_count, 4);
frame_count = nan(sample_count, 1);
valid_flags = false(sample_count, 1);
for idx = 1:sample_count
    if idx <= numel(live_result.sitl_output)
        item = live_result.sitl_output(idx);
        valid_flags(idx) = logical(item.valid);
        frame_count(idx) = double(item.frame_count);
        motor_pwm_us = double(item.motor_pwm_us(:));
        if numel(motor_pwm_us) >= 4
            pwm_us(idx, :) = motor_pwm_us(1:4).';
        end
    end
end

motor_cmd = double(live_result.motor_cmd_radps);
altitude_true_m = nan(sample_count, 1);
altitude_est_m = nan(sample_count, 1);
euler_true_rad = nan(sample_count, 3);
euler_est_rad = nan(sample_count, 3);

for idx = 1:sample_count
    state_k = live_result.state(idx);
    est_k = live_result.estimator(idx);
    altitude_true_m(idx) = -double(state_k.p_ned_m(3));
    altitude_est_m(idx) = double(est_k.alt_m);
    euler_true_rad(idx, :) = local_quat_to_euler321(double(state_k.q_nb(:))).';
    euler_est_rad(idx, :) = double(est_k.euler_rpy_rad(:)).';
end

table_value = table( ...
    time_s, ...
    frame_count, ...
    valid_flags, ...
    pwm_us(:, 1), pwm_us(:, 2), pwm_us(:, 3), pwm_us(:, 4), ...
    motor_cmd(:, 1), motor_cmd(:, 2), motor_cmd(:, 3), motor_cmd(:, 4), ...
    altitude_true_m, altitude_est_m, ...
    euler_true_rad(:, 1), euler_true_rad(:, 2), euler_true_rad(:, 3), ...
    euler_est_rad(:, 1), euler_est_rad(:, 2), euler_est_rad(:, 3), ...
    double(live_result.udp_valid_rx_count_hist(:)), ...
    double(live_result.udp_invalid_rx_count_hist(:)), ...
    double(live_result.udp_rx_datagram_count_hist(:)), ...
    'VariableNames', { ...
        'time_s', ...
        'frame_count', ...
        'sitl_packet_valid', ...
        'pwm1_us', 'pwm2_us', 'pwm3_us', 'pwm4_us', ...
        'motor1_radps', 'motor2_radps', 'motor3_radps', 'motor4_radps', ...
        'altitude_true_m', 'altitude_est_m', ...
        'roll_true_rad', 'pitch_true_rad', 'yaw_true_rad', ...
        'roll_est_rad', 'pitch_est_rad', 'yaw_est_rad', ...
        'udp_valid_rx_count', ...
        'udp_invalid_rx_count', ...
        'udp_rx_datagram_count'});
end

function text_value = local_make_exchange_log(baseline, metrics)
%LOCAL_MAKE_EXCHANGE_LOG Сформировать журнал обмена JSON-режима.

lines = strings(0, 1);
lines(end + 1, 1) = "JSON-режим ArduPilot SITL с MATLAB-моделью";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Команда запуска ArduPilot: " + string(baseline.launch_command);
lines(end + 1, 1) = "PID arducopter после baseline: " + string(baseline.process_after_launch.pid);
lines(end + 1, 1) = "Процесс жив после baseline: " + local_bool_text(baseline.process_after_live.is_alive);
lines(end + 1, 1) = "valid_rx_count: " + string(metrics.valid_rx_count);
lines(end + 1, 1) = "json_tx_count: " + string(metrics.json_tx_count);
lines(end + 1, 1) = "response_tx_count: " + string(metrics.response_tx_count);
lines(end + 1, 1) = "last_frame_count: " + string(metrics.last_frame_count);
lines(end + 1, 1) = "Средняя частота valid_rx [Hz]: " + sprintf('%.4f', metrics.valid_rx_rate_hz);
lines(end + 1, 1) = "Средняя частота response_tx [Hz]: " + sprintf('%.4f', metrics.response_tx_rate_hz);
lines(end + 1, 1) = "95-процентиль периода response_tx [s]: " + sprintf('%.6f', metrics.response_tx_period_p95_s);
lines(end + 1, 1) = "Последний sender address: " + string(metrics.last_sender_address);
lines(end + 1, 1) = "Последний sender port: " + string(metrics.last_sender_port);
lines(end + 1, 1) = "Последний статус обмена: " + string(metrics.last_exchange_status);
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_make_arm_log(arm_attempt, before_result, after_result)
%LOCAL_MAKE_ARM_LOG Сформировать журнал попытки взведения.

lines = strings(0, 1);
lines(end + 1, 1) = "Попытка взведения в JSON-режиме";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "Метод: " + string(arm_attempt.method);
lines(end + 1, 1) = "HEARTBEAT по tcp:5763: " + local_bool_text(arm_attempt.heartbeat_received);
lines(end + 1, 1) = "Взведение выполнено: " + local_bool_text(arm_attempt.arm_succeeded);
lines(end + 1, 1) = "ACK: " + string(arm_attempt.ack_result);
lines(end + 1, 1) = "Причина: " + string(arm_attempt.failure_reason);
lines(end + 1, 1) = "ШИМ до попытки [us]: [" + local_format_vector(before_result.last_pwm_us) + "]";
lines(end + 1, 1) = "ШИМ после попытки [us]: [" + local_format_vector(after_result.last_pwm_us) + "]";
lines(end + 1, 1) = "Команды винтов до попытки [rad/s]: [" + local_format_vector(before_result.last_motor_cmd_radps) + "]";
lines(end + 1, 1) = "Команды винтов после попытки [rad/s]: [" + local_format_vector(after_result.last_motor_cmd_radps) + "]";
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_make_statustext_log(arm_attempt)
%LOCAL_MAKE_STATUSTEXT_LOG Сформировать журнал STATUSTEXT.

lines = strings(0, 1);
lines(end + 1, 1) = "STATUSTEXT JSON-режима при попытке взведения";
lines(end + 1, 1) = "============================================================";
if isempty(arm_attempt.status_texts)
    lines(end + 1, 1) = "Сообщения STATUSTEXT не получены.";
else
    lines = [lines; arm_attempt.status_texts(:)]; %#ok<AGROW>
end
text_value = strjoin(lines, newline) + newline;
end

function local_plot_exchange_counts(live_result)
%LOCAL_PLOT_EXCHANGE_COUNTS Построить график счетчиков обмена.

plot(live_result.time_s, double(live_result.udp_valid_rx_count_hist), 'LineWidth', 1.5);
hold on;
plot(live_result.time_s, double(live_result.udp_rx_datagram_count_hist), 'LineWidth', 1.5);
plot(live_result.time_s, double(live_result.udp_invalid_rx_count_hist), 'LineWidth', 1.5);
hold off;
grid on;
xlabel('t, c');
ylabel('Счетчик');
title('JSON-режим: накопление счетчиков обмена');
legend({'valid\_rx', 'udp\_rx', 'invalid\_rx'}, 'Location', 'best');
end

function local_plot_pwm_channels(live_result)
%LOCAL_PLOT_PWM_CHANNELS Построить график ШИМ по каналам.

pwm_matrix = local_collect_pwm_matrix(live_result.sitl_output);
plot(live_result.time_s, pwm_matrix, 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('ШИМ, мкс');
title('JSON-режим: каналы ШИМ');
legend({'PWM1', 'PWM2', 'PWM3', 'PWM4'}, 'Location', 'best');
end

function local_plot_motor_commands(live_result)
%LOCAL_PLOT_MOTOR_COMMANDS Построить график команд винтов.

plot(live_result.time_s, double(live_result.motor_cmd_radps), 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('omega, рад/с');
title('JSON-режим: команды частоты вращения винтов');
legend({'M1', 'M2', 'M3', 'M4'}, 'Location', 'best');
end

function local_plot_altitude(live_result)
%LOCAL_PLOT_ALTITUDE Построить график высоты.

sample_count = numel(live_result.time_s);
altitude_true_m = nan(sample_count, 1);
altitude_est_m = nan(sample_count, 1);
for idx = 1:sample_count
    altitude_true_m(idx) = -double(live_result.state(idx).p_ned_m(3));
    altitude_est_m(idx) = double(live_result.estimator(idx).alt_m);
end
plot(live_result.time_s, altitude_true_m, 'LineWidth', 1.5);
hold on;
plot(live_result.time_s, altitude_est_m, 'LineWidth', 1.5);
hold off;
grid on;
xlabel('t, c');
ylabel('Высота, м');
title('JSON-режим: истинная и оцененная высота');
legend({'Истинная', 'Оцененная'}, 'Location', 'best');
end

function local_plot_attitude(live_result)
%LOCAL_PLOT_ATTITUDE Построить график углов ориентации.

sample_count = numel(live_result.time_s);
euler_true = nan(sample_count, 3);
euler_est = nan(sample_count, 3);
for idx = 1:sample_count
    euler_true(idx, :) = local_quat_to_euler321(double(live_result.state(idx).q_nb(:))).';
    euler_est(idx, :) = double(live_result.estimator(idx).euler_rpy_rad(:)).';
end

tiledlayout(3, 1);
labels = {'Крен', 'Тангаж', 'Рыскание'};
for axis_index = 1:3
    nexttile;
    plot(live_result.time_s, euler_true(:, axis_index), 'LineWidth', 1.2);
    hold on;
    plot(live_result.time_s, euler_est(:, axis_index), 'LineWidth', 1.2);
    hold off;
    grid on;
    ylabel(labels{axis_index} + ", рад");
    if axis_index == 1
        title('JSON-режим: углы ориентации');
    end
    if axis_index == 3
        xlabel('t, c');
    end
end
end

function pwm_matrix = local_collect_pwm_matrix(sitl_output)
%LOCAL_COLLECT_PWM_MATRIX Собрать историю ШИМ из массива sitl_output.

sample_count = numel(sitl_output);
pwm_matrix = nan(sample_count, 4);
for idx = 1:sample_count
    motor_pwm_us = double(sitl_output(idx).motor_pwm_us(:));
    if numel(motor_pwm_us) >= 4
        pwm_matrix(idx, :) = motor_pwm_us(1:4).';
    end
end
end

function euler_rpy_rad = local_quat_to_euler321(q_nb)
%LOCAL_QUAT_TO_EULER321 Преобразовать кватернион в углы 3-2-1.

q_nb = double(q_nb(:));
q_nb = q_nb / norm(q_nb);
q0 = q_nb(1);
q1 = q_nb(2);
q2 = q_nb(3);
q3 = q_nb(4);

roll_rad = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1^2 + q2^2));
pitch_rad = asin(max(-1, min(1, 2 * (q0 * q2 - q3 * q1))));
yaw_rad = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2^2 + q3^2));
euler_rpy_rad = [roll_rad; pitch_rad; yaw_rad];
end

function local_save_figure(path_value, plotter)
%LOCAL_SAVE_FIGURE Построить и сохранить рисунок.

fig = figure('Visible', 'off');
cleanup_obj = onCleanup(@() close(fig)); %#ok<NASGU>
plotter();
exportgraphics(fig, path_value, 'Resolution', 150);
end

function local_prepare_parent(path_value)
%LOCAL_PREPARE_PARENT Создать родительский каталог файла.

folder_path = fileparts(path_value);
if ~isfolder(folder_path)
    mkdir(folder_path);
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

local_prepare_parent(path_value);
text_value = string(text_value);
text_value(ismissing(text_value)) = "";
if numel(text_value) > 1
    text_value = strjoin(text_value, newline);
end
fid = fopen(path_value, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:task18:jsondemo:OpenLog', ...
        'Не удалось открыть файл %s.', path_value);
end
cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function text_value = local_format_vector(vec_value)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку вектора.

vec_value = double(vec_value(:));
text_value = strtrim(sprintf('%.6f ', vec_value));
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end
