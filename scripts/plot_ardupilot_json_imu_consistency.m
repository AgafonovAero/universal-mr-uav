%% PLOT_ARDUPILOT_JSON_IMU_CONSISTENCY Построить графики TASK-19 по JSON-диагностике.
% Назначение:
%   Читает сохраненные MAT-результаты TASK-19 и строит графики по
%   компонентам accel_body, норме ускорения, норме кватерниона, исходам arm
%   по вариантам accel_body и временным историям ШИМ/команд винтов.
%
% Входы:
%   none
%
% Выходы:
%   task_19_json_imu_consistency_plots - структура путей сохраненных рисунков
%
% Единицы измерения:
%   время - секунды;
%   ускорение - м/с^2;
%   команды винтов - рад/с
%
% Допущения:
%   Сценарии диагностики TASK-19 уже были выполнены и сохранили MAT-файлы.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

if ~isfolder(figures_dir)
    mkdir(figures_dir);
end

imu_mat = fullfile(reports_dir, 'task_19_json_imu_consistency.mat');
accel_mat = fullfile(reports_dir, 'task_19_accel_convention_experiments.mat');

if ~isfile(imu_mat)
    error('uav:task19:plots:MissingImuMat', ...
        'Не найден файл результата %s.', imu_mat);
end

imu_loaded = load(imu_mat);
if ~isfield(imu_loaded, 'result')
    error('uav:task19:plots:MissingImuResult', ...
        'В файле %s отсутствует переменная result.', imu_mat);
end
imu_result = imu_loaded.result;
frame_table = imu_result.frame_table;
live_result = imu_result.live_result;

accel_summary = struct();
if isfile(accel_mat)
    accel_loaded = load(accel_mat);
    if isfield(accel_loaded, 'summary')
        accel_summary = accel_loaded.summary;
    end
end

paths = struct();
paths.accel_body = fullfile(figures_dir, 'task_19_accel_body_components.png');
paths.accel_norm = fullfile(figures_dir, 'task_19_accel_norm.png');
paths.quaternion_norm = fullfile(figures_dir, 'task_19_quaternion_norm.png');
paths.arm_by_mode = fullfile(figures_dir, 'task_19_arm_result_by_accel_mode.png');
paths.pwm_after_fix = fullfile(figures_dir, 'task_19_pwm_after_fix.png');
paths.motor_after_fix = fullfile(figures_dir, 'task_19_motor_commands_after_fix.png');

local_save_figure(paths.accel_body, @() local_plot_accel_components(frame_table));
local_save_figure(paths.accel_norm, @() local_plot_accel_norm(frame_table));
local_save_figure(paths.quaternion_norm, @() local_plot_quaternion_norm(frame_table));
local_save_figure(paths.arm_by_mode, @() local_plot_arm_by_mode(accel_summary));
local_save_figure(paths.pwm_after_fix, @() local_plot_pwm(live_result, imu_result.arm_attempt));
local_save_figure(paths.motor_after_fix, @() local_plot_motor_commands(live_result, imu_result.arm_attempt));

assignin('base', 'task_19_json_imu_consistency_plots', paths);

fprintf('Графики TASK-19 сохранены\n');
fprintf('  accel_body_components : %s\n', paths.accel_body);
fprintf('  accel_norm            : %s\n', paths.accel_norm);
fprintf('  quaternion_norm       : %s\n', paths.quaternion_norm);
fprintf('  arm_result_by_mode    : %s\n', paths.arm_by_mode);
fprintf('  pwm_after_fix         : %s\n', paths.pwm_after_fix);
fprintf('  motor_commands        : %s\n', paths.motor_after_fix);

function local_plot_accel_components(frame_table)
%LOCAL_PLOT_ACCEL_COMPONENTS Построить график компонент accel_body.

plot(frame_table.json_timestamp_s, frame_table.accel_x, 'LineWidth', 1.2);
hold on;
plot(frame_table.json_timestamp_s, frame_table.accel_y, 'LineWidth', 1.2);
plot(frame_table.json_timestamp_s, frame_table.accel_z, 'LineWidth', 1.2);
hold off;
grid on;
xlabel('timestamp, с');
ylabel('accel\_body, м/с^2');
title('Компоненты accel\_body первых JSON-кадров');
legend({'ax', 'ay', 'az'}, 'Location', 'best');
end

function local_plot_accel_norm(frame_table)
%LOCAL_PLOT_ACCEL_NORM Построить график нормы accel_body.

plot(frame_table.json_timestamp_s, frame_table.accel_norm, 'LineWidth', 1.4);
grid on;
xlabel('timestamp, с');
ylabel('||accel\_body||, м/с^2');
title('Норма accel\_body первых JSON-кадров');
end

function local_plot_quaternion_norm(frame_table)
%LOCAL_PLOT_QUATERNION_NORM Построить график нормы quaternion.

plot(frame_table.json_timestamp_s, frame_table.quat_norm, 'LineWidth', 1.4);
grid on;
xlabel('timestamp, с');
ylabel('||quaternion||');
title('Норма quaternion первых JSON-кадров');
end

function local_plot_arm_by_mode(accel_summary)
%LOCAL_PLOT_ARM_BY_MODE Построить график исходов arm по вариантам accel_body.

if ~isstruct(accel_summary) || ~isfield(accel_summary, 'cases') || isempty(accel_summary.cases)
    text(0.5, 0.5, 'Данные опытов accel\_body отсутствуют', ...
        'HorizontalAlignment', 'center');
    axis off;
    return;
end

cases = accel_summary.cases;
mode_names = strings(numel(cases), 1);
ack_values = nan(numel(cases), 1);
arm_values = zeros(numel(cases), 1);

for idx = 1:numel(cases)
    mode_names(idx) = string(cases(idx).accel_mode);
    ack_values(idx) = double(cases(idx).ack_result);
    arm_values(idx) = double(logical(cases(idx).arm_succeeded));
end

yyaxis left;
bar(categorical(mode_names), ack_values);
ylabel('COMMAND\_ACK');
yyaxis right;
plot(categorical(mode_names), arm_values, 'o-', 'LineWidth', 1.4, 'MarkerSize', 6);
ylabel('arm succeeded');
ylim([-0.1, 1.1]);
grid on;
title('Результат arm по вариантам accel\_body');
end

function local_plot_pwm(live_result, arm_attempt)
%LOCAL_PLOT_PWM Построить график четырех каналов ШИМ.

sample_count = numel(live_result.time_s);
pwm_us = nan(sample_count, 4);
for idx = 1:min(sample_count, numel(live_result.sitl_output))
    pwm_vec = double(live_result.sitl_output(idx).motor_pwm_us(:));
    if numel(pwm_vec) >= 4
        pwm_us(idx, :) = pwm_vec(1:4).';
    end
end

plot(live_result.time_s, pwm_us(:, 1), 'LineWidth', 1.1);
hold on;
plot(live_result.time_s, pwm_us(:, 2), 'LineWidth', 1.1);
plot(live_result.time_s, pwm_us(:, 3), 'LineWidth', 1.1);
plot(live_result.time_s, pwm_us(:, 4), 'LineWidth', 1.1);
hold off;
grid on;
xlabel('t, с');
ylabel('ШИМ, мкс');
title("ШИМ после попытки arm, результат: " + string(local_bool_text(arm_attempt.arm_succeeded)));
legend({'PWM1', 'PWM2', 'PWM3', 'PWM4'}, 'Location', 'best');
end

function local_plot_motor_commands(live_result, arm_attempt)
%LOCAL_PLOT_MOTOR_COMMANDS Построить график команд частоты вращения винтов.

motor_cmd = double(live_result.motor_cmd_radps);
plot(live_result.time_s, motor_cmd(:, 1), 'LineWidth', 1.1);
hold on;
plot(live_result.time_s, motor_cmd(:, 2), 'LineWidth', 1.1);
plot(live_result.time_s, motor_cmd(:, 3), 'LineWidth', 1.1);
plot(live_result.time_s, motor_cmd(:, 4), 'LineWidth', 1.1);
hold off;
grid on;
xlabel('t, с');
ylabel('Команда винта, рад/с');
title("Команды винтов после попытки arm, результат: " + string(local_bool_text(arm_attempt.arm_succeeded)));
legend({'M1', 'M2', 'M3', 'M4'}, 'Location', 'best');
end

function local_save_figure(path_value, plot_fun)
%LOCAL_SAVE_FIGURE Построить и сохранить рисунок.

fig = figure('Visible', 'off');
cleanup_obj = onCleanup(@() close(fig)); %#ok<NASGU>
plot_fun();
exportgraphics(fig, path_value, 'Resolution', 150);
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русский текст.

if logical(flag_value)
    text_value = 'да';
else
    text_value = 'нет';
end
end
