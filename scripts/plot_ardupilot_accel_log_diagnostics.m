%% PLOT_ARDUPILOT_ACCEL_LOG_DIAGNOSTICS Построить графики TASK-20.
% Назначение:
%   По результатам диагностики `DataFlash`, сравнению JSON-ускорений и
%   повторной попытке `arm` строит комплект графиков для отчета TASK-20.
%
% Входы:
%   none
%
% Выходы:
%   task_20_accel_log_diagnostics_figures - структура путей к рисункам в
%   base workspace
%
% Единицы измерения:
%   время - секунды;
%   ускорение - в единицах журнала `ArduPilot`;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   Предварительно выполнены сценарии сбора DataFlash и повторной попытки
%   `arm` после исправления.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

if ~isfolder(figures_dir)
    mkdir(figures_dir);
end

accel_mat = load(fullfile(reports_dir, 'task_20_accel_instances.mat'));
compare_mat = load(fullfile(reports_dir, 'task_20_json_vs_ardupilot_accel.mat'));
arm_fix_mat = load(fullfile(reports_dir, 'task_20_arm_after_accel_log_fix.mat'));

accel_table = accel_mat.result.accel_table;
compare_table = compare_mat.result_table;
arm_result = arm_fix_mat.result;

paths = struct();
paths.accel_instances = fullfile(figures_dir, 'task_20_accel_instances.png');
paths.accel_diff = fullfile(figures_dir, 'task_20_accel_instance_difference.png');
paths.json_vs_accel = fullfile(figures_dir, 'task_20_json_vs_ardupilot_accel.png');
paths.pwm_after_fix = fullfile(figures_dir, 'task_20_pwm_after_accel_fix.png');
paths.motor_after_fix = fullfile(figures_dir, 'task_20_motor_commands_after_accel_fix.png');

local_save_figure(paths.accel_instances, @() local_plot_accel_instances(accel_table));
local_save_figure(paths.accel_diff, @() local_plot_accel_difference(accel_table));
local_save_figure(paths.json_vs_accel, @() local_plot_json_vs_accel(compare_table));
local_save_figure(paths.pwm_after_fix, @() local_plot_pwm_after_fix(arm_result));
local_save_figure(paths.motor_after_fix, @() local_plot_motor_after_fix(arm_result));

assignin('base', 'task_20_accel_log_diagnostics_figures', paths);

fprintf('Графики TASK-20 сохранены\n');
fprintf('  accel_instances                       : %s\n', paths.accel_instances);
fprintf('  accel_instance_difference             : %s\n', paths.accel_diff);
fprintf('  json_vs_ardupilot_accel              : %s\n', paths.json_vs_accel);
fprintf('  pwm_after_accel_fix                   : %s\n', paths.pwm_after_fix);
fprintf('  motor_commands_after_accel_fix        : %s\n', paths.motor_after_fix);

function local_plot_accel_instances(accel_table)
instances = unique(accel_table.instance_id);
tiledlayout(3, 1, 'TileSpacing', 'compact');
components = {'AccX', 'AccY', 'AccZ'};
titles = {'AccX', 'AccY', 'AccZ'};
for comp_idx = 1:3
    nexttile;
    hold on;
    for inst = reshape(instances, 1, [])
        rows = accel_table(accel_table.instance_id == inst, :);
        plot(rows.time_s, rows.(components{comp_idx}), 'LineWidth', 1.1);
    end
    hold off;
    grid on;
    xlabel('t, c');
    ylabel(titles{comp_idx});
    title("Экземпляры IMU: " + titles{comp_idx});
    legend(compose("IMU%d", instances), 'Location', 'best');
end
end

function local_plot_accel_difference(accel_table)
groups = groupsummary(accel_table, 'time_us', 'max', 'max_pairwise_difference');
plot(groups.time_us * 1.0e-6, groups.max_max_pairwise_difference, 'LineWidth', 1.3);
grid on;
xlabel('t, c');
ylabel('Максимальное расхождение');
title('Расхождение между экземплярами акселерометров ArduPilot');
end

function local_plot_json_vs_accel(compare_table)
plot(compare_table.time_s, compare_table.json_acc_norm, 'k-', 'LineWidth', 1.4);
hold on;
var_names = compare_table.Properties.VariableNames;
norm_cols = var_names(startsWith(var_names, 'acc_norm_imu'));
for idx = 1:numel(norm_cols)
    plot(compare_table.time_s, compare_table.(norm_cols{idx}), 'LineWidth', 1.1);
end
hold off;
grid on;
xlabel('t, c');
ylabel('Норма ускорения');
title('Сравнение JSON accel\_body и внутренних IMU ArduPilot');
legend(['JSON', string(norm_cols)], 'Location', 'best', 'Interpreter', 'none');
end

function local_plot_pwm_after_fix(arm_result)
if ~isfield(arm_result, 'arm_response') || isempty(fieldnames(arm_result.arm_response))
    plot(nan, nan);
    title('Результат arm после исправления отсутствует');
    return;
end

live_result = arm_result.arm_response.live_result;
pwm_matrix = local_collect_pwm_matrix(live_result.sitl_output);
plot(live_result.time_s, pwm_matrix, 'LineWidth', 1.1);
grid on;
xlabel('t, c');
ylabel('ШИМ, мкс');
title('ШИМ каналов после исправления акселерометров');
legend({'PWM1', 'PWM2', 'PWM3', 'PWM4'}, 'Location', 'best');
end

function local_plot_motor_after_fix(arm_result)
if ~isfield(arm_result, 'arm_response') || isempty(fieldnames(arm_result.arm_response))
    plot(nan, nan);
    title('Результат arm после исправления отсутствует');
    return;
end

live_result = arm_result.arm_response.live_result;
plot(live_result.time_s, live_result.motor_cmd_radps, 'LineWidth', 1.1);
grid on;
xlabel('t, c');
ylabel('Команда, рад/с');
title('Команды частоты вращения винтов после исправления');
legend({'Motor1', 'Motor2', 'Motor3', 'Motor4'}, 'Location', 'best');
end

function pwm_matrix = local_collect_pwm_matrix(sitl_output)
sample_count = numel(sitl_output);
pwm_matrix = nan(sample_count, 4);
for idx = 1:sample_count
    pwm_us = double(sitl_output(idx).motor_pwm_us(:));
    if numel(pwm_us) >= 4
        pwm_matrix(idx, :) = pwm_us(1:4).';
    end
end
end

function local_save_figure(file_path, plotter)
fig = figure('Visible', 'off');
cleanup_obj = onCleanup(@() close(fig)); %#ok<NASGU>
plotter();
set(fig, 'Color', 'w');
exportgraphics(fig, file_path, 'Resolution', 150);
end
