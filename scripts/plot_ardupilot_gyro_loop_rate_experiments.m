%% PLOT_ARDUPILOT_GYRO_LOOP_RATE_EXPERIMENTS Построить графики TASK-22.
% Назначение:
%   По сохраненным результатам серии опытов TASK-22 строит графики
%   зависимости результата взведения от `SCHED_LOOP_RATE`, а также
%   графики диапазонов ШИМ и команд частоты вращения винтов.
%
% Входы:
%   none
%
% Выходы:
%   task_22_gyro_loop_rate_plots - структура путей к рисункам в base
%
% Единицы измерения:
%   частоты - Гц;
%   ШИМ - микросекунды;
%   команды частоты вращения - рад/с.
%
% Допущения:
%   MAT-файл `artifacts/reports/task_22_gyro_loop_rate_experiments.mat`
%   уже сформирован сценарием `run_ardupilot_gyro_loop_rate_experiments.m`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_path = fullfile(repo_root, 'artifacts', 'reports', 'task_22_gyro_loop_rate_experiments.mat');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

if ~isfile(reports_path)
    error('uav:task22:plotGyroLoop:MissingMat', ...
        'Не найден MAT-файл серии опытов TASK-22: %s', reports_path);
end

if ~isfolder(figures_dir)
    mkdir(figures_dir);
end

loaded = load(reports_path);
result = loaded.result;
case_table = result.table;

sched_values = case_table.actual_sched_loop_rate;
case_labels = string(case_table.case_id) + " (" + string(case_table.actual_sched_loop_rate) + " Hz)";
x_values = 1:height(case_table);

paths = struct();
paths.gyro_rate_vs_loop_rate = fullfile(figures_dir, 'task_22_gyro_rate_vs_loop_rate.png');
paths.arm_result_by_sched_loop_rate = fullfile(figures_dir, 'task_22_arm_result_by_sched_loop_rate.png');
paths.pwm_after_loop_rate_fix = fullfile(figures_dir, 'task_22_pwm_after_loop_rate_fix.png');
paths.motor_commands_after_loop_rate_fix = fullfile(figures_dir, 'task_22_motor_commands_after_loop_rate_fix.png');

fig = figure('Visible', 'off');
plot(x_values, case_table.gyro_rate_hz, '-o', 'LineWidth', 1.5, 'DisplayName', 'Фактическая частота гироскопа');
hold on;
plot(x_values, case_table.min_required_gyro_rate_hz, '-s', 'LineWidth', 1.5, 'DisplayName', 'Требуемая минимальная частота');
grid on;
xticks(x_values);
xticklabels(case_labels);
xtickangle(20);
ylabel('Частота, Гц');
title('Сопоставление частоты гироскопа и требования 1.8 * SCHED\_LOOP\_RATE');
legend('Location', 'best');
exportgraphics(fig, paths.gyro_rate_vs_loop_rate, 'Resolution', 150);
close(fig);

fig = figure('Visible', 'off');
bar(x_values, double(case_table.arm_succeeded), 0.6);
grid on;
ylim([0, 1.2]);
xticks(x_values);
xticklabels(case_labels);
xtickangle(20);
ylabel('Флаг взведения');
title('Результат взведения по профилям SCHED\_LOOP\_RATE');
exportgraphics(fig, paths.arm_result_by_sched_loop_rate, 'Resolution', 150);
close(fig);

fig = figure('Visible', 'off');
bar(x_values, [case_table.motor_pwm_min_us, case_table.motor_pwm_max_us], 'grouped');
grid on;
xticks(x_values);
xticklabels(case_labels);
xtickangle(20);
ylabel('ШИМ, мкс');
title('Диапазон ШИМ винтомоторной группы после попытки взведения');
legend({'Минимум', 'Максимум'}, 'Location', 'best');
exportgraphics(fig, paths.pwm_after_loop_rate_fix, 'Resolution', 150);
close(fig);

fig = figure('Visible', 'off');
bar(x_values, [case_table.motor_cmd_min_radps, case_table.motor_cmd_max_radps], 'grouped');
grid on;
xticks(x_values);
xticklabels(case_labels);
xtickangle(20);
ylabel('Команда частоты вращения, рад/с');
title('Диапазон команд частоты вращения после попытки взведения');
legend({'Минимум', 'Максимум'}, 'Location', 'best');
exportgraphics(fig, paths.motor_commands_after_loop_rate_fix, 'Resolution', 150);
close(fig);

assignin('base', 'task_22_gyro_loop_rate_plots', paths);

fprintf('TASK-22: графики по частоте гироскопа и SCHED_LOOP_RATE сохранены\n');
fprintf('  %s\n', paths.gyro_rate_vs_loop_rate);
fprintf('  %s\n', paths.arm_result_by_sched_loop_rate);
fprintf('  %s\n', paths.pwm_after_loop_rate_fix);
fprintf('  %s\n', paths.motor_commands_after_loop_rate_fix);
