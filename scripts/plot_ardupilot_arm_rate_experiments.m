%% PLOT_ARDUPILOT_ARM_RATE_EXPERIMENTS Построить графики по опытам TASK-17.
% Назначение:
%   Загружает агрегированные результаты серии опытов TASK-17 и формирует
%   графики частоты обмена, результата взведения и диапазонов команд ШИМ
%   и частоты вращения винтов.
%
% Входы:
%   none
%
% Выходы:
%   task_17_arm_rate_plots - структура с путями построенных графиков в
%   базовом рабочем пространстве MATLAB
%
% Допущения:
%   Предварительно выполнен сценарий
%   `scripts/run_ardupilot_arm_rate_experiments.m`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
figures_dir = fullfile(repo_root, 'artifacts', 'figures');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
report_path = fullfile(reports_dir, 'task_17_arm_rate_experiments.mat');

if ~isfolder(figures_dir)
    mkdir(figures_dir);
end

loaded = load(report_path, 'experiments');
experiments = loaded.experiments;
case_results = experiments.cases;

case_labels = string({case_results.case_id});
mean_rate = [case_results.average_exchange_rate_hz];
p95_period = [case_results.p95_exchange_period_s];
arm_result = double([case_results.arm_succeeded]);
pwm_after_max = [case_results.motor_pwm_after_max_us];
motor_cmd_after_max = [case_results.motor_cmd_after_max_radps];

exchange_rate_path = fullfile(figures_dir, 'task_17_exchange_rate.png');
arm_result_path = fullfile(figures_dir, 'task_17_arm_result_by_case.png');
pwm_path = fullfile(figures_dir, 'task_17_pwm_before_after_arm.png');
motor_cmd_path = fullfile(figures_dir, 'task_17_motor_commands_before_after_arm.png');

fig = figure('Visible', 'off');
tiledlayout(2, 1);
nexttile;
bar(categorical(case_labels), mean_rate);
ylabel('Гц');
title('Средняя частота ответного обмена');
grid on;
nexttile;
bar(categorical(case_labels), p95_period);
ylabel('с');
title('95-процентиль периода ответного обмена');
grid on;
exportgraphics(fig, exchange_rate_path);
close(fig);

fig = figure('Visible', 'off');
bar(categorical(case_labels), arm_result);
ylim([0, 1.2]);
yticks([0, 1]);
yticklabels({'нет', 'да'});
title('Результат попытки взведения по опытам');
grid on;
exportgraphics(fig, arm_result_path);
close(fig);

fig = figure('Visible', 'off');
bar(categorical(case_labels), pwm_after_max);
ylabel('мкс');
title('Максимальные значения ШИМ после попытки взведения');
grid on;
exportgraphics(fig, pwm_path);
close(fig);

fig = figure('Visible', 'off');
bar(categorical(case_labels), motor_cmd_after_max);
ylabel('рад/с');
title('Максимальные команды частоты вращения после попытки взведения');
grid on;
exportgraphics(fig, motor_cmd_path);
close(fig);

plots = struct();
plots.exchange_rate = string(exchange_rate_path);
plots.arm_result = string(arm_result_path);
plots.pwm = string(pwm_path);
plots.motor_cmd = string(motor_cmd_path);
assignin('base', 'task_17_arm_rate_plots', plots);

fprintf('Построены графики TASK-17:\n');
fprintf('  %s\n', exchange_rate_path);
fprintf('  %s\n', arm_result_path);
fprintf('  %s\n', pwm_path);
fprintf('  %s\n', motor_cmd_path);
