%% PLOT_ARDUPILOT_JSON_TAKEOFF_ATTEMPT Построить графики первого взлетного прогона TASK-23.
% Назначение:
%   Загружает результаты диагностики моторов, idle arm, GUIDED takeoff и
%   throttle step, затем сохраняет комплект графиков TASK-23.
%
% Входы:
%   none
%
% Выходы:
%   task_23_takeoff_plot_paths - структура путей к построенным рисункам
%
% Единицы измерения:
%   по исходным отчетам TASK-23.
%
% Допущения:
%   Предыдущие сценарии TASK-23 уже сформировали MAT-отчеты в
%   `artifacts/reports`.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

if ~isfolder(figures_dir)
    mkdir(figures_dir);
end

balance = local_load_result(fullfile(reports_dir, 'task_23_pwm_to_thrust_balance.mat'));
idle = local_load_result(fullfile(reports_dir, 'task_23_arm_idle_observation.mat'));
guided = local_load_result(fullfile(reports_dir, 'task_23_guided_takeoff_1m.mat'));
throttle = local_load_result(fullfile(reports_dir, 'task_23_throttle_step_response.mat'));

paths = struct();
paths.pwm_to_thrust_balance = fullfile(figures_dir, 'task_23_pwm_to_thrust_balance.png');
paths.arm_idle_pwm = fullfile(figures_dir, 'task_23_arm_idle_pwm.png');
paths.guided_takeoff_altitude = fullfile(figures_dir, 'task_23_guided_takeoff_altitude.png');
paths.guided_takeoff_pwm = fullfile(figures_dir, 'task_23_guided_takeoff_pwm.png');
paths.guided_takeoff_motor_commands = fullfile(figures_dir, 'task_23_guided_takeoff_motor_commands.png');
paths.guided_takeoff_attitude = fullfile(figures_dir, 'task_23_guided_takeoff_attitude.png');
paths.throttle_step_response = fullfile(figures_dir, 'task_23_throttle_step_response.png');

local_plot_balance(balance.table, balance.hover_pwm_us, balance.mot_hover_equivalent_pwm_us, paths.pwm_to_thrust_balance);
local_plot_pwm_history(idle.history_table, 'TASK-23: idle arm, каналы ШИМ', paths.arm_idle_pwm);
local_plot_altitude(guided.history_table, 'TASK-23: GUIDED takeoff 1 м, высота', paths.guided_takeoff_altitude);
local_plot_pwm_history(guided.history_table, 'TASK-23: GUIDED takeoff 1 м, каналы ШИМ', paths.guided_takeoff_pwm);
local_plot_motor_commands(guided.history_table, 'TASK-23: GUIDED takeoff 1 м, команды винтов', paths.guided_takeoff_motor_commands);
local_plot_attitude(guided.history_table, 'TASK-23: GUIDED takeoff 1 м, углы ориентации', paths.guided_takeoff_attitude);
local_plot_throttle(throttle.history_table, 'TASK-23: throttle step response', paths.throttle_step_response);

assignin('base', 'task_23_takeoff_plot_paths', paths);

fprintf('TASK-23: графики первого взлетного прогона сохранены\n');
fprintf('  %s\n', paths.pwm_to_thrust_balance);
fprintf('  %s\n', paths.arm_idle_pwm);
fprintf('  %s\n', paths.guided_takeoff_altitude);
fprintf('  %s\n', paths.guided_takeoff_pwm);
fprintf('  %s\n', paths.guided_takeoff_motor_commands);
fprintf('  %s\n', paths.guided_takeoff_attitude);
fprintf('  %s\n', paths.throttle_step_response);

function result = local_load_result(mat_path)
%LOCAL_LOAD_RESULT Загрузить структуру result из MAT-отчета.

if ~isfile(mat_path)
    error('uav:task23:plot:MissingReport', ...
        'Не найден MAT-отчет TASK-23: %s', mat_path);
end

loaded = load(mat_path);
result = loaded.result;
end

function local_plot_balance(balance_table, hover_pwm_us, mot_hover_pwm_us, output_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(balance_table.pwm_us, balance_table.thrust_to_weight_ratio, 'LineWidth', 1.5);
grid on;
hold on;
yline(1.0, '--', 'Вес БВС', 'LineWidth', 1.0);
xline(hover_pwm_us, '--', 'Расчетное зависание', 'LineWidth', 1.0);
if isfinite(mot_hover_pwm_us)
    xline(mot_hover_pwm_us, ':', 'MOT\_THST\_HOVER', 'LineWidth', 1.0);
end
xlabel('ШИМ, мкс');
ylabel('Суммарная тяга / вес');
title('TASK-23: тяговый баланс по команде ШИМ');
exportgraphics(fig, output_path, 'Resolution', 150);
close(fig);
end

function local_plot_pwm_history(history_table, title_text, output_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(history_table.time_s, history_table{:, {'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us'}}, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('ШИМ, мкс');
title(title_text);
legend({'PWM1', 'PWM2', 'PWM3', 'PWM4'}, 'Location', 'best');
exportgraphics(fig, output_path, 'Resolution', 150);
close(fig);
end

function local_plot_altitude(history_table, title_text, output_path)
fig = figure('Visible', 'off', 'Color', 'w');
yyaxis left;
plot(history_table.time_s, history_table.altitude_m, 'LineWidth', 1.4);
ylabel('Высота, м');
yyaxis right;
plot(history_table.time_s, history_table.vertical_speed_up_mps, 'LineWidth', 1.0);
ylabel('Вертикальная скорость вверх, м/с');
grid on;
xlabel('Время, с');
title(title_text);
legend({'Высота', 'Вертикальная скорость'}, 'Location', 'best');
exportgraphics(fig, output_path, 'Resolution', 150);
close(fig);
end

function local_plot_motor_commands(history_table, title_text, output_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(history_table.time_s, history_table{:, {'motor_1_radps', 'motor_2_radps', 'motor_3_radps', 'motor_4_radps'}}, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('Команда частоты вращения, рад/с');
title(title_text);
legend({'M1', 'M2', 'M3', 'M4'}, 'Location', 'best');
exportgraphics(fig, output_path, 'Resolution', 150);
close(fig);
end

function local_plot_attitude(history_table, title_text, output_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(history_table.time_s, history_table{:, {'roll_rad', 'pitch_rad', 'yaw_rad'}}, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('Угол, рад');
title(title_text);
legend({'roll', 'pitch', 'yaw'}, 'Location', 'best');
exportgraphics(fig, output_path, 'Resolution', 150);
close(fig);
end

function local_plot_throttle(history_table, title_text, output_path)
fig = figure('Visible', 'off', 'Color', 'w');
yyaxis left;
plot(history_table.time_s, history_table.throttle_target_us, 'LineWidth', 1.4);
ylabel('Команда газа, мкс');
yyaxis right;
plot(history_table.time_s, history_table{:, {'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us'}}, 'LineWidth', 1.0);
ylabel('Выходной ШИМ, мкс');
grid on;
xlabel('Время, с');
title(title_text);
legend({'Throttle target', 'PWM1', 'PWM2', 'PWM3', 'PWM4'}, 'Location', 'best');
exportgraphics(fig, output_path, 'Resolution', 150);
close(fig);
end
