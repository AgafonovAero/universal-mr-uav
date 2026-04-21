%% PLOT_ARDUPILOT_OFFICIAL_MATLAB_BACKEND
% Построить графики TASK-25 по официальному MATLAB backend ArduPilot.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');
if ~isfolder(figures_dir)
    mkdir(figures_dir);
end

lockstep_csv = fullfile(reports_dir, 'task_25_official_lockstep_20s.csv');
arm_csv = fullfile(reports_dir, 'task_25_official_arm_takeoff_attempt.csv');

if ~isfile(lockstep_csv)
    error('uav:task25:plot:MissingLockstepCsv', ...
        'Не найден CSV lockstep-прогона TASK-25: %s', lockstep_csv);
end
if ~isfile(arm_csv)
    error('uav:task25:plot:MissingArmCsv', ...
        'Не найден CSV arm/takeoff-прогона TASK-25: %s', arm_csv);
end

lockstep_table = readtable(lockstep_csv, 'TextType', 'string');
arm_table = readtable(arm_csv, 'TextType', 'string');

local_plot_counts(lockstep_table, fullfile(figures_dir, 'task_25_lockstep_counts.png'));
local_plot_pwm(arm_table, fullfile(figures_dir, 'task_25_pwm_channels.png'));
local_plot_motor_commands(arm_table, fullfile(figures_dir, 'task_25_motor_commands.png'));
local_plot_accel(lockstep_table, fullfile(figures_dir, 'task_25_accel_body.png'));
local_plot_altitude(arm_table, fullfile(figures_dir, 'task_25_altitude.png'));
local_plot_attitude(lockstep_table, fullfile(figures_dir, 'task_25_attitude.png'));

fprintf('TASK-25: графики официального MATLAB backend построены.\n');

function local_plot_counts(data_table, save_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(data_table.time_s, data_table.valid_rx_count, 'LineWidth', 1.4);
hold on;
plot(data_table.time_s, data_table.json_response_tx_count, 'LineWidth', 1.4);
plot(data_table.time_s, data_table.duplicate_frame_count, '--', 'LineWidth', 1.2);
plot(data_table.time_s, data_table.missed_frame_count, '--', 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('Счетчик');
title('TASK-25: счетчики official lockstep');
legend({'valid\_rx\_count', 'json\_response\_tx\_count', 'duplicate\_frame\_count', 'missed\_frame\_count'}, ...
    'Location', 'best');
exportgraphics(fig, save_path, 'Resolution', 150);
close(fig);
end

function local_plot_pwm(data_table, save_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(data_table.time_s, data_table.pwm_1_us, 'LineWidth', 1.2);
hold on;
plot(data_table.time_s, data_table.pwm_2_us, 'LineWidth', 1.2);
plot(data_table.time_s, data_table.pwm_3_us, 'LineWidth', 1.2);
plot(data_table.time_s, data_table.pwm_4_us, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('ШИМ, мкс');
title('TASK-25: каналы ШИМ в официальном arm/takeoff-прогоне');
legend({'PWM1', 'PWM2', 'PWM3', 'PWM4'}, 'Location', 'best');
exportgraphics(fig, save_path, 'Resolution', 150);
close(fig);
end

function local_plot_motor_commands(data_table, save_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(data_table.time_s, data_table.motor_1_radps, 'LineWidth', 1.2);
hold on;
plot(data_table.time_s, data_table.motor_2_radps, 'LineWidth', 1.2);
plot(data_table.time_s, data_table.motor_3_radps, 'LineWidth', 1.2);
plot(data_table.time_s, data_table.motor_4_radps, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('Частота вращения, рад/с');
title('TASK-25: команды винтомоторной группе');
legend({'Motor1', 'Motor2', 'Motor3', 'Motor4'}, 'Location', 'best');
exportgraphics(fig, save_path, 'Resolution', 150);
close(fig);
end

function local_plot_accel(data_table, save_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(data_table.time_s, data_table.accel_body_x, 'LineWidth', 1.2);
hold on;
plot(data_table.time_s, data_table.accel_body_y, 'LineWidth', 1.2);
plot(data_table.time_s, data_table.accel_body_z, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('accel\_body, м/с^2');
title('TASK-25: компоненты accel\_body');
legend({'X_b', 'Y_b', 'Z_b'}, 'Location', 'best');
exportgraphics(fig, save_path, 'Resolution', 150);
close(fig);
end

function local_plot_altitude(data_table, save_path)
fig = figure('Visible', 'off', 'Color', 'w');
yyaxis left;
plot(data_table.time_s, data_table.altitude_m, 'LineWidth', 1.4);
ylabel('Высота, м');
yyaxis right;
plot(data_table.time_s, data_table.total_thrust_to_weight, 'LineWidth', 1.2);
ylabel('Суммарная тяга / вес');
grid on;
xlabel('Время, с');
title('TASK-25: высота и тяговооруженность');
exportgraphics(fig, save_path, 'Resolution', 150);
close(fig);
end

function local_plot_attitude(data_table, save_path)
fig = figure('Visible', 'off', 'Color', 'w');
plot(data_table.time_s, data_table.roll_rad, 'LineWidth', 1.2);
hold on;
plot(data_table.time_s, data_table.pitch_rad, 'LineWidth', 1.2);
plot(data_table.time_s, data_table.yaw_rad, 'LineWidth', 1.2);
grid on;
xlabel('Время, с');
ylabel('Угол, рад');
title('TASK-25: ориентация модели');
legend({'roll', 'pitch', 'yaw'}, 'Location', 'best');
exportgraphics(fig, save_path, 'Resolution', 150);
close(fig);
end
