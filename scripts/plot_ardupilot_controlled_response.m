%% PLOT_ARDUPILOT_CONTROLLED_RESPONSE Построить графики управляемого отклика ArduPilot.
% Назначение:
%   Загружает результаты наблюдения ШИМ и первого управляемого прогона,
%   затем строит и сохраняет набор PNG-графиков для инженерного анализа.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

obs_loaded = load(fullfile(reports_dir, 'task_16_pwm_observation.mat'));
observation = obs_loaded.observation;
ctrl_loaded = load(fullfile(reports_dir, 'task_16_first_controlled_response.mat'));
controlled = ctrl_loaded.result;

if controlled.executed
    plot_source = controlled.live_result;
    plot_source_name = "управляемый прогон";
else
    plot_source = observation;
    plot_source_name = "наблюдение ШИМ при невзведенном ArduPilot";
end

local_ensure_dir(figures_dir);
local_plot_pwm(observation, figures_dir);
local_plot_motor_commands(observation, figures_dir);
local_plot_altitude(plot_source, figures_dir, plot_source_name);
local_plot_attitude(plot_source, figures_dir, plot_source_name);
local_plot_exchange_counts(plot_source, figures_dir, plot_source_name);

fprintf('Графики TASK-16 сохранены:\n');
fprintf('  artifacts/figures/task_16_pwm_channels.png\n');
fprintf('  artifacts/figures/task_16_motor_commands.png\n');
fprintf('  artifacts/figures/task_16_altitude.png\n');
fprintf('  artifacts/figures/task_16_attitude_angles.png\n');
fprintf('  artifacts/figures/task_16_exchange_counts.png\n');
fprintf('  источник для высоты/углов/счетчиков    : %s\n', char(plot_source_name));

function local_plot_pwm(observation, figures_dir)
figure_handle = figure('Visible', 'off');
plot(observation.time_s, observation.motor_pwm_us, 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('ШИМ, мкс');
title('Каналы ШИМ ArduPilot');
legend('Канал 1', 'Канал 2', 'Канал 3', 'Канал 4', 'Location', 'best');
exportgraphics(figure_handle, fullfile(figures_dir, 'task_16_pwm_channels.png'));
close(figure_handle);
end

function local_plot_motor_commands(observation, figures_dir)
figure_handle = figure('Visible', 'off');
plot(observation.time_s, observation.motor_cmd_radps, 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('\omega, рад/с');
title('Команды частоты вращения винтов');
legend('Двигатель 1', 'Двигатель 2', 'Двигатель 3', 'Двигатель 4', 'Location', 'best');
exportgraphics(figure_handle, fullfile(figures_dir, 'task_16_motor_commands.png'));
close(figure_handle);
end

function local_plot_altitude(source_result, figures_dir, source_name)
figure_handle = figure('Visible', 'off');
altitude_m = -arrayfun(@(s) double(s.p_ned_m(3)), source_result.state);
est_alt_m = arrayfun(@(e) double(e.alt_m), source_result.estimator);
plot(source_result.time_s, altitude_m, 'LineWidth', 1.2);
hold on;
plot(source_result.time_s, est_alt_m, '--', 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('Высота, м');
title("Высота модели и оцененная высота: " + source_name);
legend('Высота модели', 'Оцененная высота', 'Location', 'best');
exportgraphics(figure_handle, fullfile(figures_dir, 'task_16_altitude.png'));
close(figure_handle);
end

function local_plot_attitude(source_result, figures_dir, source_name)
figure_handle = figure('Visible', 'off');
angles_rad = zeros(numel(source_result.time_s), 3);
for idx = 1:numel(source_result.time_s)
    angles_rad(idx, :) = local_quat_to_euler_rpy(source_result.state(idx).q_nb).';
end
plot(source_result.time_s, angles_rad, 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('Угол, рад');
title("Углы крена, тангажа и рыскания: " + source_name);
legend('Крен', 'Тангаж', 'Рыскание', 'Location', 'best');
exportgraphics(figure_handle, fullfile(figures_dir, 'task_16_attitude_angles.png'));
close(figure_handle);
end

function local_plot_exchange_counts(source_result, figures_dir, source_name)
figure_handle = figure('Visible', 'off');
n_rows = numel(source_result.exchange_diag);
valid_count = zeros(n_rows, 1);
tx_count = zeros(n_rows, 1);
reply_count = zeros(n_rows, 1);
for idx = 1:n_rows
    valid_count(idx) = sum(arrayfun(@(d) double(d.rx_valid_count), source_result.exchange_diag(1:idx)));
    tx_count(idx) = sum(arrayfun(@(d) double(d.tx_ok), source_result.exchange_diag(1:idx)));
    reply_count(idx) = sum(arrayfun(@(d) double(d.handshake_confirmed), source_result.exchange_diag(1:idx)));
end
plot(source_result.time_s, valid_count, 'LineWidth', 1.2);
hold on;
plot(source_result.time_s, tx_count, 'LineWidth', 1.2);
plot(source_result.time_s, reply_count, 'LineWidth', 1.2);
grid on;
xlabel('t, c');
ylabel('Счетчик');
title("Счетчики обмена JSON/UDP: " + source_name);
legend('Валидные пакеты', 'Отправленные строки JSON', 'Ответные передачи', 'Location', 'best');
exportgraphics(figure_handle, fullfile(figures_dir, 'task_16_exchange_counts.png'));
close(figure_handle);
end

function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
%LOCAL_QUAT_TO_EULER_RPY Преобразовать кватернион в углы Эйлера.

q_nb = q_nb(:) ./ norm(q_nb);
w = q_nb(1);
x = q_nb(2);
y = q_nb(3);
z = q_nb(4);

roll_rad = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x^2 + y^2));
pitch_arg = 2.0 * (w * y - z * x);
pitch_arg = min(max(pitch_arg, -1.0), 1.0);
pitch_rad = asin(pitch_arg);
yaw_rad = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y^2 + z^2));

euler_rpy_rad = [roll_rad; pitch_rad; yaw_rad];
end

function local_ensure_dir(path_value)
%LOCAL_ENSURE_DIR Создать каталог при отсутствии.

if ~isfolder(path_value)
    mkdir(path_value);
end
end
