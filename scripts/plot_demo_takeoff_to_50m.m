%% PLOT_DEMO_TAKEOFF_TO_50M Build and save the takeoff-demo PNG figures.
% Description:
%   Loads the saved takeoff demo MAT artifact and exports altitude,
%   altitude-error, vertical-speed, and motor-command plots into
%   `artifacts/figures/`.
%
% Inputs:
%   none
%
% Outputs:
%   none
%
% Units:
%   SI only
%
% Assumptions:
%   `scripts/run_demo_takeoff_to_50m.m` has already created the MAT file.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');
if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
end

mat_file = fullfile(reports_dir, 'demo_takeoff_to_50m.mat');
if ~exist(mat_file, 'file')
    error('uav:demo:plot_takeoff:MissingMatFile', ...
        'Expected MAT artifact to exist: %s', mat_file);
end

loaded_data = load(mat_file, 'demo');
demo = loaded_data.demo;
series = demo.series;

altitude_png = fullfile(figures_dir, 'demo_takeoff_altitude.png');
altitude_error_png = fullfile(figures_dir, 'demo_takeoff_altitude_error.png');
vertical_speed_png = fullfile(figures_dir, 'demo_takeoff_vertical_speed.png');
motor_png = fullfile(figures_dir, 'demo_takeoff_motor_cmd.png');

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, series.altitude_m, 'LineWidth', 1.6);
hold on;
plot(series.time_s, series.altitude_est_m, '--', 'LineWidth', 1.4);
plot(series.time_s, series.altitude_ref_m, ':', 'LineWidth', 1.8);
grid on;
xlabel('Time, s');
ylabel('Altitude, m');
title('Takeoff to 50 m: altitude');
legend({'True altitude', 'Estimated altitude', 'Reference'}, ...
    'Location', 'best');
local_export_figure(fig, altitude_png);

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, series.altitude_ref_m - series.altitude_m, 'LineWidth', 1.6);
hold on;
plot(series.time_s, series.altitude_ref_m - series.altitude_est_m, ...
    '--', 'LineWidth', 1.4);
yline(0.0, ':', 'LineWidth', 1.2);
grid on;
xlabel('Time, s');
ylabel('Altitude error, m');
title('Takeoff to 50 m: altitude error');
legend({'Reference - true', 'Reference - estimated', 'Zero'}, ...
    'Location', 'best');
local_export_figure(fig, altitude_error_png);

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, series.vertical_speed_mps, 'LineWidth', 1.6);
hold on;
plot(series.time_s, series.vertical_speed_est_mps, '--', 'LineWidth', 1.4);
plot(series.time_s, series.vertical_speed_ref_mps, ':', 'LineWidth', 1.8);
grid on;
xlabel('Time, s');
ylabel('Vertical speed, m/s');
title('Takeoff to 50 m: vertical speed');
legend({'True vertical speed', 'Estimated vertical speed', 'Reference'}, ...
    'Location', 'best');
local_export_figure(fig, vertical_speed_png);

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, series.motor_cmd_radps, 'LineWidth', 1.3);
grid on;
xlabel('Time, s');
ylabel('Motor command, rad/s');
title('Takeoff to 50 m: motor commands');
legend({'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'}, 'Location', 'best');
local_export_figure(fig, motor_png);

fprintf('Saved PNG: %s\n', altitude_png);
fprintf('Saved PNG: %s\n', altitude_error_png);
fprintf('Saved PNG: %s\n', vertical_speed_png);
fprintf('Saved PNG: %s\n', motor_png);

function local_export_figure(fig, file_path)
%LOCAL_EXPORT_FIGURE Export one figure and close it afterwards.

exportgraphics(fig, file_path, 'Resolution', 150);
close(fig);
end
