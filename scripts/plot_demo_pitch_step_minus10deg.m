%% PLOT_DEMO_PITCH_STEP_MINUS10DEG Build and save the pitch-demo PNG figures.
% Description:
%   Loads the saved pitch-step demo MAT artifact and exports pitch,
%   altitude, and motor-command plots into `artifacts/figures/`.
%
% Inputs:
%   none
%
% Outputs:
%   none
%
% Units:
%   SI only, with explicit degree conversion for pitch display
%
% Assumptions:
%   `scripts/run_demo_pitch_step_minus10deg.m` has already created the MAT
%   file.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');
if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
end

mat_file = fullfile(reports_dir, 'demo_pitch_step_minus10deg.mat');
if ~exist(mat_file, 'file')
    error('uav:demo:plot_pitch:MissingMatFile', ...
        'Expected MAT artifact to exist: %s', mat_file);
end

loaded_data = load(mat_file, 'demo');
demo = loaded_data.demo;
series = demo.series;

pitch_png = fullfile(figures_dir, 'demo_pitch_angle.png');
altitude_png = fullfile(figures_dir, 'demo_pitch_altitude.png');
motor_png = fullfile(figures_dir, 'demo_pitch_motor_cmd.png');

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, rad2deg(series.pitch_rad), 'LineWidth', 1.6);
hold on;
plot(series.time_s, rad2deg(series.pitch_est_rad), '--', 'LineWidth', 1.4);
plot(series.time_s, rad2deg(series.pitch_ref_rad), ':', 'LineWidth', 1.8);
grid on;
xlabel('Time, s');
ylabel('Pitch, deg');
title('Pitch step -10 deg: attitude response');
legend({'True pitch', 'Estimated pitch', 'Reference'}, 'Location', 'best');
local_export_figure(fig, pitch_png);

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, series.altitude_m, 'LineWidth', 1.6);
hold on;
plot(series.time_s, series.altitude_est_m, '--', 'LineWidth', 1.4);
plot(series.time_s, series.altitude_ref_m, ':', 'LineWidth', 1.8);
grid on;
xlabel('Time, s');
ylabel('Altitude, m');
title('Pitch step -10 deg: altitude');
legend({'True altitude', 'Estimated altitude', 'Reference'}, ...
    'Location', 'best');
local_export_figure(fig, altitude_png);

fig = figure('Visible', 'off', 'Color', 'w');
plot(series.time_s, series.motor_cmd_radps, 'LineWidth', 1.3);
grid on;
xlabel('Time, s');
ylabel('Motor command, rad/s');
title('Pitch step -10 deg: motor commands');
legend({'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'}, 'Location', 'best');
local_export_figure(fig, motor_png);

fprintf('Saved PNG: %s\n', pitch_png);
fprintf('Saved PNG: %s\n', altitude_png);
fprintf('Saved PNG: %s\n', motor_png);

function local_export_figure(fig, file_path)
%LOCAL_EXPORT_FIGURE Export one figure and close it afterwards.

exportgraphics(fig, file_path, 'Resolution', 150);
close(fig);
end
