%% RUN_DEMO_TAKEOFF_TO_50M Execute an estimator-driven takeoff-and-hold demo.
% Description:
%   Runs a reproducible climb to 50 m with altitude hold on top of
%   `uav.sim.run_case_closed_loop_with_estimator`, saves MAT/CSV artifacts,
%   and prints compact end-of-run diagnostics.
%
% Inputs:
%   none
%
% Outputs:
%   demo_takeoff_to_50m - assigned in base workspace
%
% Units:
%   SI only, angles in code remain in radians
%
% Assumptions:
%   Sensor noise is disabled and the controller closes the loop only
%   through estimator outputs and measured gyro rates.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
if ~exist(reports_dir, 'dir')
    mkdir(reports_dir);
end

params = uav.sim.make_deterministic_demo_params();
profile = local_make_takeoff_profile(params);
controller_cfg = profile.controller_cfg;

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = profile.t_final_s;
case_cfg.reference_fun = ...
    @(t_s, ~, ~, ~) local_reference_at_time(t_s, profile);
case_cfg.controller_fun = @(ctrl_input, ctrl_state, dt_s, params_local) ...
    uav.ctrl.demo_takeoff_hold_controller( ...
        ctrl_input, ctrl_state, dt_s, params_local, controller_cfg);

log = uav.sim.run_case_closed_loop_with_estimator(case_cfg);
refs = local_make_reference_history(log.time_s, profile);
series = uav.sim.postprocess_demo_log(log, refs);
demo_table = uav.sim.demo_series_to_table(series);
metrics = local_make_metrics(series, profile);

mat_file = fullfile(reports_dir, 'demo_takeoff_to_50m.mat');
csv_file = fullfile(reports_dir, 'demo_takeoff_to_50m.csv');

demo = struct();
demo.name = 'demo_takeoff_to_50m';
demo.case_cfg = case_cfg;
demo.profile = profile;
demo.log = log;
demo.refs = refs;
demo.series = series;
demo.table = demo_table;
demo.metrics = metrics;
demo.mat_file = mat_file;
demo.csv_file = csv_file;

save(mat_file, 'demo');
writetable(demo_table, csv_file);
assignin('base', 'demo_takeoff_to_50m', demo);

fprintf('Demo takeoff to 50 m diagnostics:\n');
fprintf('  final altitude [m]          : %.6f\n', metrics.final_altitude_m);
fprintf('  peak altitude error [m]     : %.6f\n', metrics.peak_altitude_error_m);
fprintf('  final estimated altitude [m]: %.6f\n', ...
    metrics.final_estimated_altitude_m);
fprintf('  final accel weight [-]      : %.6f\n', ...
    metrics.final_accel_correction_weight);
fprintf('  final quat norms [-]        : true=%.12f est=%.12f\n', ...
    metrics.final_true_quat_norm, metrics.final_estimated_quat_norm);
fprintf('  saved MAT                   : %s\n', mat_file);
fprintf('  saved CSV                   : %s\n', csv_file);

function profile = local_make_takeoff_profile(params)
%LOCAL_MAKE_TAKEOFF_PROFILE Build the deterministic takeoff-demo settings.
% Description:
%   Collects the altitude reference and controller gains for the 50 m
%   estimator-driven takeoff scenario.
%
% Inputs:
%   params - baseline parameter struct
%
% Outputs:
%   profile - scalar struct with demo settings
%
% Units:
%   SI only
%
% Assumptions:
%   The vehicle starts from level hover on the ground-state preset.

profile = struct();
profile.target_altitude_m = 50.0;
profile.climb_rate_mps = 4.0;
profile.t_final_s = 24.0;
profile.controller_cfg = struct( ...
    'altitude_kp_per_s2', 0.55, ...
    'vertical_speed_kp_per_s', 1.40, ...
    'up_accel_limit_mps2', 3.5, ...
    'angle_kp_radps_per_rad', [3.0; 3.2; 1.2], ...
    'body_rate_cmd_limit_radps', [0.7; 0.7; 0.5], ...
    'rate_pid_gains', struct( ...
        'Kp', [0.12; 0.12; 0.08], ...
        'Ki', [0.04; 0.04; 0.02], ...
        'Kd', [0.003; 0.003; 0.001]), ...
    'body_moment_limit_Nm', [0.05; 0.05; 0.03], ...
    'total_thrust_min_N', 0.40 * params.mass_kg * params.gravity_mps2, ...
    'total_thrust_max_N', 1.80 * params.mass_kg * params.gravity_mps2, ...
    'min_body_z_up_gain', 0.35);
end

function ref = local_reference_at_time(t_s, profile)
%LOCAL_REFERENCE_AT_TIME Return the takeoff reference at one time instant.

ramp_duration_s = profile.target_altitude_m / profile.climb_rate_mps;

if t_s < ramp_duration_s
    altitude_ref_m = profile.climb_rate_mps * t_s;
    vertical_speed_ref_mps = profile.climb_rate_mps;
else
    altitude_ref_m = profile.target_altitude_m;
    vertical_speed_ref_mps = 0.0;
end

ref = struct();
ref.altitude_ref_m = altitude_ref_m;
ref.vertical_speed_ref_mps = vertical_speed_ref_mps;
ref.pitch_ref_rad = 0.0;
end

function refs = local_make_reference_history(time_s, profile)
%LOCAL_MAKE_REFERENCE_HISTORY Sample the takeoff reference on the run grid.

n_samples = numel(time_s);
refs = struct();
refs.altitude_ref_m = zeros(n_samples, 1);
refs.vertical_speed_ref_mps = zeros(n_samples, 1);
refs.pitch_ref_rad = zeros(n_samples, 1);

for k = 1:n_samples
    ref_k = local_reference_at_time(time_s(k), profile);
    refs.altitude_ref_m(k) = ref_k.altitude_ref_m;
    refs.vertical_speed_ref_mps(k) = ref_k.vertical_speed_ref_mps;
    refs.pitch_ref_rad(k) = ref_k.pitch_ref_rad;
end
end

function metrics = local_make_metrics(series, profile)
%LOCAL_MAKE_METRICS Build compact scalar diagnostics for reporting.

metrics = struct();
metrics.final_altitude_m = series.altitude_m(end);
metrics.peak_altitude_error_m = ...
    max(abs(series.altitude_ref_m - series.altitude_m));
metrics.final_estimated_altitude_m = series.altitude_est_m(end);
metrics.final_accel_correction_weight = series.accel_correction_weight(end);
metrics.final_true_quat_norm = series.quat_norm_true(end);
metrics.final_estimated_quat_norm = series.quat_norm_est(end);
metrics.final_altitude_reference_m = profile.target_altitude_m;
end
