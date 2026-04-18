%% RUN_DEMO_PITCH_STEP_MINUS10DEG Execute an estimator-driven pitch demo.
% Description:
%   Climbs to a safe altitude, commands a -10 deg pitch step, closes the
%   loop only through estimator outputs and measured gyro rates, and saves
%   MAT/CSV artifacts for plotting and reporting.
%
% Inputs:
%   none
%
% Outputs:
%   demo_pitch_step_minus10deg - assigned in base workspace
%
% Units:
%   SI only, with explicit degree conversion only for printed diagnostics
%
% Assumptions:
%   Sensor noise is disabled and the demo remains within small-angle
%   verification bounds.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
if ~exist(reports_dir, 'dir')
    mkdir(reports_dir);
end

params = uav.sim.make_deterministic_demo_params();
profile = local_make_pitch_profile(params);

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = profile.t_final_s;
case_cfg.reference_fun = @(t_s, ~, ~, ~) local_reference_at_time(t_s, profile);
case_cfg.controller_fun = @(ctrl_input, ctrl_state, dt_s, params_local) ...
    uav.ctrl.demo_pitch_hold_controller( ...
        ctrl_input, ctrl_state, dt_s, params_local, profile.controller_cfg);

log = uav.sim.run_case_closed_loop_with_estimator(case_cfg);
refs = local_make_reference_history(log.time_s, profile);
series = uav.sim.postprocess_demo_log(log, refs);
demo_table = uav.sim.demo_series_to_table(series);
metrics = local_make_metrics(series, profile);

mat_file = fullfile(reports_dir, 'demo_pitch_step_minus10deg.mat');
csv_file = fullfile(reports_dir, 'demo_pitch_step_minus10deg.csv');

demo = struct();
demo.name = 'demo_pitch_step_minus10deg';
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
assignin('base', 'demo_pitch_step_minus10deg', demo);

fprintf('Demo pitch step -10 deg diagnostics:\n');
fprintf('  final pitch [deg]             : %.6f\n', metrics.final_pitch_deg);
fprintf('  final estimated pitch [deg]   : %.6f\n', metrics.final_estimated_pitch_deg);
fprintf('  final pitch estimation err [deg]: %.6f\n', ...
    metrics.final_pitch_estimation_error_deg);
fprintf('  peak pitch tracking err [deg] : %.6f\n', metrics.peak_pitch_error_deg);
fprintf('  min accel weight [-]          : %.6f\n', metrics.min_accel_correction_weight);
fprintf('  max consistency metric [m/s^2]: %.6f\n', metrics.max_accel_consistency_metric);
fprintf('  final altitude [m]            : %.6f\n', metrics.final_altitude_m);
fprintf('  quat norms [-]                : true=%.12f est=%.12f\n', ...
    metrics.final_true_quat_norm, metrics.final_estimated_quat_norm);
fprintf('  saved MAT                     : %s\n', mat_file);
fprintf('  saved CSV                     : %s\n', csv_file);

function profile = local_make_pitch_profile(params)
%LOCAL_MAKE_PITCH_PROFILE Build the deterministic pitch-demo settings.
% Description:
%   Collects the altitude/pitch references and estimator-driven controller
%   gains for the -10 deg pitch scenario.
%
% Inputs:
%   params - baseline parameter struct
%
% Outputs:
%   profile - scalar struct with demo settings
%
% Units:
%   SI only, pitch reference stored in radians
%
% Assumptions:
%   The safe altitude is reached before the pitch step is commanded.

profile = struct();
profile.safe_altitude_m = 20.0;
profile.climb_rate_mps = 3.0;
profile.pitch_cmd_rad = deg2rad(-10.0);
profile.pitch_cmd_deg = -10.0;
profile.pitch_step_time_s = 9.0;
profile.t_final_s = 18.0;
profile.controller_cfg = struct( ...
    'altitude_kp_per_s2', 0.60, ...
    'vertical_speed_kp_per_s', 1.45, ...
    'up_accel_limit_mps2', 3.0, ...
    'angle_kp_radps_per_rad', [3.2; 4.0; 1.2], ...
    'body_rate_cmd_limit_radps', [0.8; 0.9; 0.5], ...
    'rate_pid_gains', struct( ...
        'Kp', [0.14; 0.18; 0.08], ...
        'Ki', [0.05; 0.07; 0.02], ...
        'Kd', [0.003; 0.004; 0.001]), ...
    'body_moment_limit_Nm', [0.06; 0.08; 0.03], ...
    'total_thrust_min_N', 0.45 * params.mass_kg * params.gravity_mps2, ...
    'total_thrust_max_N', 1.95 * params.mass_kg * params.gravity_mps2, ...
    'min_body_z_up_gain', 0.35);
end

function ref = local_reference_at_time(t_s, profile)
%LOCAL_REFERENCE_AT_TIME Return altitude and pitch references at one instant.

ramp_duration_s = profile.safe_altitude_m / profile.climb_rate_mps;

if t_s < ramp_duration_s
    altitude_ref_m = profile.climb_rate_mps * t_s;
    vertical_speed_ref_mps = profile.climb_rate_mps;
else
    altitude_ref_m = profile.safe_altitude_m;
    vertical_speed_ref_mps = 0.0;
end

if t_s >= profile.pitch_step_time_s
    pitch_ref_rad = profile.pitch_cmd_rad;
else
    pitch_ref_rad = 0.0;
end

ref = struct();
ref.altitude_ref_m = altitude_ref_m;
ref.vertical_speed_ref_mps = vertical_speed_ref_mps;
ref.pitch_ref_rad = pitch_ref_rad;
end

function refs = local_make_reference_history(time_s, profile)
%LOCAL_MAKE_REFERENCE_HISTORY Sample the pitch-demo reference on the run grid.

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

step_indices = find(series.time_s >= profile.pitch_step_time_s);
if isempty(step_indices)
    step_indices = 1:numel(series.time_s);
end

metrics = struct();
metrics.final_pitch_deg = rad2deg(series.pitch_rad(end));
metrics.final_estimated_pitch_deg = rad2deg(series.pitch_est_rad(end));
metrics.final_pitch_estimation_error_deg = series.pitch_estimation_error_deg(end);
metrics.peak_pitch_error_deg = rad2deg(max(abs( ...
    series.pitch_ref_rad(step_indices) - series.pitch_rad(step_indices))));
metrics.max_pitch_estimation_error_deg = max(abs( ...
    series.pitch_estimation_error_deg(step_indices)));
metrics.min_accel_correction_weight = min(series.accel_correction_weight(step_indices));
metrics.max_accel_consistency_metric = max(series.accel_consistency_metric(step_indices));
metrics.final_altitude_m = series.altitude_m(end);
metrics.final_true_quat_norm = series.quat_norm_true(end);
metrics.final_estimated_quat_norm = series.quat_norm_est(end);
end
