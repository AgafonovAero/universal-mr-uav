%% RUN_DEMO_TAKEOFF_TO_50M Execute a deterministic takeoff-and-hold demo.
% Description:
%   Runs a reproducible climb to 50 m with altitude hold on top of
%   `uav.sim.run_case_with_estimator`, saves MAT/CSV artifacts, and prints
%   compact end-of-run diagnostics.
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
%   Sensor noise is disabled and the altitude controller is a thin demo
%   law around the existing code-centric kernel.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
if ~exist(reports_dir, 'dir')
    mkdir(reports_dir);
end

params = uav.sim.make_deterministic_demo_params();
profile = local_make_takeoff_profile(params);

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = profile.t_final_s;
case_cfg.command_fun = @(t_s, state, params_local) local_takeoff_command( ...
    t_s, state, params_local, profile);

log = uav.sim.run_case_with_estimator(case_cfg);
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
fprintf('  final estimated altitude [m]: %.6f\n', metrics.final_estimated_altitude_m);
fprintf('  final quat norms [-]        : true=%.12f est=%.12f\n', ...
    metrics.final_true_quat_norm, metrics.final_estimated_quat_norm);
fprintf('  saved MAT                   : %s\n', mat_file);
fprintf('  saved CSV                   : %s\n', csv_file);

function profile = local_make_takeoff_profile(params)
%LOCAL_MAKE_TAKEOFF_PROFILE Build the deterministic takeoff-demo settings.
% Description:
%   Collects the altitude-reference and controller parameters for the 50 m
%   takeoff scenario.
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
profile.altitude_kp_per_s2 = 0.55;
profile.vertical_speed_kp_per_s = 1.40;
profile.up_accel_limit_mps2 = 3.5;
profile.total_thrust_min_N = 0.40 * params.mass_kg * params.gravity_mps2;
profile.total_thrust_max_N = 1.80 * params.mass_kg * params.gravity_mps2;
profile.t_final_s = 24.0;
end

function motor_cmd_radps = local_takeoff_command(t_s, state, params, profile)
%LOCAL_TAKEOFF_COMMAND Compute the motor command for the takeoff demo.
% Description:
%   Tracks a ramp-to-hold altitude reference with a simple vertical PD law
%   while keeping zero body moments.
%
% Inputs:
%   t_s     - simulation time [s]
%   state   - canonical plant state struct
%   params  - baseline parameter struct
%   profile - takeoff profile struct
%
% Outputs:
%   motor_cmd_radps - 4x1 motor speed command [rad/s]
%
% Units:
%   SI only
%
% Assumptions:
%   The scenario remains near-level so zero body moments are sufficient.

[altitude_ref_m, vertical_speed_ref_mps] = local_reference_at_time(t_s, profile);
[altitude_m, vertical_speed_mps] = local_altitude_and_vertical_speed(state);

up_accel_cmd_mps2 = profile.altitude_kp_per_s2 * ...
    (altitude_ref_m - altitude_m) + ...
    profile.vertical_speed_kp_per_s * ...
    (vertical_speed_ref_mps - vertical_speed_mps);
up_accel_cmd_mps2 = local_clip(up_accel_cmd_mps2, ...
    -profile.up_accel_limit_mps2, profile.up_accel_limit_mps2);

body_z_up_gain = local_body_z_up_gain(state.q_nb);
total_thrust_N = params.mass_kg * ...
    (params.gravity_mps2 + up_accel_cmd_mps2) ./ body_z_up_gain;
total_thrust_N = local_clip(total_thrust_N, ...
    profile.total_thrust_min_N, profile.total_thrust_max_N);

[motor_cmd_radps, ~] = uav.vmg.mixer_quad_x(total_thrust_N, zeros(3, 1), params);
motor_cmd_radps = local_clip(motor_cmd_radps, ...
    params.motor.omega_min_radps, params.motor.omega_max_radps);
end

function refs = local_make_reference_history(time_s, profile)
%LOCAL_MAKE_REFERENCE_HISTORY Sample the takeoff reference on the run grid.

n_samples = numel(time_s);
refs = struct();
refs.altitude_ref_m = zeros(n_samples, 1);
refs.vertical_speed_ref_mps = zeros(n_samples, 1);
refs.pitch_ref_rad = zeros(n_samples, 1);

for k = 1:n_samples
    [refs.altitude_ref_m(k), refs.vertical_speed_ref_mps(k)] = ...
        local_reference_at_time(time_s(k), profile);
end
end

function [altitude_ref_m, vertical_speed_ref_mps] = local_reference_at_time(t_s, profile)
%LOCAL_REFERENCE_AT_TIME Return the altitude reference at one time instant.

ramp_duration_s = profile.target_altitude_m / profile.climb_rate_mps;

if t_s < ramp_duration_s
    altitude_ref_m = profile.climb_rate_mps * t_s;
    vertical_speed_ref_mps = profile.climb_rate_mps;
else
    altitude_ref_m = profile.target_altitude_m;
    vertical_speed_ref_mps = 0.0;
end
end

function [altitude_m, vertical_speed_mps] = local_altitude_and_vertical_speed(state)
%LOCAL_ALTITUDE_AND_VERTICAL_SPEED Extract altitude and vertical speed.

c_nb = uav.core.quat_to_dcm(state.q_nb);
v_ned_mps = c_nb * state.v_b_mps;
altitude_m = -state.p_ned_m(3);
vertical_speed_mps = -v_ned_mps(3);
end

function gain = local_body_z_up_gain(q_nb)
%LOCAL_BODY_Z_UP_GAIN Return the vertical thrust projection factor.

c_nb = uav.core.quat_to_dcm(q_nb);
gain = max(c_nb(3, 3), 0.35);
end

function value = local_clip(value, min_value, max_value)
%LOCAL_CLIP Saturate a scalar or vector between two bounds.

value = min(max(value, min_value), max_value);
end

function metrics = local_make_metrics(series, profile)
%LOCAL_MAKE_METRICS Build compact scalar diagnostics for reporting.

metrics = struct();
metrics.final_altitude_m = series.altitude_m(end);
metrics.peak_altitude_error_m = max(abs(series.altitude_ref_m - series.altitude_m));
metrics.final_estimated_altitude_m = series.altitude_est_m(end);
metrics.final_true_quat_norm = series.quat_norm_true(end);
metrics.final_estimated_quat_norm = series.quat_norm_est(end);
metrics.final_altitude_reference_m = profile.target_altitude_m;
end
