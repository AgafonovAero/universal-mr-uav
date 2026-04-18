%% RUN_DEMO_PITCH_STEP_MINUS10DEG Execute a deterministic safe-altitude pitch demo.
% Description:
%   Climbs to a safe altitude, commands a -10 deg pitch step, holds the
%   mode on top of `uav.sim.run_case_with_estimator`, and saves MAT/CSV
%   artifacts for plotting and reporting.
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
%   Sensor noise is disabled and the demo uses a minimal true-state
%   feedback law around the existing code-centric kernel.

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
case_cfg.command_fun = @(t_s, state, params_local) local_pitch_command( ...
    t_s, state, params_local, profile);

log = uav.sim.run_case_with_estimator(case_cfg);
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
fprintf('  final pitch [deg]           : %.6f\n', metrics.final_pitch_deg);
fprintf('  final estimated pitch [deg] : %.6f\n', metrics.final_estimated_pitch_deg);
fprintf('  peak pitch error [deg]      : %.6f\n', metrics.peak_pitch_error_deg);
fprintf('  final altitude [m]          : %.6f\n', metrics.final_altitude_m);
fprintf('  quat norms [-]              : true=%.12f est=%.12f\n', ...
    metrics.final_true_quat_norm, metrics.final_estimated_quat_norm);
fprintf('  saved MAT                   : %s\n', mat_file);
fprintf('  saved CSV                   : %s\n', csv_file);

function profile = local_make_pitch_profile(params)
%LOCAL_MAKE_PITCH_PROFILE Build the deterministic pitch-demo settings.
% Description:
%   Collects the altitude and pitch reference settings together with the
%   thin demo controller gains for the -10 deg pitch scenario.
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
profile.altitude_kp_per_s2 = 0.60;
profile.vertical_speed_kp_per_s = 1.45;
profile.up_accel_limit_mps2 = 3.0;
profile.pitch_kp_Nm_per_rad = 0.20;
profile.pitch_kd_Nm_per_radps = 0.04;
profile.roll_kp_Nm_per_rad = 0.06;
profile.roll_kd_Nm_per_radps = 0.02;
profile.yaw_kp_Nm_per_rad = 0.03;
profile.yaw_kd_Nm_per_radps = 0.01;
profile.body_moment_limit_Nm = 0.06;
profile.total_thrust_min_N = 0.45 * params.mass_kg * params.gravity_mps2;
profile.total_thrust_max_N = 1.90 * params.mass_kg * params.gravity_mps2;
profile.t_final_s = 18.0;
end

function motor_cmd_radps = local_pitch_command(t_s, state, params, profile)
%LOCAL_PITCH_COMMAND Compute the motor command for the pitch-step demo.
% Description:
%   Tracks the safe-altitude reference and applies a bounded pitch moment
%   command for the -10 deg step while damping roll and yaw drift.
%
% Inputs:
%   t_s     - simulation time [s]
%   state   - canonical plant state struct
%   params  - baseline parameter struct
%   profile - pitch demo profile
%
% Outputs:
%   motor_cmd_radps - 4x1 motor speed command [rad/s]
%
% Units:
%   SI only
%
% Assumptions:
%   Body rates stay moderate and the demo remains inside small-angle
%   bounds around the commanded pitch.

[altitude_ref_m, vertical_speed_ref_mps, pitch_ref_rad] = ...
    local_reference_at_time(t_s, profile);
[altitude_m, vertical_speed_mps] = local_altitude_and_vertical_speed(state);
euler_rpy_rad = local_quat_to_euler_rpy(state.q_nb);

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

body_moments_Nm = [ ...
    -profile.roll_kp_Nm_per_rad * euler_rpy_rad(1) - ...
        profile.roll_kd_Nm_per_radps * state.w_b_radps(1); ...
    profile.pitch_kp_Nm_per_rad * (pitch_ref_rad - euler_rpy_rad(2)) - ...
        profile.pitch_kd_Nm_per_radps * state.w_b_radps(2); ...
    -profile.yaw_kp_Nm_per_rad * euler_rpy_rad(3) - ...
        profile.yaw_kd_Nm_per_radps * state.w_b_radps(3)];
body_moments_Nm = local_clip(body_moments_Nm, ...
    -profile.body_moment_limit_Nm, profile.body_moment_limit_Nm);

[motor_cmd_radps, ~] = uav.vmg.mixer_quad_x(total_thrust_N, body_moments_Nm, params);
motor_cmd_radps = local_clip(motor_cmd_radps, ...
    params.motor.omega_min_radps, params.motor.omega_max_radps);
end

function refs = local_make_reference_history(time_s, profile)
%LOCAL_MAKE_REFERENCE_HISTORY Sample the pitch-demo reference on the run grid.

n_samples = numel(time_s);
refs = struct();
refs.altitude_ref_m = zeros(n_samples, 1);
refs.vertical_speed_ref_mps = zeros(n_samples, 1);
refs.pitch_ref_rad = zeros(n_samples, 1);

for k = 1:n_samples
    [refs.altitude_ref_m(k), refs.vertical_speed_ref_mps(k), ...
        refs.pitch_ref_rad(k)] = local_reference_at_time(time_s(k), profile);
end
end

function [altitude_ref_m, vertical_speed_ref_mps, pitch_ref_rad] = local_reference_at_time(t_s, profile)
%LOCAL_REFERENCE_AT_TIME Return altitude and pitch references at one time instant.

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

function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
%LOCAL_QUAT_TO_EULER_RPY Convert q_nb into roll/pitch/yaw angles.

q_nb = uav.core.quat_normalize(q_nb);

qw = q_nb(1);
qx = q_nb(2);
qy = q_nb(3);
qz = q_nb(4);

sin_pitch = 2.0 .* (qw * qy - qz * qx);

euler_rpy_rad = [ ...
    atan2(2.0 .* (qw * qx + qy * qz), 1.0 - 2.0 .* (qx^2 + qy^2)); ...
    asin(max(min(sin_pitch, 1.0), -1.0)); ...
    atan2(2.0 .* (qw * qz + qx * qy), 1.0 - 2.0 .* (qy^2 + qz^2))];
end

function value = local_clip(value, min_value, max_value)
%LOCAL_CLIP Saturate a scalar or vector between two bounds.

value = min(max(value, min_value), max_value);
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
metrics.peak_pitch_error_deg = rad2deg(max(abs( ...
    series.pitch_ref_rad(step_indices) - series.pitch_rad(step_indices))));
metrics.final_altitude_m = series.altitude_m(end);
metrics.final_true_quat_norm = series.quat_norm_true(end);
metrics.final_estimated_quat_norm = series.quat_norm_est(end);
end
