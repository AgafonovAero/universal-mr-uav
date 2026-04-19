function series = postprocess_demo_log(log, refs)
%POSTPROCESS_DEMO_LOG Convert one estimator-backed run log into numeric histories.
% Description:
%   Expands the struct-array histories returned by
%   `uav.sim.run_case_with_estimator` into vectorized time series for demo
%   plotting, CSV export, and compact report generation.
%
% Inputs:
%   log  - struct returned by uav.sim.run_case_with_estimator
%   refs - optional struct with reference histories aligned with log.time_s
%
% Outputs:
%   series - struct with true, estimated, and reference histories
%
% Units:
%   SI only, angles in radians
%
% Assumptions:
%   The reference histories, when provided, are already sampled on the
%   same time grid as `log.time_s`.

if (nargin < 2 || isempty(refs)) && isfield(log, 'reference') && ...
        isstruct(log.reference) && numel(log.reference) == numel(log.time_s)
    refs = local_refs_from_log(log.reference);
elseif nargin < 2 || isempty(refs)
    refs = struct();
end

time_s = log.time_s(:);
n_samples = numel(time_s);

series = struct();
series.time_s = time_s;
series.altitude_ref_m = ...
    local_ref_or_default(refs, 'altitude_ref_m', n_samples, 0.0);
series.vertical_speed_ref_mps = local_ref_or_default(refs, ...
    'vertical_speed_ref_mps', n_samples, 0.0);
series.pitch_ref_rad = ...
    local_ref_or_default(refs, 'pitch_ref_rad', n_samples, 0.0);

series.altitude_m = zeros(n_samples, 1);
series.vertical_speed_mps = zeros(n_samples, 1);
series.pitch_rad = zeros(n_samples, 1);
series.roll_rad = zeros(n_samples, 1);
series.yaw_rad = zeros(n_samples, 1);
series.altitude_est_m = zeros(n_samples, 1);
series.vertical_speed_est_mps = zeros(n_samples, 1);
series.pitch_est_rad = zeros(n_samples, 1);
series.roll_est_rad = zeros(n_samples, 1);
series.yaw_est_rad = zeros(n_samples, 1);
series.accel_correction_weight = nan(n_samples, 1);
series.accel_consistency_metric = nan(n_samples, 1);
series.motor_speed_radps = zeros(n_samples, 4);
series.motor_cmd_radps = log.motor_cmd_radps;

if isfield(log, 'quat_norm_true')
    series.quat_norm_true = log.quat_norm_true(:);
else
    series.quat_norm_true = zeros(n_samples, 1);
end

if isfield(log, 'quat_norm_est')
    series.quat_norm_est = log.quat_norm_est(:);
else
    series.quat_norm_est = zeros(n_samples, 1);
end

for k = 1:n_samples
    state_k = log.state(k);
    est_k = log.estimator(k);

    c_nb = uav.core.quat_to_dcm(state_k.q_nb);
    v_ned_mps = c_nb * state_k.v_b_mps;
    euler_true_rad = local_quat_to_euler_rpy(state_k.q_nb);

    series.altitude_m(k) = -state_k.p_ned_m(3);
    series.vertical_speed_mps(k) = -v_ned_mps(3);
    series.roll_rad(k) = euler_true_rad(1);
    series.pitch_rad(k) = euler_true_rad(2);
    series.yaw_rad(k) = euler_true_rad(3);
    series.altitude_est_m(k) = est_k.alt_m;
    series.vertical_speed_est_mps(k) = est_k.vz_mps;
    series.roll_est_rad(k) = est_k.euler_rpy_rad(1);
    series.pitch_est_rad(k) = est_k.euler_rpy_rad(2);
    series.yaw_est_rad(k) = est_k.euler_rpy_rad(3);
    series.motor_speed_radps(k, :) = state_k.omega_m_radps(:).';

    if isfield(log, 'estimator_diag') && numel(log.estimator_diag) >= k && ...
            isfield(log.estimator_diag(k), 'attitude')
        att_diag_k = log.estimator_diag(k).attitude;
        if isfield(att_diag_k, 'accel_correction_weight')
            series.accel_correction_weight(k) = att_diag_k.accel_correction_weight;
        end
        if isfield(att_diag_k, 'accel_consistency_metric')
            series.accel_consistency_metric(k) = att_diag_k.accel_consistency_metric;
        end
    end

    if ~isfield(log, 'quat_norm_true')
        series.quat_norm_true(k) = norm(state_k.q_nb);
    end
    if ~isfield(log, 'quat_norm_est')
        series.quat_norm_est(k) = norm(est_k.q_nb);
    end
end

series.pitch_estimation_error_deg = ...
    rad2deg(series.pitch_rad - series.pitch_est_rad);
series.true_vs_est_pitch_deg = [ ...
    rad2deg(series.pitch_rad), rad2deg(series.pitch_est_rad)];
end

function ref_hist = local_ref_or_default(refs, field_name, n_samples, default_value)
%LOCAL_REF_OR_DEFAULT Return one reference history or expand a scalar default.

if isfield(refs, field_name) && ~isempty(refs.(field_name))
    ref_hist = refs.(field_name);
else
    ref_hist = default_value;
end

if isscalar(ref_hist)
    ref_hist = repmat(ref_hist, n_samples, 1);
else
    ref_hist = ref_hist(:);
    if numel(ref_hist) ~= n_samples
        error('uav:sim:postprocess_demo_log:ReferenceLength', ...
            'Expected refs.%s to have %d samples, got %d.', ...
            field_name, n_samples, numel(ref_hist));
    end
end
end

function refs = local_refs_from_log(reference_hist)
%LOCAL_REFS_FROM_LOG Extract numeric reference histories from log.reference.

n_samples = numel(reference_hist);
refs = struct();
refs.altitude_ref_m = local_hist_field_or_default(reference_hist, ...
    'altitude_ref_m', n_samples, 0.0);
refs.vertical_speed_ref_mps = local_hist_field_or_default(reference_hist, ...
    'vertical_speed_ref_mps', n_samples, 0.0);
refs.pitch_ref_rad = local_hist_field_or_default(reference_hist, ...
    'pitch_ref_rad', n_samples, 0.0);
end

function values = local_hist_field_or_default( ...
        hist, field_name, n_samples, default_value)
%LOCAL_HIST_FIELD_OR_DEFAULT Read one scalar field from a struct-array history.

if isempty(hist) || ~isfield(hist, field_name)
    values = repmat(default_value, n_samples, 1);
    return;
end

values = zeros(n_samples, 1);
for k = 1:n_samples
    values(k) = hist(k).(field_name);
end
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
