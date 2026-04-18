function log = run_case_with_estimator(case_cfg)
%RUN_CASE_WITH_ESTIMATOR Execute a Stage-1.5 case with sensor and estimator logs.
% Description:
%   Runs the transparent discrete-time case loop, samples the sensor layer,
%   and advances the minimal estimator layer on top of the existing plant
%   and sensor models.
%
% Inputs:
%   case_cfg - struct with params, state0, dt_s, t_final_s, and command_fun
%
% Outputs:
%   log - struct with plant, sensor, and estimator histories
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   t_final_s is an integer multiple of dt_s and command_fun is stateless.

case_cfg = local_validate_case_cfg(case_cfg);
n_steps = round(case_cfg.t_final_s / case_cfg.dt_s) + 1;
time_s = (0:(n_steps - 1)).' .* case_cfg.dt_s;

state_hist = repmat(case_cfg.state0, n_steps, 1);
motor_cmd_hist_radps = zeros(n_steps, 4);
quat_norm_true = zeros(n_steps, 1);
quat_norm_est = zeros(n_steps, 1);
sensor_hist = repmat(local_empty_sensor_sample(), n_steps, 1);
estimator_hist = repmat(local_empty_estimator_sample(), n_steps, 1);

est_prev = local_empty_estimator_sample();
is_initialized = false;

for k = 1:n_steps
    state_k = uav.core.state_validate(state_hist(k));
    snapshot = local_snapshot_diag(state_k, case_cfg.params);
    sens_k = uav.sensors.sensors_step(state_k, snapshot, case_cfg.params);

    if ~is_initialized
        est_prev = uav.est.estimator_init(case_cfg.params, sens_k);
        is_initialized = true;
        dt_est_s = 0.0;
    else
        dt_est_s = case_cfg.dt_s;
    end

    [est_k, ~] = uav.est.estimator_step(est_prev, sens_k, dt_est_s, case_cfg.params);

    sensor_hist(k) = sens_k;
    estimator_hist(k) = est_k;
    quat_norm_true(k) = snapshot.quat_norm;
    quat_norm_est(k) = norm(est_k.q_nb);

    motor_cmd_k_radps = case_cfg.command_fun(time_s(k), state_k, case_cfg.params);
    motor_cmd_k_radps = motor_cmd_k_radps(:);
    if numel(motor_cmd_k_radps) ~= 4
        error('uav:sim:run_case_with_estimator:CommandSize', ...
            'Expected command_fun to return 4 motor commands.');
    end
    motor_cmd_hist_radps(k, :) = motor_cmd_k_radps.';

    if k < n_steps
        [state_hist(k + 1), ~] = uav.sim.plant_step_struct( ...
            state_k, motor_cmd_k_radps, case_cfg.dt_s, case_cfg.params);
    end

    est_prev = est_k;
end

log = struct();
log.time_s = time_s;
log.state = state_hist;
log.sensors = sensor_hist;
log.estimator = estimator_hist;
log.quat_norm_true = quat_norm_true;
log.quat_norm_est = quat_norm_est;
log.motor_cmd_radps = motor_cmd_hist_radps;
end

function case_cfg = local_validate_case_cfg(case_cfg)
%LOCAL_VALIDATE_CASE_CFG Validate the run_case_with_estimator configuration.

if ~isstruct(case_cfg) || ~isscalar(case_cfg)
    error('uav:sim:run_case_with_estimator:CaseCfgType', ...
        'Expected case_cfg to be a scalar struct.');
end

required_fields = {'params', 'state0', 'dt_s', 't_final_s', 'command_fun'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(case_cfg, field_name)
        error('uav:sim:run_case_with_estimator:MissingField', ...
            'Missing required case_cfg field "%s".', field_name);
    end
end

if ~isa(case_cfg.command_fun, 'function_handle')
    error('uav:sim:run_case_with_estimator:CommandFunType', ...
        'Expected case_cfg.command_fun to be a function handle.');
end

validateattributes(case_cfg.dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'case_cfg.dt_s');
validateattributes(case_cfg.t_final_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'case_cfg.t_final_s');

n_intervals = round(case_cfg.t_final_s / case_cfg.dt_s);
if abs(n_intervals * case_cfg.dt_s - case_cfg.t_final_s) > 1.0e-12
    error('uav:sim:run_case_with_estimator:TimeGrid', ...
        'Expected t_final_s to be an integer multiple of dt_s.');
end

case_cfg.state0 = uav.core.state_validate(case_cfg.state0);
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build sampled diagnostics for one state snapshot.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function sens = local_empty_sensor_sample()
%LOCAL_EMPTY_SENSOR_SAMPLE Preallocate one sensor history sample.

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', zeros(3, 1), ...
    'gyro_b_radps', zeros(3, 1));
sens.baro = struct( ...
    'alt_m', 0.0, ...
    'pressure_pa', 0.0);
sens.mag = struct( ...
    'field_b_uT', zeros(3, 1));
sens.gnss = struct( ...
    'pos_ned_m', zeros(3, 1), ...
    'vel_ned_mps', zeros(3, 1));
end

function est = local_empty_estimator_sample()
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Preallocate one estimator history sample.

est = struct();
est.attitude = struct( ...
    'q_nb', [1.0; 0.0; 0.0; 0.0], ...
    'euler_rpy_rad', zeros(3, 1));
est.altitude = struct( ...
    'alt_m', 0.0, ...
    'vz_mps', 0.0);
est.q_nb = est.attitude.q_nb;
est.euler_rpy_rad = est.attitude.euler_rpy_rad;
est.alt_m = est.altitude.alt_m;
est.vz_mps = est.altitude.vz_mps;
end
