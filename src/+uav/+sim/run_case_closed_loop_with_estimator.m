function log = run_case_closed_loop_with_estimator(case_cfg)
%RUN_CASE_CLOSED_LOOP_WITH_ESTIMATOR Execute estimator-driven closed loop.
% Description:
%   Runs a transparent discrete-time simulation loop where plant state is
%   sampled by the sensor layer, converted into estimator outputs, and only
%   then exposed to the demo controller. The controller never receives the
%   true state directly.
%
% Inputs:
%   case_cfg - struct with params, state0, dt_s, t_final_s, and
%              controller_fun; optional reference_fun and controller_state0
%
% Outputs:
%   log - struct with plant, sensor, estimator, reference, and controller
%         histories together with diagnostics
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   t_final_s is an integer multiple of dt_s and controller_fun is
%   deterministic for a given controller-input struct.

case_cfg = local_validate_case_cfg(case_cfg);
n_steps = round(case_cfg.t_final_s / case_cfg.dt_s) + 1;
time_s = (0:(n_steps - 1)).' .* case_cfg.dt_s;

state_hist = repmat(case_cfg.state0, n_steps, 1);
motor_cmd_hist_radps = zeros(n_steps, 4);
quat_norm_true = zeros(n_steps, 1);
quat_norm_est = zeros(n_steps, 1);
sensor_hist = repmat(local_empty_sensor_sample(), n_steps, 1);
estimator_hist = repmat(local_empty_estimator_sample(), n_steps, 1);

reference_hist = struct([]);
estimator_diag_hist = repmat(local_empty_estimator_diag_sample(), n_steps, 1);
controller_diag_hist = struct([]);
controller_state_hist = struct([]);

% Initialize estimator/controller state before the main simulation loop.
est_prev = local_empty_estimator_sample();
controller_state = case_cfg.controller_state0;
is_initialized = false;

for k = 1:n_steps
    % Sample the current plant state through the code-centric sensor layer.
    state_k = uav.core.state_validate(state_hist(k));
    snapshot = local_snapshot_diag(state_k, case_cfg.params);
    sens_k = uav.sensors.sensors_step(state_k, snapshot, case_cfg.params);

    % Update the estimator strictly from sensor outputs.
    if ~is_initialized
        est_prev = uav.est.estimator_init(case_cfg.params, sens_k);
        dt_est_s = 0.0;
        is_initialized = true;
    else
        dt_est_s = case_cfg.dt_s;
    end

    [est_k, est_diag_k] = uav.est.estimator_step( ...
        est_prev, sens_k, dt_est_s, case_cfg.params);

    % Build controller inputs only from estimator outputs and references.
    ref_k = case_cfg.reference_fun(time_s(k), est_k, sens_k, case_cfg.params);
    ctrl_input = struct( ...
        'time_s', time_s(k), ...
        'sensors', sens_k, ...
        'estimator', est_k, ...
        'reference', ref_k);
    [motor_cmd_k_radps, controller_state, controller_diag_k] = ...
        case_cfg.controller_fun(ctrl_input, controller_state, ...
            case_cfg.dt_s, case_cfg.params);

    motor_cmd_k_radps = motor_cmd_k_radps(:);
    if numel(motor_cmd_k_radps) ~= 4
        error('uav:sim:run_case_closed_loop_with_estimator:CommandSize', ...
            'Expected controller_fun to return 4 motor commands.');
    end

    if k == 1
        reference_hist = repmat(ref_k, n_steps, 1);
        controller_diag_hist = repmat(controller_diag_k, n_steps, 1);
        controller_state_hist = repmat(controller_state, n_steps, 1);
    end

    % Store all histories needed for postprocessing and verification.
    sensor_hist(k) = sens_k;
    estimator_hist(k) = est_k;
    reference_hist(k) = ref_k;
    estimator_diag_hist(k) = est_diag_k;
    controller_diag_hist(k) = controller_diag_k;
    controller_state_hist(k) = controller_state;
    quat_norm_true(k) = snapshot.quat_norm;
    quat_norm_est(k) = norm(est_k.q_nb);
    motor_cmd_hist_radps(k, :) = motor_cmd_k_radps.';

    % Advance the plant if the simulation horizon is not finished yet.
    if k < n_steps
        [state_hist(k + 1), ~] = uav.sim.plant_step_struct( ...
            state_k, motor_cmd_k_radps, case_cfg.dt_s, case_cfg.params);
    end

    est_prev = est_k;
end

% Pack the final run log in one compact struct.
log = struct();
log.time_s = time_s;
log.state = state_hist;
log.sensors = sensor_hist;
log.estimator = estimator_hist;
log.reference = reference_hist;
log.estimator_diag = estimator_diag_hist;
log.controller_diag = controller_diag_hist;
log.controller_state = controller_state_hist;
log.quat_norm_true = quat_norm_true;
log.quat_norm_est = quat_norm_est;
log.motor_cmd_radps = motor_cmd_hist_radps;
end

function case_cfg = local_validate_case_cfg(case_cfg)
%LOCAL_VALIDATE_CASE_CFG Validate the closed-loop runner configuration.

if ~isstruct(case_cfg) || ~isscalar(case_cfg)
    error('uav:sim:run_case_closed_loop_with_estimator:CaseCfgType', ...
        'Expected case_cfg to be a scalar struct.');
end

required_fields = {'params', 'state0', 'dt_s', 't_final_s', 'controller_fun'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(case_cfg, field_name)
        error('uav:sim:run_case_closed_loop_with_estimator:MissingField', ...
            'Missing required case_cfg field "%s".', field_name);
    end
end

if ~isa(case_cfg.controller_fun, 'function_handle')
    error('uav:sim:run_case_closed_loop_with_estimator:ControllerFunType', ...
        'Expected case_cfg.controller_fun to be a function handle.');
end

if ~isfield(case_cfg, 'reference_fun') || ...
        isempty(case_cfg.reference_fun)
    case_cfg.reference_fun = @local_default_reference_fun;
elseif ~isa(case_cfg.reference_fun, 'function_handle')
    error('uav:sim:run_case_closed_loop_with_estimator:ReferenceFunType', ...
        'Expected case_cfg.reference_fun to be a function handle.');
end

if ~isfield(case_cfg, 'controller_state0') || ...
        isempty(case_cfg.controller_state0)
    case_cfg.controller_state0 = struct();
elseif ~isstruct(case_cfg.controller_state0) || ...
        ~isscalar(case_cfg.controller_state0)
    error('uav:sim:run_case_closed_loop_with_estimator:ControllerStateType', ...
        'Expected case_cfg.controller_state0 to be a scalar struct.');
end

validateattributes(case_cfg.dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'case_cfg.dt_s');
validateattributes(case_cfg.t_final_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'case_cfg.t_final_s');

n_intervals = round(case_cfg.t_final_s / case_cfg.dt_s);
if abs(n_intervals * case_cfg.dt_s - case_cfg.t_final_s) > 1.0e-12
    error('uav:sim:run_case_closed_loop_with_estimator:TimeGrid', ...
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

function diag = local_empty_estimator_diag_sample()
%LOCAL_EMPTY_ESTIMATOR_DIAG_SAMPLE Preallocate estimator diagnostics.

diag = struct();
diag.attitude = struct( ...
    'quat_norm', 1.0, ...
    'accel_correction_weight', 0.0, ...
    'accel_consistency_metric', nan, ...
    'acc_correction_norm', 0.0, ...
    'mag_correction_norm', 0.0);
diag.altitude = struct( ...
    'az_ned_mps2', 0.0, ...
    'baro_residual_m', 0.0);
end

function ref = local_default_reference_fun(~, ~, ~, ~)
%LOCAL_DEFAULT_REFERENCE_FUN Return one empty reference struct by default.

ref = struct();
end
