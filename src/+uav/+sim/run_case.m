function log = run_case(case_cfg)
%RUN_CASE Execute a time-domain engineering case on the Stage-1.5 kernel.
% Description:
%   Runs a transparent discrete-time simulation loop defined by case_cfg.
%   Motor commands are sampled through case_cfg.command_fun at each time
%   step and propagated with uav.sim.plant_step_struct.
%
% Inputs:
%   case_cfg - struct with params, state0, dt_s, t_final_s, and command_fun
%
% Outputs:
%   log - struct with time history, state history, commands, rotor speeds,
%         forces, moments, and quaternion norm
%
% Units:
%   SI only
%
% Assumptions:
%   t_final_s is an integer multiple of dt_s and command_fun is stateless.

case_cfg = local_validate_case_cfg(case_cfg);
n_steps = round(case_cfg.t_final_s / case_cfg.dt_s) + 1;
time_s = (0:(n_steps - 1)).' .* case_cfg.dt_s;

state_hist = repmat(case_cfg.state0, n_steps, 1);
motor_cmd_hist_radps = zeros(n_steps, 4);
omega_hist_radps = zeros(n_steps, 4);
forces_hist_b_N = zeros(n_steps, 3);
moments_hist_b_Nm = zeros(n_steps, 3);
quat_norm = zeros(n_steps, 1);

for k = 1:n_steps
    state_k = uav.core.state_validate(state_hist(k));
    snapshot = local_snapshot_diag(state_k, case_cfg.params);

    omega_hist_radps(k, :) = state_k.omega_m_radps.';
    forces_hist_b_N(k, :) = snapshot.forces_b_N.';
    moments_hist_b_Nm(k, :) = snapshot.moments_b_Nm.';
    quat_norm(k) = snapshot.quat_norm;

    motor_cmd_k_radps = case_cfg.command_fun(time_s(k), state_k, case_cfg.params);
    motor_cmd_k_radps = motor_cmd_k_radps(:);
    if numel(motor_cmd_k_radps) ~= 4
        error('uav:sim:run_case:CommandSize', ...
            'Expected command_fun to return 4 motor commands.');
    end
    motor_cmd_hist_radps(k, :) = motor_cmd_k_radps.';

    if k < n_steps
        [state_hist(k + 1), ~] = uav.sim.plant_step_struct( ...
            state_k, motor_cmd_k_radps, case_cfg.dt_s, case_cfg.params);
    end
end

log = struct();
log.time_s = time_s;
log.state = state_hist;
log.motor_cmd_radps = motor_cmd_hist_radps;
log.omega_m_radps = omega_hist_radps;
log.forces_b_N = forces_hist_b_N;
log.moments_b_Nm = moments_hist_b_Nm;
log.quat_norm = quat_norm;
end

function case_cfg = local_validate_case_cfg(case_cfg)
%LOCAL_VALIDATE_CASE_CFG Validate the run_case configuration.

if ~isstruct(case_cfg) || ~isscalar(case_cfg)
    error('uav:sim:run_case:CaseCfgType', ...
        'Expected case_cfg to be a scalar struct.');
end

required_fields = {'params', 'state0', 'dt_s', 't_final_s', 'command_fun'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(case_cfg, field_name)
        error('uav:sim:run_case:MissingField', ...
            'Missing required case_cfg field "%s".', field_name);
    end
end

if ~isa(case_cfg.command_fun, 'function_handle')
    error('uav:sim:run_case:CommandFunType', ...
        'Expected case_cfg.command_fun to be a function handle.');
end

validateattributes(case_cfg.dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'case_cfg.dt_s');
validateattributes(case_cfg.t_final_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'case_cfg.t_final_s');

n_intervals = round(case_cfg.t_final_s / case_cfg.dt_s);
if abs(n_intervals * case_cfg.dt_s - case_cfg.t_final_s) > 1.0e-12
    error('uav:sim:run_case:TimeGrid', ...
        'Expected t_final_s to be an integer multiple of dt_s.');
end

case_cfg.state0 = uav.core.state_validate(case_cfg.state0);
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build log diagnostics for one sampled state.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end
