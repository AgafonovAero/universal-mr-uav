%% BUILD_SIL_TOP Programmatically build the thin SIL-prep Simulink shell.
% Description:
%   Recreates the minimal Simulink top model used to close the boundary
%   between a future external flight stack and the existing code-centric
%   kernel. The model contains only two MATLAB System blocks and logging
%   sinks for the main bridge outputs.
%
% Inputs:
%   none
%
% Outputs:
%   sil_top_build - assigned in base workspace with build metadata
%
% Units:
%   not applicable
%
% Assumptions:
%   Simulink is installed and the repository is available locally.

repo_root = fileparts(fileparts(mfilename('fullpath')));
models_dir = fullfile(repo_root, 'models');
model_name = 'sil_top';
model_file = fullfile(models_dir, [model_name '.slx']);

if ~exist(models_dir, 'dir')
    mkdir(models_dir);
end

params = local_get_base_or_default('sil_params', uav.sim.default_params_quad_x250());
dt_s = local_get_base_or_default('sil_dt_s', params.demo.dt_s);
t_final_s = local_get_base_or_default('sil_t_final_s', params.demo.case_hover_t_final_s);
state0 = local_get_base_or_default('sil_state0', ...
    uav.core.state_unpack(params.demo.initial_state_plant));
rates_cfg = local_get_base_or_default('sil_rates_cfg', ...
    local_default_rates_cfg(params, dt_s));
stub_mode = local_get_base_or_default('sil_stub_mode', "hover");
actuation_mode = local_get_base_or_default('sil_actuation_mode', "norm01");
time0_s = local_get_base_or_default('sil_time_s', 0.0);
initial_sensor_packet = local_get_base_or_default('sil_initial_sensor_packet', ...
    local_initial_sensor_packet());

assignin('base', 'sil_params', params);
assignin('base', 'sil_dt_s', dt_s);
assignin('base', 'sil_t_final_s', t_final_s);
assignin('base', 'sil_state0', state0);
assignin('base', 'sil_rates_cfg', rates_cfg);
assignin('base', 'sil_stub_mode', stub_mode);
assignin('base', 'sil_actuation_mode', actuation_mode);
assignin('base', 'sil_time_s', time0_s);
assignin('base', 'sil_initial_sensor_packet', initial_sensor_packet);
assignin('base', 'sil_model_name', model_name);
assignin('base', 'sil_model_file', model_file);

bus_defs = uav.sl.make_sil_bus_defs();
assignin('base', 'sil_bus_defs', bus_defs);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

if exist(model_file, 'file') == 2
    delete(model_file);
end

load_system('simulink');
new_system(model_name, 'Model');

set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', local_scalar_string(dt_s), ...
    'StopTime', 'sil_t_final_s', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on');

stub_block = add_block( ...
    'simulink/User-Defined Functions/MATLAB System', ...
    [model_name '/stub_external_fcs'], ...
    'Position', [70 90 250 160]);

set_param(stub_block, ...
    'System', 'uav.sl.StubExternalFCSSystem', ...
    'SimulateUsing', 'Interpreted execution', ...
    'mode_name', char(string(stub_mode)), ...
    'params', 'sil_params');

bridge_block = add_block( ...
    'simulink/User-Defined Functions/MATLAB System', ...
    [model_name '/sil_bridge'], ...
    'Position', [330 70 550 180]);

set_param(bridge_block, ...
    'System', 'uav.sl.Stage15SILBridgeSystem', ...
    'SimulateUsing', 'Interpreted execution', ...
    'dt_s', 'sil_dt_s', ...
    'params', 'sil_params', ...
    'state0', 'sil_state0', ...
    'rates_cfg', 'sil_rates_cfg', ...
    'time_s', 'sil_time_s', ...
    'actuation_mode', char(string(actuation_mode)));

feedback_delay_block = add_block( ...
    'simulink/Discrete/Unit Delay', ...
    [model_name '/sensor_packet_feedback_delay'], ...
    'Position', [610 240 690 280], ...
    'SampleTime', 'sil_dt_s', ...
    'InitialCondition', 'sil_initial_sensor_packet');

sensor_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/sensor_packet_out'], ...
    'Position', [660 55 690 75]);

truth_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/truth_out'], ...
    'Position', [660 110 690 130]);

diag_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/diag_out'], ...
    'Position', [660 165 690 185]);

add_line(model_name, 'stub_external_fcs/1', 'sil_bridge/1', 'autorouting', 'on');
add_line(model_name, 'sil_bridge/1', 'sensor_packet_feedback_delay/1', 'autorouting', 'on');
add_line(model_name, 'sensor_packet_feedback_delay/1', 'stub_external_fcs/1', 'autorouting', 'on');
add_line(model_name, 'sil_bridge/1', 'sensor_packet_out/1', 'autorouting', 'on');
add_line(model_name, 'sil_bridge/2', 'truth_out/1', 'autorouting', 'on');
add_line(model_name, 'sil_bridge/3', 'diag_out/1', 'autorouting', 'on');

set_param(model_name, 'SimulationCommand', 'update');
save_system(model_name, model_file);
close_system(model_name, 0);

sil_top_build = struct();
sil_top_build.model_name = model_name;
sil_top_build.model_file = model_file;
sil_top_build.stub_block = [model_name '/stub_external_fcs'];
sil_top_build.bridge_block = [model_name '/sil_bridge'];
sil_top_build.feedback_delay_block = [model_name '/sensor_packet_feedback_delay'];
sil_top_build.sensor_log_block = sensor_log_block;
sil_top_build.truth_log_block = truth_log_block;
sil_top_build.diag_log_block = diag_log_block;
sil_top_build.bus_defs = bus_defs;

assignin('base', 'sil_top_build', sil_top_build);

fprintf('Built thin SIL-prep model: %s\n', model_file);

function value = local_get_base_or_default(var_name, default_value)
%LOCAL_GET_BASE_OR_DEFAULT Read a base-workspace variable when available.
% Description:
%   Returns a base workspace variable if it already exists; otherwise
%   returns the supplied default value.
%
% Inputs:
%   var_name      - variable name in base workspace
%   default_value - fallback value
%
% Outputs:
%   value - resolved value
%
% Units:
%   not applicable
%
% Assumptions:
%   Base workspace access is acceptable for thin-shell orchestration.

if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    value = evalin('base', var_name);
else
    value = default_value;
end
end

function rates_cfg = local_default_rates_cfg(params, dt_s)
%LOCAL_DEFAULT_RATES_CFG Build one default SIL scheduler config.
% Description:
%   Uses params.sil.rates when available and otherwise falls back to the
%   minimal TASK-08 scheduler defaults.
%
% Inputs:
%   params - parameter struct
%   dt_s   - base discrete step [s]
%
% Outputs:
%   rates_cfg - scheduler-rate config struct
%
% Units:
%   seconds
%
% Assumptions:
%   The fallback rates match the Stage-1.5+ SIL-prep requirement.

if isstruct(params) && isfield(params, 'sil') && isstruct(params.sil) && ...
        isfield(params.sil, 'rates') && isstruct(params.sil.rates)
    rates_cfg = params.sil.rates;
    rates_cfg.base_dt_s = dt_s;
else
    rates_cfg = struct( ...
        'base_dt_s', dt_s, ...
        'baro_period_s', 0.02, ...
        'mag_period_s', 0.02, ...
        'gnss_period_s', 0.1);
end
end

function text = local_scalar_string(value)
%LOCAL_SCALAR_STRING Convert one scalar to a stable text representation.
% Description:
%   Formats a scalar numeric value so it can be written into model
%   parameters without losing precision.
%
% Inputs:
%   value - scalar numeric value
%
% Outputs:
%   text - formatted scalar string
%
% Units:
%   not applicable
%
% Assumptions:
%   Value is finite.

text = sprintf('%.16g', value);
end

function packet = local_initial_sensor_packet()
%LOCAL_INITIAL_SENSOR_PACKET Build the feedback-delay initial packet value.
% Description:
%   Creates a deterministic zero-valued sensor packet used only as the
%   initial condition of the feedback delay that breaks the algebraic loop.
%
% Inputs:
%   none
%
% Outputs:
%   packet - canonical external sensor packet struct
%
% Units:
%   SI only
%
% Assumptions:
%   The initial packet is used only before the first bridge sample exists.

packet = struct();
packet.time_s = 0.0;
packet.imu_valid = false;
packet.imu = struct( ...
    'accel_b_mps2', zeros(3, 1), ...
    'gyro_b_radps', zeros(3, 1));
packet.baro_valid = false;
packet.baro = struct( ...
    'alt_m', 0.0, ...
    'pressure_pa', 0.0);
packet.mag_valid = false;
packet.mag = struct( ...
    'field_b_uT', zeros(3, 1));
packet.gnss_valid = false;
packet.gnss = struct( ...
    'pos_ned_m', zeros(3, 1), ...
    'vel_ned_mps', zeros(3, 1));
end
