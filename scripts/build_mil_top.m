%% BUILD_MIL_TOP Programmatically build the thin MIL Simulink shell.
% Description:
%   Recreates the minimal Simulink top model used for Stage-1.5+ MIL
%   orchestration. The model contains only a command source, one MATLAB
%   System block, and logging sinks for the main outputs.
%
% Inputs:
%   none
%
% Outputs:
%   mil_top_build - assigned in base workspace with build metadata
%
% Units:
%   not applicable
%
% Assumptions:
%   Simulink is installed and the repository is available locally.

repo_root = fileparts(fileparts(mfilename('fullpath')));
models_dir = fullfile(repo_root, 'models');
model_name = 'mil_top';
model_file = fullfile(models_dir, [model_name '.slx']);

if ~exist(models_dir, 'dir')
    mkdir(models_dir);
end

params = local_get_base_or_default('mil_params', uav.sim.default_params_quad_x250());
dt_s = local_get_base_or_default('mil_dt_s', params.demo.dt_s);
t_final_s = local_get_base_or_default('mil_t_final_s', params.demo.case_hover_t_final_s);
state0 = local_get_base_or_default('mil_state0', ...
    uav.core.state_unpack(params.demo.initial_state_plant));
cmd_profile = local_get_base_or_default('mil_motor_cmd_profile', ...
    uav.sl.make_demo_command_profile('hover', params, dt_s, t_final_s));
motor_cmd_ts = local_get_base_or_default('mil_motor_cmd_ts', cmd_profile.timeseries);

assignin('base', 'mil_params', params);
assignin('base', 'mil_dt_s', dt_s);
assignin('base', 'mil_t_final_s', t_final_s);
assignin('base', 'mil_state0', state0);
assignin('base', 'mil_motor_cmd_profile', cmd_profile);
assignin('base', 'mil_motor_cmd_ts', motor_cmd_ts);
assignin('base', 'mil_model_name', model_name);
assignin('base', 'mil_model_file', model_file);

bus_defs = uav.sl.make_bus_defs();
assignin('base', 'mil_bus_defs', bus_defs);

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
    'StopTime', 'mil_t_final_s', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on');

source_block = add_block( ...
    'simulink/Sources/From Workspace', ...
    [model_name '/motor_cmd_profile'], ...
    'Position', [40 90 160 120], ...
    'VariableName', 'mil_motor_cmd_ts', ...
    'Interpolate', 'on');

shell_block = add_block( ...
    'simulink/User-Defined Functions/MATLAB System', ...
    [model_name '/thin_shell'], ...
    'Position', [220 55 420 155]);

set_param(shell_block, ...
    'System', 'uav.sl.Stage15MILSystem', ...
    'SimulateUsing', 'Interpreted execution', ...
    'dt_s', 'mil_dt_s', ...
    'state0', 'mil_state0', ...
    'params', 'mil_params');

state_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/state_out'], ...
    'Position', [520 30 550 50]);

sensors_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/sensors_out'], ...
    'Position', [520 75 550 95]);

estimator_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/estimator_out'], ...
    'Position', [520 120 550 140]);

diag_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/diag_out'], ...
    'Position', [520 165 550 185]);

add_line(model_name, 'motor_cmd_profile/1', 'thin_shell/1', 'autorouting', 'on');
add_line(model_name, 'thin_shell/1', 'state_out/1', 'autorouting', 'on');
add_line(model_name, 'thin_shell/2', 'sensors_out/1', 'autorouting', 'on');
add_line(model_name, 'thin_shell/3', 'estimator_out/1', 'autorouting', 'on');
add_line(model_name, 'thin_shell/4', 'diag_out/1', 'autorouting', 'on');

set_param(model_name, 'SimulationCommand', 'update');
save_system(model_name, model_file);
close_system(model_name, 0);

mil_top_build = struct();
mil_top_build.model_name = model_name;
mil_top_build.model_file = model_file;
mil_top_build.shell_block = [model_name '/thin_shell'];
mil_top_build.source_block = source_block;
mil_top_build.state_log_block = state_log_block;
mil_top_build.sensors_log_block = sensors_log_block;
mil_top_build.estimator_log_block = estimator_log_block;
mil_top_build.diag_log_block = diag_log_block;
mil_top_build.bus_defs = bus_defs;

assignin('base', 'mil_top_build', mil_top_build);

fprintf('Built thin MIL model: %s\n', model_file);

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
