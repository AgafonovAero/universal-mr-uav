%% BUILD_MIL_TOP_ATC Programmatically build the thin ATC MIL Simulink shell.
% Description:
%   Recreates the minimal Simulink top model used for controller-in-the-
%   loop integration with the external `atc_controller` repository. The
%   model contains only one MATLAB System block and logging sinks for the
%   main bridge outputs.
%
% Inputs:
%   none
%
% Outputs:
%   mil_top_atc_build - assigned in base workspace with build metadata
%
% Units:
%   not applicable
%
% Assumptions:
%   Simulink is installed and the repository is available locally.

repo_root = fileparts(fileparts(mfilename('fullpath')));
models_dir = fullfile(repo_root, 'models');
model_name = 'mil_top_atc';
model_file = fullfile(models_dir, [model_name '.slx']);

if ~exist(models_dir, 'dir')
    mkdir(models_dir);
end

params = local_get_base_or_default('mil_atc_params', ...
    uav.sim.default_params_quad_x250());
dt_s = local_get_base_or_default('mil_atc_dt_s', params.demo.dt_s);
t_final_s = local_get_base_or_default('mil_atc_t_final_s', ...
    params.demo.case_hover_t_final_s);
state0 = local_get_base_or_default('mil_atc_state0', ...
    uav.core.state_unpack(params.demo.initial_state_plant));
bridge_cfg = local_get_base_or_default('mil_atc_bridge_cfg', ...
    uav.atc.default_atc_bridge_config([], params));

assignin('base', 'mil_atc_params', params);
assignin('base', 'mil_atc_dt_s', dt_s);
assignin('base', 'mil_atc_t_final_s', t_final_s);
assignin('base', 'mil_atc_state0', state0);
assignin('base', 'mil_atc_bridge_cfg', bridge_cfg);
assignin('base', 'mil_atc_model_name', model_name);
assignin('base', 'mil_atc_model_file', model_file);

bus_defs = uav.sl.make_atc_bus_defs();
assignin('base', 'mil_atc_bus_defs', bus_defs);

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
    'StopTime', 'mil_atc_t_final_s', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on');

shell_block = add_block( ...
    'simulink/User-Defined Functions/MATLAB System', ...
    [model_name '/atc_mil_shell'], ...
    'Position', [130 70 360 210]);

set_param(shell_block, ...
    'System', 'uav.sl.Stage15ATCMILSystem', ...
    'SimulateUsing', 'Interpreted execution', ...
    'dt_s', 'mil_atc_dt_s', ...
    'state0', 'mil_atc_state0', ...
    'params', 'mil_atc_params', ...
    'bridge_cfg', 'mil_atc_bridge_cfg');

truth_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/truth_out'], ...
    'Position', [470 40 500 60]);

sensors_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/sensors_out'], ...
    'Position', [470 85 500 105]);

estimator_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/estimator_out'], ...
    'Position', [470 130 500 150]);

atc_cmd_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/atc_cmd_out'], ...
    'Position', [470 175 500 195]);

diag_log_block = add_block( ...
    'simulink/Sinks/Out1', ...
    [model_name '/diag_out'], ...
    'Position', [470 220 500 240]);

add_line(model_name, 'atc_mil_shell/1', 'truth_out/1', 'autorouting', 'on');
add_line(model_name, 'atc_mil_shell/2', 'sensors_out/1', 'autorouting', 'on');
add_line(model_name, 'atc_mil_shell/3', 'estimator_out/1', 'autorouting', 'on');
add_line(model_name, 'atc_mil_shell/4', 'atc_cmd_out/1', 'autorouting', 'on');
add_line(model_name, 'atc_mil_shell/5', 'diag_out/1', 'autorouting', 'on');

set_param(model_name, 'SimulationCommand', 'update');
save_system(model_name, model_file);
close_system(model_name, 0);

mil_top_atc_build = struct();
mil_top_atc_build.model_name = model_name;
mil_top_atc_build.model_file = model_file;
mil_top_atc_build.shell_block = [model_name '/atc_mil_shell'];
mil_top_atc_build.truth_log_block = truth_log_block;
mil_top_atc_build.sensors_log_block = sensors_log_block;
mil_top_atc_build.estimator_log_block = estimator_log_block;
mil_top_atc_build.atc_cmd_log_block = atc_cmd_log_block;
mil_top_atc_build.diag_log_block = diag_log_block;
mil_top_atc_build.bus_defs = bus_defs;

assignin('base', 'mil_top_atc_build', mil_top_atc_build);

fprintf('Built thin ATC MIL model: %s\n', model_file);

function value = local_get_base_or_default(var_name, default_value)
%LOCAL_GET_BASE_OR_DEFAULT Read a base-workspace variable when available.

if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    value = evalin('base', var_name);
else
    value = default_value;
end
end

function text = local_scalar_string(value)
%LOCAL_SCALAR_STRING Convert one scalar to a stable text representation.

text = sprintf('%.16g', value);
end
