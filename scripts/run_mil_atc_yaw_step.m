%% RUN_MIL_ATC_YAW_STEP Build and run the ATC MIL shell in yaw-step mode.
% Description:
%   Rebuilds the minimal ATC MIL shell, runs the external `atc_controller`
%   in direct MATLAB-call mode, and prints compact end-of-run diagnostics
%   for the yaw-step smoke scenario.
%
% Inputs:
%   none
%
% Outputs:
%   mil_atc_yaw_step_demo - assigned in base workspace with parsed results
%
% Units:
%   SI only, angles printed in radians
%
% Assumptions:
%   Sensor noise is disabled for a deterministic smoke scenario.

repo_root = fileparts(fileparts(mfilename('fullpath')));

params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
dt_s = params.demo.dt_s;
t_final_s = params.demo.case_yaw_step_t_final_s;
state0 = uav.core.state_unpack(params.demo.initial_state_plant);
bridge_cfg = uav.atc.default_atc_bridge_config([], params);
bridge_cfg.mode_name = "yaw_step";

assignin('base', 'mil_atc_params', params);
assignin('base', 'mil_atc_dt_s', dt_s);
assignin('base', 'mil_atc_t_final_s', t_final_s);
assignin('base', 'mil_atc_state0', state0);
assignin('base', 'mil_atc_bridge_cfg', bridge_cfg);

run(fullfile(repo_root, 'scripts', 'build_mil_top_atc.m'));
mil_top_atc_build = evalin('base', 'mil_top_atc_build');

model_file = char(mil_top_atc_build.model_file);
if exist(model_file, 'file') ~= 2
    error('uav:sl:run_mil_atc_yaw_step:MissingModelFile', ...
        'Expected built model file %s to exist.', model_file);
end

sim_out = sim(model_file, ...
    'StopTime', sprintf('%.16g', t_final_s), ...
    'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded(mil_top_atc_build.model_name)
    close_system(mil_top_atc_build.model_name, 0);
end

yout = sim_out.get('yout');
final_truth = local_extract_last_signal_sample(yout.get(1));
final_sensors = local_extract_last_signal_sample(yout.get(2));
final_estimator = local_extract_last_signal_sample(yout.get(3));
final_atc_cmd = local_extract_last_signal_sample(yout.get(4));
final_diag = local_extract_last_signal_sample(yout.get(5));

final_true_altitude_m = -final_truth.p_ned_m(3);
final_estimated_altitude_m = final_estimator.alt_m;
final_yaw_rad = final_estimator.euler_rpy_rad(3);
final_yaw_rate_radps = final_truth.w_b_radps(3);

fprintf('Thin ATC MIL yaw-step diagnostics:\n');
fprintf('  final estimated yaw [rad]        : %.6f\n', final_yaw_rad);
fprintf('  final true yaw rate [rad/s]      : %.6f\n', final_yaw_rate_radps);
fprintf('  final true altitude [m]          : %.6f\n', final_true_altitude_m);
fprintf('  final estimated altitude [m]     : %.6f\n', final_estimated_altitude_m);
fprintf('  final ATC motor norm [-]         : [%.6f %.6f %.6f %.6f]\n', ...
    final_atc_cmd.motor_norm_01);
fprintf('  final spool state [-]            : %u\n', final_diag.spool_state);
fprintf('  final true quat norm [-]         : %.12f\n', final_diag.quat_norm_true);
fprintf('  final estimated quat norm [-]    : %.12f\n', final_diag.quat_norm_est);

mil_atc_yaw_step_demo = struct();
mil_atc_yaw_step_demo.params = params;
mil_atc_yaw_step_demo.dt_s = dt_s;
mil_atc_yaw_step_demo.t_final_s = t_final_s;
mil_atc_yaw_step_demo.state0 = state0;
mil_atc_yaw_step_demo.bridge_cfg = bridge_cfg;
mil_atc_yaw_step_demo.final_truth = final_truth;
mil_atc_yaw_step_demo.final_sensors = final_sensors;
mil_atc_yaw_step_demo.final_estimator = final_estimator;
mil_atc_yaw_step_demo.final_atc_cmd = final_atc_cmd;
mil_atc_yaw_step_demo.final_diag = final_diag;
mil_atc_yaw_step_demo.final_true_altitude_m = final_true_altitude_m;
mil_atc_yaw_step_demo.final_estimated_altitude_m = final_estimated_altitude_m;
mil_atc_yaw_step_demo.final_yaw_rad = final_yaw_rad;
mil_atc_yaw_step_demo.final_yaw_rate_radps = final_yaw_rate_radps;

assignin('base', 'mil_atc_yaw_step_demo', mil_atc_yaw_step_demo);

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.imu.gyro_bias_b_radps = zeros(3, 1);
params.sensors.imu.gyro_noise_std_b_radps = zeros(3, 1);
params.sensors.baro.alt_bias_m = 0.0;
params.sensors.baro.alt_noise_std_m = 0.0;
params.sensors.gnss.pos_bias_ned_m = zeros(3, 1);
params.sensors.gnss.pos_noise_std_ned_m = zeros(3, 1);
params.sensors.gnss.vel_bias_ned_mps = zeros(3, 1);
params.sensors.gnss.vel_noise_std_ned_mps = zeros(3, 1);
params.sensors.mag.field_bias_b_uT = zeros(3, 1);
params.sensors.mag.field_noise_std_b_uT = zeros(3, 1);
end

function sample = local_extract_last_signal_sample(signal)
%LOCAL_EXTRACT_LAST_SIGNAL_SAMPLE Extract the final sample from one bus log.

if ~isa(signal, 'Simulink.SimulationData.Signal')
    error('uav:sl:run_mil_atc_yaw_step:UnexpectedDatasetElement', ...
        'Expected yout elements to be Simulink.SimulationData.Signal.');
end

sample = local_extract_last_struct_sample(signal.Values);
end

function sample = local_extract_last_struct_sample(value_struct)
%LOCAL_EXTRACT_LAST_STRUCT_SAMPLE Recursively extract one final bus sample.

sample = struct();
field_names = fieldnames(value_struct);

for k = 1:numel(field_names)
    field_name = field_names{k};
    field_value = value_struct.(field_name);

    if isstruct(field_value)
        sample.(field_name) = local_extract_last_struct_sample(field_value);
    elseif isa(field_value, 'timeseries')
        sample.(field_name) = local_extract_last_timeseries_sample(field_value);
    else
        error('uav:sl:run_mil_atc_yaw_step:UnexpectedLeafType', ...
            'Expected bus leaves to be stored as timeseries objects.');
    end
end
end

function sample = local_extract_last_timeseries_sample(ts)
%LOCAL_EXTRACT_LAST_TIMESERIES_SAMPLE Extract the final timeseries sample.

data = ts.Data;
n_samples = numel(ts.Time);

if isvector(data) && numel(data) == n_samples
    sample = data(end);
    return;
end

if ndims(data) >= 3 && size(data, ndims(data)) == n_samples
    indices = repmat({':'}, 1, ndims(data));
    indices{end} = n_samples;
    sample = squeeze(data(indices{:}));
elseif size(data, 1) == n_samples
    sample = data(end, :).';
elseif size(data, 2) == n_samples
    sample = data(:, end);
else
    error('uav:sl:run_mil_atc_yaw_step:UnexpectedTimeseriesShape', ...
        'Unable to extract the final sample from the logged timeseries.');
end

if isvector(sample) && ~isscalar(sample)
    sample = sample(:);
end
end
