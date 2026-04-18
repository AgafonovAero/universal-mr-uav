%% RUN_MIL_TOP_HOVER Build and run the thin MIL shell on a hover profile.
% Description:
%   Rebuilds the minimal Simulink shell, generates a deterministic hover
%   motor-command profile, runs the model, and prints compact end-of-run
%   diagnostics.
%
% Inputs:
%   none
%
% Outputs:
%   mil_top_hover_demo - assigned in base workspace with parsed results
%
% Units:
%   SI only, angles printed in radians
%
% Assumptions:
%   Sensor noise is disabled for a deterministic smoke scenario.

repo_root = fileparts(fileparts(mfilename('fullpath')));

params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
dt_s = params.demo.dt_s;
t_final_s = params.demo.case_hover_t_final_s;
state0 = uav.core.state_unpack(params.demo.initial_state_plant);
cmd_profile = uav.sl.make_demo_command_profile('hover', params, dt_s, t_final_s);

assignin('base', 'mil_params', params);
assignin('base', 'mil_dt_s', dt_s);
assignin('base', 'mil_t_final_s', t_final_s);
assignin('base', 'mil_state0', state0);
assignin('base', 'mil_motor_cmd_profile', cmd_profile);
assignin('base', 'mil_motor_cmd_ts', cmd_profile.timeseries);

run(fullfile(repo_root, 'scripts', 'build_mil_top.m'));
mil_top_build = evalin('base', 'mil_top_build');

load_system(mil_top_build.model_file);
sim_out = sim(mil_top_build.model_name, ...
    'StopTime', sprintf('%.16g', t_final_s), ...
    'ReturnWorkspaceOutputs', 'on');
close_system(mil_top_build.model_name, 0);

yout = sim_out.get('yout');
state_log = local_unpack_dataset_signal(yout.get(1));
sensors_log = local_unpack_dataset_signal(yout.get(2));
estimator_log = local_unpack_dataset_signal(yout.get(3));
diag_log = local_unpack_dataset_signal(yout.get(4));

final_state = state_log(end);
final_estimator = estimator_log(end);
final_diag = diag_log(end);
final_position_ned_m = final_state.p_ned_m;
final_altitude_m = -final_state.p_ned_m(3);
final_estimated_altitude_m = final_estimator.alt_m;
final_estimated_euler_rpy_rad = final_estimator.euler_rpy_rad;
final_true_quat_norm = final_diag.plant.quat_norm;
final_estimated_quat_norm = final_diag.estimator.attitude.quat_norm;

fprintf('Thin MIL hover diagnostics:\n');
fprintf('  final position NED [m]      : [%.6f %.6f %.6f]\n', ...
    final_position_ned_m);
fprintf('  final altitude [m]          : %.6f\n', final_altitude_m);
fprintf('  final estimated altitude [m]: %.6f\n', final_estimated_altitude_m);
fprintf('  final estimated Euler [rad] : [%.6f %.6f %.6f]\n', ...
    final_estimated_euler_rpy_rad);
fprintf('  final true quat norm [-]    : %.12f\n', final_true_quat_norm);
fprintf('  final estimated quat norm[-]: %.12f\n', final_estimated_quat_norm);

mil_top_hover_demo = struct();
mil_top_hover_demo.time_s = cmd_profile.time_s;
mil_top_hover_demo.params = params;
mil_top_hover_demo.dt_s = dt_s;
mil_top_hover_demo.t_final_s = t_final_s;
mil_top_hover_demo.state0 = state0;
mil_top_hover_demo.cmd_profile = cmd_profile;
mil_top_hover_demo.state_log = state_log;
mil_top_hover_demo.sensors_log = sensors_log;
mil_top_hover_demo.estimator_log = estimator_log;
mil_top_hover_demo.diag_log = diag_log;
mil_top_hover_demo.final_position_ned_m = final_position_ned_m;
mil_top_hover_demo.final_altitude_m = final_altitude_m;
mil_top_hover_demo.final_estimated_altitude_m = final_estimated_altitude_m;
mil_top_hover_demo.final_estimated_euler_rpy_rad = final_estimated_euler_rpy_rad;
mil_top_hover_demo.final_true_quat_norm = final_true_quat_norm;
mil_top_hover_demo.final_estimated_quat_norm = final_estimated_quat_norm;

assignin('base', 'mil_top_hover_demo', mil_top_hover_demo);

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.
% Description:
%   Disables all configured sensor biases and white noise for repeatable
%   hover smoke runs.
%
% Inputs:
%   params - parameter struct
%
% Outputs:
%   params - parameter struct with deterministic sensor settings
%
% Units:
%   SI only
%
% Assumptions:
%   Only demo settings are modified.

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

function samples = local_unpack_dataset_signal(signal)
%LOCAL_UNPACK_DATASET_SIGNAL Convert one logged bus signal into samples.
% Description:
%   Expands the hierarchical timeseries structure stored in one dataset
%   element of `yout` into a struct array with one sample per time step.
%
% Inputs:
%   signal - one Simulink.SimulationData.Signal from yout
%
% Outputs:
%   samples - struct array with one sample per time step
%
% Units:
%   not applicable
%
% Assumptions:
%   The bus leaves are stored as timeseries objects.

if ~isa(signal, 'Simulink.SimulationData.Signal')
    error('uav:sl:run_mil_top_hover:UnexpectedDatasetElement', ...
        'Expected yout elements to be Simulink.SimulationData.Signal.');
end

values = signal.Values;
if ~isstruct(values)
    error('uav:sl:run_mil_top_hover:UnexpectedSignalValues', ...
        'Expected signal.Values to be a struct of timeseries leaves.');
end

time_s = local_extract_time(values);
n_samples = numel(time_s);
samples = repmat(struct(), n_samples, 1);
field_names = fieldnames(values);

for k = 1:numel(field_names)
    field_name = field_names{k};
    field_value = values.(field_name);

    if isstruct(field_value)
        expanded = local_expand_struct_timeseries(field_value, n_samples);
        for idx = 1:n_samples
            samples(idx).(field_name) = expanded(idx);
        end
        continue;
    end

    if ~isa(field_value, 'timeseries')
        error('uav:sl:run_mil_top_hover:UnexpectedLeafType', ...
            'Expected bus leaves to be stored as timeseries.');
    end

    for idx = 1:n_samples
        samples(idx).(field_name) = local_extract_timeseries_sample( ...
            field_value, idx, n_samples);
    end
end
end

function samples = local_expand_struct_timeseries(value_struct, n_samples)
%LOCAL_EXPAND_STRUCT_TIMESERIES Expand one nested struct of timeseries.
% Description:
%   Recursively expands nested bus substructures into one struct sample per
%   time step.
%
% Inputs:
%   value_struct - nested struct with timeseries leaves
%   n_samples    - number of time steps
%
% Outputs:
%   samples - struct array with one sample per time step
%
% Units:
%   not applicable
%
% Assumptions:
%   Every leaf is a timeseries object.

samples = repmat(struct(), n_samples, 1);
field_names = fieldnames(value_struct);

for k = 1:numel(field_names)
    field_name = field_names{k};
    field_value = value_struct.(field_name);

    if isstruct(field_value)
        expanded = local_expand_struct_timeseries(field_value, n_samples);
        for idx = 1:n_samples
            samples(idx).(field_name) = expanded(idx);
        end
        continue;
    end

    for idx = 1:n_samples
        samples(idx).(field_name) = local_extract_timeseries_sample( ...
            field_value, idx, n_samples);
    end
end
end

function time_s = local_extract_time(value_struct)
%LOCAL_EXTRACT_TIME Return the common time vector from a bus-value struct.
% Description:
%   Recursively searches the first timeseries leaf and returns its time
%   vector.
%
% Inputs:
%   value_struct - nested struct with timeseries leaves
%
% Outputs:
%   time_s - common time vector [s]
%
% Units:
%   seconds
%
% Assumptions:
%   All leaf timeseries use the same simulation time base.

time_s = [];
field_names = fieldnames(value_struct);

for k = 1:numel(field_names)
    field_value = value_struct.(field_names{k});

    if isstruct(field_value)
        time_s = local_extract_time(field_value);
    elseif isa(field_value, 'timeseries')
        time_s = field_value.Time(:);
    end

    if ~isempty(time_s)
        return;
    end
end

error('uav:sl:run_mil_top_hover:MissingTimeVector', ...
    'Unable to locate a timeseries leaf inside signal.Values.');
end

function sample = local_extract_timeseries_sample(ts, sample_idx, n_samples)
%LOCAL_EXTRACT_TIMESERIES_SAMPLE Extract one sample from a timeseries leaf.
% Description:
%   Reads one time sample from a scalar, vector, or matrix timeseries and
%   reshapes vectors back into column form to match the code-centric API.
%
% Inputs:
%   ts         - timeseries object
%   sample_idx - sample index
%   n_samples  - number of time samples in the run
%
% Outputs:
%   sample - numeric scalar or column vector sample
%
% Units:
%   inherited from the leaf signal
%
% Assumptions:
%   The final dimension, or one matrix dimension, corresponds to time.

data = ts.Data;

if isvector(data) && numel(data) == n_samples
    sample = data(sample_idx);
    return;
end

if ndims(data) >= 3 && size(data, ndims(data)) == n_samples
    indices = repmat({':'}, 1, ndims(data));
    indices{end} = sample_idx;
    sample = data(indices{:});
    sample = squeeze(sample);
elseif size(data, 1) == n_samples
    sample = data(sample_idx, :).';
elseif size(data, 2) == n_samples
    sample = data(:, sample_idx);
else
    error('uav:sl:run_mil_top_hover:UnexpectedTimeseriesShape', ...
        'Unable to map timeseries data to sample index %d.', sample_idx);
end

if isvector(sample) && ~isscalar(sample)
    sample = sample(:);
end
end
