%% RUN_SIL_STUB_HOVER Build and run the thin SIL-prep shell in hover mode.
% Description:
%   Rebuilds the minimal SIL-prep Simulink shell, runs the temporary
%   external FCS stub in hover mode, and prints compact end-of-run
%   diagnostics.
%
% Inputs:
%   none
%
% Outputs:
%   sil_stub_hover_demo - assigned in base workspace with parsed results
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
rates_cfg = params.sil.rates;

assignin('base', 'sil_params', params);
assignin('base', 'sil_dt_s', dt_s);
assignin('base', 'sil_t_final_s', t_final_s);
assignin('base', 'sil_state0', state0);
assignin('base', 'sil_rates_cfg', rates_cfg);
assignin('base', 'sil_stub_mode', "hover");
assignin('base', 'sil_actuation_mode', "norm01");
assignin('base', 'sil_time_s', 0.0);

run(fullfile(repo_root, 'scripts', 'build_sil_top.m'));
sil_top_build = evalin('base', 'sil_top_build');

model_file = char(sil_top_build.model_file);
if exist(model_file, 'file') ~= 2
    error('uav:sl:run_sil_stub_hover:MissingModelFile', ...
        'Expected built model file %s to exist.', model_file);
end

sim_out = sim(model_file, ...
    'StopTime', sprintf('%.16g', t_final_s), ...
    'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded(sil_top_build.model_name)
    close_system(sil_top_build.model_name, 0);
end

yout = sim_out.get('yout');
sensor_packet_log = local_extract_last_signal_sample(yout.get(1));
truth_log = local_extract_last_signal_sample(yout.get(2));
diag_log = local_extract_last_signal_sample(yout.get(3));

stub = uav.sl.StubExternalFCSSystem('mode_name', "hover", 'params', params);
final_actuator_cmd = step(stub, sensor_packet_log);
release(stub);

final_true_position_ned_m = truth_log.state.p_ned_m;
final_true_altitude_m = -truth_log.state.p_ned_m(3);
final_estimated_altitude_m = truth_log.estimator.alt_m;
final_true_quat_norm = diag_log.quat_norm_true;
final_estimated_quat_norm = diag_log.quat_norm_est;

fprintf('Thin SIL-prep hover diagnostics:\n');
fprintf('  final true position NED [m]      : [%.6f %.6f %.6f]\n', ...
    final_true_position_ned_m);
fprintf('  final true altitude [m]          : %.6f\n', final_true_altitude_m);
fprintf('  final estimated altitude [m]     : %.6f\n', final_estimated_altitude_m);
fprintf('  final true quat norm [-]         : %.12f\n', final_true_quat_norm);
fprintf('  final estimated quat norm [-]    : %.12f\n', final_estimated_quat_norm);
fprintf('  final hover actuator cmd [-]     : [%.6f %.6f %.6f %.6f]\n', ...
    final_actuator_cmd.motor_norm_01);

sil_stub_hover_demo = struct();
sil_stub_hover_demo.params = params;
sil_stub_hover_demo.dt_s = dt_s;
sil_stub_hover_demo.t_final_s = t_final_s;
sil_stub_hover_demo.state0 = state0;
sil_stub_hover_demo.rates_cfg = rates_cfg;
sil_stub_hover_demo.final_sensor_packet = sensor_packet_log;
sil_stub_hover_demo.final_truth = truth_log;
sil_stub_hover_demo.final_diag = diag_log;
sil_stub_hover_demo.final_true_position_ned_m = final_true_position_ned_m;
sil_stub_hover_demo.final_true_altitude_m = final_true_altitude_m;
sil_stub_hover_demo.final_estimated_altitude_m = final_estimated_altitude_m;
sil_stub_hover_demo.final_true_quat_norm = final_true_quat_norm;
sil_stub_hover_demo.final_estimated_quat_norm = final_estimated_quat_norm;
sil_stub_hover_demo.final_hover_actuator_cmd = final_actuator_cmd.motor_norm_01;

assignin('base', 'sil_stub_hover_demo', sil_stub_hover_demo);

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.
% Description:
%   Disables all configured sensor biases and white noise for repeatable
%   SIL smoke runs.
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

function sample = local_extract_last_signal_sample(signal)
%LOCAL_EXTRACT_LAST_SIGNAL_SAMPLE Extract the final sample from one bus log.
% Description:
%   Recursively reads the last sample of a logged Simulink bus signal and
%   rebuilds it as a scalar MATLAB struct.
%
% Inputs:
%   signal - one Simulink.SimulationData.Signal from yout
%
% Outputs:
%   sample - scalar struct with the final logged sample
%
% Units:
%   inherited from the logged leaves
%
% Assumptions:
%   Bus leaves are stored as timeseries objects.

if ~isa(signal, 'Simulink.SimulationData.Signal')
    error('uav:sl:run_sil_stub_hover:UnexpectedDatasetElement', ...
        'Expected yout elements to be Simulink.SimulationData.Signal.');
end

sample = local_extract_last_struct_sample(signal.Values);
end

function sample = local_extract_last_struct_sample(value_struct)
%LOCAL_EXTRACT_LAST_STRUCT_SAMPLE Recursively extract one final bus sample.
% Description:
%   Walks through nested structs of timeseries leaves and reconstructs the
%   final scalar struct sample.
%
% Inputs:
%   value_struct - nested struct with timeseries leaves
%
% Outputs:
%   sample - scalar struct sample
%
% Units:
%   inherited from the leaves
%
% Assumptions:
%   All bus leaves are timeseries objects.

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
        error('uav:sl:run_sil_stub_hover:UnexpectedLeafType', ...
            'Expected bus leaves to be stored as timeseries objects.');
    end
end
end

function sample = local_extract_last_timeseries_sample(ts)
%LOCAL_EXTRACT_LAST_TIMESERIES_SAMPLE Extract the final timeseries sample.
% Description:
%   Reads the final sample from a scalar, vector, or matrix timeseries and
%   reshapes vectors back into column form to match the code-centric API.
%
% Inputs:
%   ts - timeseries object
%
% Outputs:
%   sample - numeric scalar or column vector sample
%
% Units:
%   inherited from the leaf signal
%
% Assumptions:
%   One timeseries dimension corresponds to time.

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
    error('uav:sl:run_sil_stub_hover:UnexpectedTimeseriesShape', ...
        'Unable to extract the final sample from the logged timeseries.');
end

if isvector(sample) && ~isscalar(sample)
    sample = sample(:);
end
end
