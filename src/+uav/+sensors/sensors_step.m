function sens = sensors_step(state, diag, params)
%SENSORS_STEP Aggregate the stateless Stage-1.5 sensor layer outputs.
% Description:
%   Calls the individual sensor functions and returns one composite sensor
%   snapshot without introducing any hidden state.
%
% Inputs:
%   state  - canonical plant state struct
%   diag   - diagnostic struct for the current plant sample
%   params - parameter struct with sensor settings
%
% Outputs:
%   sens - struct with imu, baro, mag, and gnss substructs
%
% Units:
%   SI only, magnetic field in uT
%
% Assumptions:
%   The sensor layer is a thin measurement wrapper over the plant kernel.

sens = struct();
sens.imu = uav.sensors.imu_measure(state, diag, params);
sens.baro = uav.sensors.baro_measure(state, params);
sens.mag = uav.sensors.mag_measure(state, params);
sens.gnss = uav.sensors.gnss_measure(state, params);
end
