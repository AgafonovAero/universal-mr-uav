%% RUN_SENSOR_SANITY_DEMO Print a deterministic Stage-1.5 sensor snapshot.
% Description:
%   Builds a level-hover state with zero sensor noise and prints one
%   transparent sensor-layer sample for engineering sanity checks.
%
% Inputs:
%   none
%
% Outputs:
%   demo - returned in base workspace when assigned by the caller
%
% Units:
%   SI only, magnetic field in uT
%
% Assumptions:
%   The scenario uses identity attitude and hover rotor speed.

params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);
state.p_ned_m = [12.0; -3.0; -25.0];
state.v_b_mps = [3.0; -1.0; 0.5];
state.omega_m_radps = repmat(params.hover_omega_radps, 4, 1);

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);
diag = struct();
diag.forces_b_N = fm.forces_b_N;
diag.moments_b_Nm = fm.moments_b_Nm;

sens = uav.sensors.sensors_step(state, diag, params);

fprintf('Sensor sanity demo:\n');
fprintf('  IMU accel [m/s^2]      : [%.6f %.6f %.6f]\n', sens.imu.accel_b_mps2);
fprintf('  IMU gyro [rad/s]       : [%.6f %.6f %.6f]\n', sens.imu.gyro_b_radps);
fprintf('  Baro altitude [m]      : %.6f\n', sens.baro.alt_m);
fprintf('  Baro pressure [Pa]     : %.6f\n', sens.baro.pressure_pa);
fprintf('  Mag body field [uT]    : [%.6f %.6f %.6f]\n', sens.mag.field_b_uT);
fprintf('  GNSS position NED [m]  : [%.6f %.6f %.6f]\n', sens.gnss.pos_ned_m);
fprintf('  GNSS velocity NED [m/s]: [%.6f %.6f %.6f]\n', sens.gnss.vel_ned_mps);

demo = struct();
demo.state = state;
demo.diag = diag;
demo.sensors = sens;

assignin('base', 'sensor_sanity_demo', demo);

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
