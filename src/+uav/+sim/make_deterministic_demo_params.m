function params = make_deterministic_demo_params()
%MAKE_DETERMINISTIC_DEMO_PARAMS Return baseline parameters with zero sensor noise.
% Description:
%   Builds the default quad-X demo parameter set and disables all sensor
%   bias and white-noise terms so scripted demos remain deterministic and
%   reproducible across local reruns.
%
% Inputs:
%   none
%
% Outputs:
%   params - deterministic parameter struct for demo scenarios
%
% Units:
%   SI only, angles in radians
%
% Assumptions:
%   The default parameter preset remains the source of truth for the
%   baseline airframe values.

params = uav.sim.default_params_quad_x250();

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
