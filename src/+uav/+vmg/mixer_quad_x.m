function [omega_cmd_radps, thrust_cmd_N] = mixer_quad_x(totalThrust_N, bodyMoments_Nm, params)
%MIXER_QUAD_X Mix total thrust and body moments for a quad-X geometry.
% Description:
%   Converts a desired total thrust magnitude and body moment vector into
%   four rotor speed commands for a quad-X layout using explicit rotor
%   coordinates in the body frame.
%
% Inputs:
%   totalThrust_N   - desired total upward thrust magnitude [N]
%   bodyMoments_Nm  - desired body moments [Mx; My; Mz] [N*m]
%   params          - struct with motor_xy_m, kT_N_per_radps2,
%                     kQ_Nm_per_radps2, and optionally spin_dir
%
% Outputs:
%   omega_cmd_radps - 4x1 rotor speed commands [rad/s]
%   thrust_cmd_N    - 4x1 rotor thrust commands [N]
%
% Units:
%   SI only
%
% Assumptions:
%   Rotor numbering is 1 front-left, 2 front-right, 3 rear-right,
%   4 rear-left. Positive thrust acts along -Z_b. Rotor coordinates are
%   given as params.motor_xy_m(i, :) = [x_i, y_i] in the body frame.
%   Yaw moment sign is set by params.spin_dir, with the default
%   [1; -1; 1; -1].

if isfield(params, 'spin_dir') && ~isempty(params.spin_dir)
    spin_dir = params.spin_dir(:);
else
    spin_dir = [1.0; -1.0; 1.0; -1.0];
end

motor_xy_m = params.motor_xy_m;
if ~isequal(size(motor_xy_m), [4, 2])
    error('uav:vmg:mixer_quad_x:InvalidMotorGeometry', ...
        'Expected params.motor_xy_m to have size 4x2.');
end

rotor = uav.vmg.rotor_coeffs(params);
x_i_m = motor_xy_m(:, 1);
y_i_m = motor_xy_m(:, 2);
yaw_gain_m = rotor.kQ_Nm_per_radps2 ./ rotor.kT_N_per_radps2;

allocation = [ ...
    1.0,        1.0,        1.0,        1.0; ...
    -y_i_m(1), -y_i_m(2), -y_i_m(3), -y_i_m(4); ...
     x_i_m(1),  x_i_m(2),  x_i_m(3),  x_i_m(4); ...
    yaw_gain_m * spin_dir(1), yaw_gain_m * spin_dir(2), ...
    yaw_gain_m * spin_dir(3), yaw_gain_m * spin_dir(4)];

thrust_cmd_N = allocation \ [totalThrust_N; bodyMoments_Nm(:)];
thrust_cmd_N = max(thrust_cmd_N, 0.0);
omega_cmd_radps = sqrt(thrust_cmd_N ./ rotor.kT_N_per_radps2);
end
