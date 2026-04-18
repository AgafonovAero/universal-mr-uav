function fm = forces_moments_sum(omega_radps, params)
%FORCES_MOMENTS_SUM Sum quadrotor thrust forces and moments in body axes.
% Description:
%   Computes per-rotor thrust and drag torque, then sums total body force
%   and moment contributions using the quad-X rotor geometry.
%
% Inputs:
%   omega_radps - 4x1 rotor angular speeds [rad/s]
%   params      - struct with motor_xy_m, spin_dir, and rotor coefficients
%
% Outputs:
%   fm - struct with rotor_thrust_N, rotor_torque_Nm, forces_b_N,
%        and moments_b_Nm
%
% Units:
%   SI only
%
% Assumptions:
%   Positive thrust acts along -Z_b and yaw moment is determined by
%   params.spin_dir.

omega_radps = omega_radps(:);
if numel(omega_radps) ~= 4
    error('uav:core:forces_moments_sum:SizeMismatch', ...
        'Expected omega_radps to have 4 elements.');
end

[kT_N_per_radps2, kQ_Nm_per_radps2] = local_get_rotor_coeffs(params);
[rotor_thrust_N, rotor_torque_Nm] = uav.vmg.rotor_simple( ...
    omega_radps, kT_N_per_radps2, kQ_Nm_per_radps2);

x_i_m = params.motor_xy_m(:, 1);
y_i_m = params.motor_xy_m(:, 2);
spin_dir = params.spin_dir(:);

forces_b_N = [0.0; 0.0; -sum(rotor_thrust_N)];
moments_b_Nm = [ ...
    sum(-y_i_m .* rotor_thrust_N); ...
    sum( x_i_m .* rotor_thrust_N); ...
    sum(spin_dir .* rotor_torque_Nm)];

fm = struct();
fm.rotor_thrust_N = rotor_thrust_N(:);
fm.rotor_torque_Nm = rotor_torque_Nm(:);
fm.forces_b_N = forces_b_N;
fm.moments_b_Nm = moments_b_Nm;
end

function [kT_N_per_radps2, kQ_Nm_per_radps2] = local_get_rotor_coeffs(params)
%LOCAL_GET_ROTOR_COEFFS Read rotor coefficients with Stage-1 compatibility.

if isfield(params, 'rotor') && isfield(params.rotor, 'kT_N_per_radps2')
    kT_N_per_radps2 = params.rotor.kT_N_per_radps2;
else
    kT_N_per_radps2 = params.kT_N_per_radps2;
end

if isfield(params, 'rotor') && isfield(params.rotor, 'kQ_Nm_per_radps2')
    kQ_Nm_per_radps2 = params.rotor.kQ_Nm_per_radps2;
else
    kQ_Nm_per_radps2 = params.kQ_Nm_per_radps2;
end
end
