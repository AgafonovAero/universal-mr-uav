function [x_next, diag] = plant_step(x_prev, motor_cmd_radps, dt_s, params)
%PLANT_STEP Execute one discrete simulation step for the plant kernel.
% Description:
%   Advances the 17-state plant model
%   [p_ned; v_b; q_nb; w_b; omega_m] by one time step using a first-order
%   motor model, summed rotor forces and moments, and the existing 6DOF
%   rigid-body equations.
%
% Inputs:
%   x_prev          - 17x1 plant state [m; m/s; quaternion; rad/s; rad/s]
%   motor_cmd_radps - 4x1 motor speed commands [rad/s]
%   dt_s            - time step [s]
%   params          - baseline plant and propulsion parameter struct
%
% Outputs:
%   x_next - 17x1 updated plant state
%   diag   - struct with propulsion and rigid-body diagnostics
%
% Units:
%   SI only
%
% Assumptions:
%   Explicit Euler integration is sufficient for the minimal Stage-1.5
%   kernel and motor commands are omega references.

x_prev = x_prev(:);
motor_cmd_radps = motor_cmd_radps(:);

if numel(x_prev) ~= 17
    error('uav:sim:plant_step:StateSizeMismatch', ...
        'Expected x_prev to have 17 elements.');
end

if numel(motor_cmd_radps) ~= 4
    error('uav:sim:plant_step:CommandSizeMismatch', ...
        'Expected motor_cmd_radps to have 4 elements.');
end

x_rigid_prev = x_prev(1:13);
omega_prev_radps = x_prev(14:17);

[omega_next_radps, motor_diag] = uav.vmg.motor_esc_step( ...
    omega_prev_radps, motor_cmd_radps, dt_s, params.motor);
fm = uav.core.forces_moments_sum(omega_next_radps, params);

inputs = struct();
inputs.forces_b_N = fm.forces_b_N;
inputs.moments_b_Nm = fm.moments_b_Nm;

dx_rigid = uav.core.eom6dof_quat(0.0, x_rigid_prev, params, inputs);
x_rigid_next = x_rigid_prev + dt_s .* dx_rigid;
x_rigid_next(7:10) = uav.core.quat_normalize(x_rigid_next(7:10));

x_next = [x_rigid_next; omega_next_radps];

diag = struct();
diag.motor = motor_diag;
diag.fm = fm;
diag.omega_m_radps = omega_next_radps;
diag.forces_b_N = fm.forces_b_N;
diag.moments_b_Nm = fm.moments_b_Nm;
diag.quat_norm = norm(x_rigid_next(7:10));
diag.dx_rigid = dx_rigid;
end
