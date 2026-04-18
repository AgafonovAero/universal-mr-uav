function dx = eom6dof_quat(~, x, params, inputs)
%EOM6DOF_QUAT Minimal rigid-body 6DOF equations of motion with quaternions.
% Description:
%   Implements a minimal rigid-body model with translational and rotational
%   dynamics. The packed state is
%   [p_ned_m; v_b_mps; q_nb; w_b_rps], where q_nb rotates body-frame
%   vectors into the NED frame. Inputs are applied body forces and moments.
%
% Inputs:
%   x      - 13x1 state vector [m; m/s; quaternion; rad/s]
%   params - struct with mass_kg, inertia_kgm2, gravity_mps2
%   inputs - struct with forces_b_N and moments_b_Nm
%
% Outputs:
%   dx - 13x1 state derivative in packed form
%
% Units:
%   Position [m], velocity [m/s], quaternion [-], angular rate [rad/s],
%   force [N], moment [N*m]
%
% Assumptions:
%   Rigid body, constant mass and inertia, gravity-only environment, and
%   body-frame velocity coordinates.

mass_kg = params.mass_kg;
inertia_kgm2 = params.inertia_kgm2;

v_b_mps = x(4:6);
q_nb = uav.core.quat_normalize(x(7:10));
w_b_rps = x(11:13);

c_nb = uav.core.quat_to_dcm(q_nb);
g_n_mps2 = uav.env.gravity_ned(params.gravity_mps2);
g_b_mps2 = c_nb.' * g_n_mps2;

p_dot_ned_mps = c_nb * v_b_mps;
v_dot_b_mps2 = (inputs.forces_b_N(:) ./ mass_kg) + g_b_mps2 - cross(w_b_rps, v_b_mps);
q_dot = 0.5 * local_quat_multiply(q_nb, [0.0; w_b_rps]);
w_dot_b_rps2 = inertia_kgm2 \ (inputs.moments_b_Nm(:) - cross(w_b_rps, inertia_kgm2 * w_b_rps));

dx = [p_dot_ned_mps; v_dot_b_mps2; q_dot; w_dot_b_rps2];
end

function q_out = local_quat_multiply(q_left, q_right)
%LOCAL_QUAT_MULTIPLY Multiply two scalar-first Hamilton quaternions.

s_left = q_left(1);
v_left = q_left(2:4);
s_right = q_right(1);
v_right = q_right(2:4);

q_out = [ ...
    s_left * s_right - dot(v_left, v_right); ...
    s_left * v_right + s_right * v_left + cross(v_left, v_right)];
end
