function x_packed = state_pack(state)
%STATE_PACK Pack the canonical plant state struct into a 17x1 vector.
% Description:
%   Converts the canonical Stage-1.5 state struct into the packed plant
%   state vector ordered as [p_ned; v_b; q_nb; w_b; omega_m].
%
% Inputs:
%   state - canonical state struct
%
% Outputs:
%   x_packed - 17x1 plant state vector
%
% Units:
%   [m; m/s; quaternion; rad/s; rad/s]
%
% Assumptions:
%   The input follows the canonical field naming from uav.core.state_validate.

state = uav.core.state_validate(state);

x_packed = [ ...
    state.p_ned_m; ...
    state.v_b_mps; ...
    state.q_nb; ...
    state.w_b_radps; ...
    state.omega_m_radps];
end
