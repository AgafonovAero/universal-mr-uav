function state = state_unpack(x_packed)
%STATE_UNPACK Unpack a packed plant state vector into the canonical struct.
% Description:
%   Converts the Stage-1.5 packed plant vector ordered as
%   [p_ned; v_b; q_nb; w_b; omega_m] into the canonical state struct.
%
% Inputs:
%   x_packed - 17-element plant state vector
%
% Outputs:
%   state - canonical state struct
%
% Units:
%   [m; m/s; quaternion; rad/s; rad/s]
%
% Assumptions:
%   The input vector uses the canonical Stage-1.5 state order.

if ~isnumeric(x_packed) || ~isreal(x_packed)
    error('uav:core:state_unpack:StateType', ...
        'Expected x_packed to be a real numeric vector.');
end

x_packed = x_packed(:);
if numel(x_packed) ~= 17
    error('uav:core:state_unpack:StateSize', ...
        'Expected x_packed to have 17 elements.');
end

if any(~isfinite(x_packed))
    error('uav:core:state_unpack:StateFinite', ...
        'Expected x_packed to contain only finite values.');
end

state = struct();
state.p_ned_m = x_packed(1:3);
state.v_b_mps = x_packed(4:6);
state.q_nb = x_packed(7:10);
state.w_b_radps = x_packed(11:13);
state.omega_m_radps = x_packed(14:17);

state = uav.core.state_validate(state);
end
