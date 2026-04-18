function state = state_validate(state)
%STATE_VALIDATE Validate the canonical plant state struct.
% Description:
%   Checks that the canonical plant state struct contains every required
%   field with the expected size, numeric type, and finite values.
%
% Inputs:
%   state - struct with fields p_ned_m, v_b_mps, q_nb, w_b_radps,
%           and omega_m_radps
%
% Outputs:
%   state - validated canonical state struct
%
% Units:
%   p_ned_m [m], v_b_mps [m/s], q_nb [-], w_b_radps [rad/s],
%   omega_m_radps [rad/s]
%
% Assumptions:
%   The canonical state API uses scalar-first q_nb and column vectors.

if ~isstruct(state) || ~isscalar(state)
    error('uav:core:state_validate:StateType', ...
        'Expected state to be a scalar struct.');
end

state.p_ned_m = local_validate_field(state, 'p_ned_m', 3);
state.v_b_mps = local_validate_field(state, 'v_b_mps', 3);
state.q_nb = local_validate_field(state, 'q_nb', 4);
state.w_b_radps = local_validate_field(state, 'w_b_radps', 3);
state.omega_m_radps = local_validate_field(state, 'omega_m_radps', 4);

if norm(state.q_nb) <= eps
    error('uav:core:state_validate:QuaternionNorm', ...
        'Expected q_nb to have a nonzero norm.');
end
end

function value = local_validate_field(state, field_name, expected_len)
%LOCAL_VALIDATE_FIELD Validate one canonical state field.

if ~isfield(state, field_name)
    error('uav:core:state_validate:MissingField', ...
        'Missing required state field "%s".', field_name);
end

value = state.(field_name);
if ~isnumeric(value) || ~isreal(value)
    error('uav:core:state_validate:FieldType', ...
        'Expected state.%s to be a real numeric vector.', field_name);
end

if ~isequal(size(value), [expected_len, 1])
    error('uav:core:state_validate:FieldSize', ...
        'Expected state.%s to have size %dx1.', field_name, expected_len);
end

if any(~isfinite(value))
    error('uav:core:state_validate:FieldFinite', ...
        'Expected state.%s to contain only finite values.', field_name);
end
end
