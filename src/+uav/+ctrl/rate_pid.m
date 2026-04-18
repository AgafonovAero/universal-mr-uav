function [u_cmd, state] = rate_pid(rateCmd_rps, rateMeas_rps, state, gains, dt_s, limits)
%RATE_PID Minimal body-rate PID scaffold with integrator and saturation hooks.
% Description:
%   Computes a PID-style body-rate control command with persistent external
%   state. Integrator and output clipping are implemented as simple hooks.
%
% Inputs:
%   rateCmd_rps   - commanded body rates [rad/s]
%   rateMeas_rps  - measured body rates [rad/s]
%   state         - struct with fields integrator and prevError
%   gains         - struct with fields Kp, Ki, Kd
%   dt_s          - sample time [s]
%   limits        - optional struct with integrator_min, integrator_max,
%                   output_min, and output_max
%
% Outputs:
%   u_cmd - control command vector
%   state - updated controller state
%
% Units:
%   Rates [rad/s], dt [s], command units follow gain selection
%
% Assumptions:
%   Gains are applied elementwise and anti-windup is limited to clipping.

rateCmd_rps = rateCmd_rps(:);
rateMeas_rps = rateMeas_rps(:);
axis_count = numel(rateCmd_rps);

if nargin < 3 || isempty(state)
    state.integrator = zeros(axis_count, 1);
    state.prevError = zeros(axis_count, 1);
end

if nargin < 5 || isempty(dt_s)
    dt_s = 0.0;
end

if nargin < 6
    limits = struct();
end

state = local_prepare_state(state, axis_count);

kp = local_expand_vector(local_get_struct_value(gains, 'Kp', 0.0), axis_count);
ki = local_expand_vector(local_get_struct_value(gains, 'Ki', 0.0), axis_count);
kd = local_expand_vector(local_get_struct_value(gains, 'Kd', 0.0), axis_count);

integrator_min = local_expand_vector(local_get_struct_value(limits, 'integrator_min', -inf), axis_count);
integrator_max = local_expand_vector(local_get_struct_value(limits, 'integrator_max', inf), axis_count);
output_min = local_expand_vector(local_get_struct_value(limits, 'output_min', -inf), axis_count);
output_max = local_expand_vector(local_get_struct_value(limits, 'output_max', inf), axis_count);

error_rps = rateCmd_rps - rateMeas_rps;

if dt_s > 0.0
    state.integrator = state.integrator + error_rps .* dt_s;
    errorRate_rps2 = (error_rps - state.prevError) ./ dt_s;
else
    errorRate_rps2 = zeros(axis_count, 1);
end

state.integrator = min(max(state.integrator, integrator_min), integrator_max);

u_cmd = kp .* error_rps + ki .* state.integrator + kd .* errorRate_rps2;
u_cmd = min(max(u_cmd, output_min), output_max);

state.prevError = error_rps;
end

function state = local_prepare_state(state, axis_count)
%LOCAL_PREPARE_STATE Ensure controller state fields exist and are column vectors.

state.integrator = local_expand_vector(local_get_struct_value(state, 'integrator', 0.0), axis_count);
state.prevError = local_expand_vector(local_get_struct_value(state, 'prevError', 0.0), axis_count);
end

function value = local_get_struct_value(s, fieldName, defaultValue)
%LOCAL_GET_STRUCT_VALUE Read a struct field with a scalar default.

if isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function vec = local_expand_vector(value, axis_count)
%LOCAL_EXPAND_VECTOR Expand a scalar or validate a vector length.

if isscalar(value)
    vec = repmat(value, axis_count, 1);
else
    vec = value(:);
    if numel(vec) ~= axis_count
        error('uav:ctrl:rate_pid:SizeMismatch', ...
            'Expected %d elements, got %d.', axis_count, numel(vec));
    end
end
end
