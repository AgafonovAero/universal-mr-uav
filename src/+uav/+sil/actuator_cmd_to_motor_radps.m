function motor_cmd_radps = actuator_cmd_to_motor_radps(act, params)
%ACTUATOR_CMD_TO_MOTOR_RADPS Convert one external actuator packet to motors.
% Description:
%   Maps the canonical external SIL actuator command into a 4x1 motor
%   speed command vector in rad/s without introducing hidden parameters.
%
% Inputs:
%   act    - scalar struct with fields mode and motor_norm_01
%   params - parameter struct with motor speed limits
%
% Outputs:
%   motor_cmd_radps - 4x1 motor speed command [rad/s]
%
% Units:
%   motor_cmd_radps [rad/s]
%
% Assumptions:
%   Only the canonical mode "norm01" is supported at this stage.

local_validate_actuator_cmd(act);

mode_name = local_normalize_mode_name(act.mode);
if mode_name ~= "norm01"
    error('uav:sil:actuator_cmd_to_motor_radps:UnsupportedMode', ...
        'Unsupported actuator mode "%s". Only "norm01" is supported.', ...
        mode_name);
end

omega_min_radps = local_read_motor_limit(params, 'omega_min_radps');
omega_max_radps = local_read_motor_limit(params, 'omega_max_radps');
if omega_max_radps < omega_min_radps
    error('uav:sil:actuator_cmd_to_motor_radps:MotorRange', ...
        'Expected params.motor.omega_max_radps >= params.motor.omega_min_radps.');
end

motor_norm_01 = act.motor_norm_01(:);
motor_norm_01 = min(max(motor_norm_01, 0.0), 1.0);

motor_cmd_radps = omega_min_radps + ...
    motor_norm_01 .* (omega_max_radps - omega_min_radps);
motor_cmd_radps = min(max(motor_cmd_radps, omega_min_radps), omega_max_radps);
end

function local_validate_actuator_cmd(act)
%LOCAL_VALIDATE_ACTUATOR_CMD Validate the canonical actuator command struct.
% Description:
%   Checks that the SIL actuator packet has the expected fields and motor
%   command shape before the numeric conversion is applied.
%
% Inputs:
%   act - scalar actuator-command struct
%
% Outputs:
%   none
%
% Units:
%   not applicable
%
% Assumptions:
%   The caller uses the Stage-1.5+ canonical field names.

if ~isstruct(act) || ~isscalar(act)
    error('uav:sil:actuator_cmd_to_motor_radps:ActuatorType', ...
        'Expected act to be a scalar struct.');
end

required_fields = {'mode', 'motor_norm_01'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(act, field_name)
        error('uav:sil:actuator_cmd_to_motor_radps:MissingField', ...
            'Expected act.%s to be present.', field_name);
    end
end

validateattributes(act.motor_norm_01, {'numeric'}, ...
    {'real', 'finite', 'numel', 4}, mfilename, 'act.motor_norm_01');
end

function mode_name = local_normalize_mode_name(mode_value)
%LOCAL_NORMALIZE_MODE_NAME Normalize one actuator mode value to string.
% Description:
%   Converts a char or string scalar mode identifier into one trimmed lower
%   case string for deterministic validation.
%
% Inputs:
%   mode_value - actuator mode representation
%
% Outputs:
%   mode_name - normalized mode string
%
% Units:
%   not applicable
%
% Assumptions:
%   Only scalar text values are accepted.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
    return;
end

if ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
    return;
end

error('uav:sil:actuator_cmd_to_motor_radps:ModeType', ...
    'Expected act.mode to be a char vector or string scalar.');
end

function value = local_read_motor_limit(params, field_name)
%LOCAL_READ_MOTOR_LIMIT Validate one motor limit from params.motor.
% Description:
%   Reads a scalar motor speed limit from the parameter struct and validates
%   that it is finite and nonnegative.
%
% Inputs:
%   params      - parameter struct
%   field_name  - field name under params.motor
%
% Outputs:
%   value - validated scalar motor limit
%
% Units:
%   radians per second
%
% Assumptions:
%   params.motor exists and stores scalar numeric limits.

if ~isstruct(params) || ~isscalar(params) || ~isfield(params, 'motor') || ...
        ~isstruct(params.motor) || ~isfield(params.motor, field_name)
    error('uav:sil:actuator_cmd_to_motor_radps:MissingMotorField', ...
        'Expected params.motor.%s to be present.', field_name);
end

value = params.motor.(field_name);
validateattributes(value, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ['params.motor.' field_name]);
end
