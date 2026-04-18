function motor_norm_01 = hover_trim_from_params(params)
%HOVER_TRIM_FROM_PARAMS Compute one nominal normalized hover motor command.
% Description:
%   Converts the minimal hover thrust requirement from the vehicle
%   parameters into a 4x1 normalized actuator command vector.
%
% Inputs:
%   params - parameter struct with mass, gravity, rotor, and motor limits
%
% Outputs:
%   motor_norm_01 - 4x1 normalized motor command in [0, 1]
%
% Units:
%   dimensionless normalized command
%
% Assumptions:
%   The baseline vehicle uses four identical rotors with the simple
%   T = kT * omega^2 propulsion model.

required_fields = {'mass_kg', 'gravity_mps2', 'rotor', 'motor'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(params, field_name)
        error('uav:sil:hover_trim_from_params:MissingField', ...
            'Expected params.%s to be present.', field_name);
    end
end

validateattributes(params.mass_kg, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'params.mass_kg');
validateattributes(params.gravity_mps2, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.gravity_mps2');

if ~isstruct(params.rotor) || ~isfield(params.rotor, 'kT_N_per_radps2')
    error('uav:sil:hover_trim_from_params:MissingRotorField', ...
        'Expected params.rotor.kT_N_per_radps2 to be present.');
end

validateattributes(params.rotor.kT_N_per_radps2, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.rotor.kT_N_per_radps2');

omega_min_radps = local_read_motor_limit(params, 'omega_min_radps');
omega_max_radps = local_read_motor_limit(params, 'omega_max_radps');
omega_span_radps = omega_max_radps - omega_min_radps;
if omega_span_radps <= 0.0
    error('uav:sil:hover_trim_from_params:MotorRange', ...
        'Expected params.motor.omega_max_radps > params.motor.omega_min_radps.');
end

hover_omega_radps = sqrt((params.mass_kg * params.gravity_mps2) / ...
    (4.0 * params.rotor.kT_N_per_radps2));
hover_norm_01 = (hover_omega_radps - omega_min_radps) ./ omega_span_radps;
hover_norm_01 = min(max(hover_norm_01, 0.0), 1.0);

motor_norm_01 = repmat(hover_norm_01, 4, 1);
end

function value = local_read_motor_limit(params, field_name)
%LOCAL_READ_MOTOR_LIMIT Validate one motor limit from params.motor.
% Description:
%   Reads one scalar motor speed limit from params.motor.
%
% Inputs:
%   params     - parameter struct
%   field_name - field name under params.motor
%
% Outputs:
%   value - validated motor speed limit
%
% Units:
%   radians per second
%
% Assumptions:
%   params.motor is present and scalar.

if ~isstruct(params.motor) || ~isfield(params.motor, field_name)
    error('uav:sil:hover_trim_from_params:MissingMotorField', ...
        'Expected params.motor.%s to be present.', field_name);
end

value = params.motor.(field_name);
validateattributes(value, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ['params.motor.' field_name]);
end
