function motor_cmd_radps = pwm_to_motor_radps(pwm_us, params, cfg)
%PWM_TO_MOTOR_RADPS Convert ArduPilot PWM values into motor rad/s commands.
% Description:
%   Maps PWM microseconds into normalized throttle in [0, 1], then applies
%   the configured motor speed limits from params.motor. Saturation is
%   explicit and no hidden calibration parameters are introduced.
%
% Inputs:
%   pwm_us - motor PWM vector [us]
%   params - vehicle parameter struct with motor speed limits
%   cfg    - ArduPilot adapter config with PWM limits
%
% Outputs:
%   motor_cmd_radps - motor speed command vector [rad/s]
%
% Units:
%   PWM [us], motor command [rad/s]
%
% Assumptions:
%   PWM semantics are linear placeholder semantics for TASK-10 loopback
%   verification and still require later alignment with a real ArduPilot
%   setup.

if nargin < 3 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

validateattributes(pwm_us, {'numeric'}, ...
    {'real', 'finite', 'vector'}, mfilename, 'pwm_us');
pwm_us = pwm_us(:);

omega_min_radps = local_read_motor_limit(params, 'omega_min_radps');
omega_max_radps = local_read_motor_limit(params, 'omega_max_radps');
if omega_max_radps < omega_min_radps
    error('uav:ardupilot:pwm_to_motor_radps:MotorRange', ...
        'Expected params.motor.omega_max_radps >= params.motor.omega_min_radps.');
end

validateattributes(cfg.pwm_min_us, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, 'cfg.pwm_min_us');
validateattributes(cfg.pwm_max_us, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, 'cfg.pwm_max_us');
if cfg.pwm_max_us <= cfg.pwm_min_us
    error('uav:ardupilot:pwm_to_motor_radps:PwmRange', ...
        'Expected cfg.pwm_max_us > cfg.pwm_min_us.');
end

pwm_norm_01 = (pwm_us - cfg.pwm_min_us) ./ ...
    (cfg.pwm_max_us - cfg.pwm_min_us);
pwm_norm_01 = min(max(pwm_norm_01, 0.0), 1.0);

motor_cmd_radps = omega_min_radps + ...
    pwm_norm_01 .* (omega_max_radps - omega_min_radps);
motor_cmd_radps = min(max(motor_cmd_radps, omega_min_radps), omega_max_radps);
end

function value = local_read_motor_limit(params, field_name)
%LOCAL_READ_MOTOR_LIMIT Read and validate one motor speed limit.

if ~isstruct(params) || ~isscalar(params) || ~isfield(params, 'motor') || ...
        ~isstruct(params.motor) || ~isfield(params.motor, field_name)
    error('uav:ardupilot:pwm_to_motor_radps:MissingMotorField', ...
        'Expected params.motor.%s to be present.', field_name);
end

value = params.motor.(field_name);
validateattributes(value, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, ['params.motor.' field_name]);
end
