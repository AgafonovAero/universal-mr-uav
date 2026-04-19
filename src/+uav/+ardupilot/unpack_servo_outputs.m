function servo = unpack_servo_outputs(packet, cfg)
%UNPACK_SERVO_OUTPUTS Unpack a canonical servo-output packet.
% Description:
%   Validates and normalizes one future ArduPilot servo-output payload into
%   a deterministic struct used by loopback smoke runners. The function is
%   tolerant to extra PWM channels and reports a message instead of
%   crashing on malformed inputs.
%
% Inputs:
%   packet - scalar struct with packet.pwm_us as 4x1 or Nx1 vector
%   cfg    - optional ArduPilot adapter config
%
% Outputs:
%   servo - scalar struct with pwm_us, motor_pwm_us, valid, and message
%
% Units:
%   PWM values are in microseconds
%
% Assumptions:
%   Only the first cfg.motor_count channels are consumed as motor outputs.

if nargin < 2 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_validate_cfg(cfg);
servo = local_empty_servo(cfg.motor_count);

if ~isstruct(packet) || ~isscalar(packet)
    servo.message = "Expected packet to be a scalar struct.";
    return;
end

if ~isfield(packet, 'pwm_us')
    servo.message = "Expected packet.pwm_us to be present.";
    return;
end

pwm_us = packet.pwm_us;
if ~isnumeric(pwm_us) || ~isreal(pwm_us)
    servo.message = "Expected packet.pwm_us to be a real numeric vector.";
    return;
end

pwm_us = pwm_us(:);
if any(~isfinite(pwm_us))
    servo.message = "Expected packet.pwm_us to contain only finite values.";
    return;
end

if numel(pwm_us) < cfg.motor_count
    servo.message = sprintf( ...
        'Expected at least %d PWM channels, got %d.', ...
        cfg.motor_count, numel(pwm_us));
    return;
end

servo.pwm_us = pwm_us;
servo.motor_pwm_us = pwm_us(1:cfg.motor_count);
servo.valid = true;

if numel(pwm_us) == cfg.motor_count
    servo.message = "Motor PWM packet unpacked successfully.";
else
    servo.message = sprintf( ...
        'Motor PWM packet unpacked successfully; using the first %d channels out of %d.', ...
        cfg.motor_count, numel(pwm_us));
end
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Validate the minimal config fields.

if ~isstruct(cfg) || ~isscalar(cfg) || ~isfield(cfg, 'motor_count')
    error('uav:ardupilot:unpack_servo_outputs:CfgType', ...
        'Expected cfg.motor_count to be present.');
end

validateattributes(cfg.motor_count, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'integer', 'positive'}, ...
    mfilename, 'cfg.motor_count');
end

function servo = local_empty_servo(motor_count)
%LOCAL_EMPTY_SERVO Create one deterministic empty servo struct.

servo = struct();
servo.pwm_us = zeros(motor_count, 1);
servo.motor_pwm_us = zeros(motor_count, 1);
servo.valid = false;
servo.message = "";
end
