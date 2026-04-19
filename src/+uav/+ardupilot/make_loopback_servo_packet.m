function packet = make_loopback_servo_packet(mode, time_s, cfg)
%MAKE_LOOPBACK_SERVO_PACKET Build a fake servo-output packet for loopback.
% Description:
%   Produces a deterministic stand-in for future ArduPilot PWM outputs.
%   TASK-10 uses this helper to exercise the adapter boundary without a
%   real JSON/UDP SITL backend.
%
% Inputs:
%   mode   - "hover" or "yaw_step"
%   time_s - simulation time [s]
%   cfg    - optional ArduPilot adapter config
%
% Outputs:
%   packet - scalar struct with mode, time_s, pwm_us, and message
%
% Units:
%   time_s [s], pwm_us [us]
%
% Assumptions:
%   The loopback packet is a smoke-test placeholder and does not represent
%   real ArduPilot mode logic or arming semantics.

if nargin < 3 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_validate_cfg(cfg);
validateattributes(time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'time_s');

mode_name = local_normalize_mode_name(mode);
pwm_hover_us = repmat(cfg.pwm_hover_us, cfg.motor_count, 1);

switch mode_name
    case "hover"
        pwm_us = pwm_hover_us;
        message = "Loopback hover packet with constant hover PWM.";
    case "yaw_step"
        pwm_us = pwm_hover_us;
        if time_s >= cfg.loopback_yaw_step_time_s
            yaw_sign = local_default_yaw_signs(cfg.motor_count);
            pwm_us = pwm_hover_us + cfg.loopback_yaw_delta_us .* yaw_sign;
            message = "Loopback yaw-step packet after the synthetic PWM step.";
        else
            message = "Loopback yaw-step packet before the synthetic PWM step.";
        end
    otherwise
        error('uav:ardupilot:make_loopback_servo_packet:UnsupportedMode', ...
            'Unsupported loopback mode "%s".', mode_name);
end

pwm_us = min(max(pwm_us, cfg.pwm_min_us), cfg.pwm_max_us);

packet = struct();
packet.mode = mode_name;
packet.time_s = time_s;
packet.pwm_us = pwm_us(:);
packet.message = message;
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Validate the loopback config fields.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:make_loopback_servo_packet:CfgType', ...
        'Expected cfg to be a scalar struct.');
end

required_fields = { ...
    'motor_count', 'pwm_min_us', 'pwm_max_us', 'pwm_hover_us', ...
    'loopback_yaw_step_time_s', 'loopback_yaw_delta_us'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name)
        error('uav:ardupilot:make_loopback_servo_packet:MissingCfgField', ...
            'Expected cfg.%s to be present.', field_name);
    end
end
end

function mode_name = local_normalize_mode_name(mode_value)
%LOCAL_NORMALIZE_MODE_NAME Normalize one loopback mode name.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
elseif ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
else
    error('uav:ardupilot:make_loopback_servo_packet:ModeType', ...
        'Expected mode to be a char vector or string scalar.');
end
end

function yaw_sign = local_default_yaw_signs(motor_count)
%LOCAL_DEFAULT_YAW_SIGNS Return one deterministic yaw-differential pattern.

yaw_sign = (-1.0) .^ ((0:(motor_count - 1)).');
end
