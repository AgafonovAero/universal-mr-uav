function motor_cmd_radps = pwm_to_motor_radps(pwm_us, params, cfg)
%PWM_TO_MOTOR_RADPS Преобразовать команды ШИМ в частоты вращения винтов.
% Description:
%   Преобразует длительности импульсов ШИМ в нормированное управляющее
%   воздействие на интервале `[0, 1]`, а затем переносит его в диапазон
%   допустимых частот вращения из `params.motor`. Ограничение диапазона
%   задается явно, скрытые калибровочные параметры не используются.
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
%   Семантика ШИМ на текущем этапе задается как линейная инженерная
%   заготовка для проверочных прогонов и требует последующего согласования
%   с реальным `ArduPilot`.

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
        'Ожидалось, что params.motor.omega_max_radps >= params.motor.omega_min_radps.');
end

validateattributes(cfg.pwm_min_us, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, 'cfg.pwm_min_us');
validateattributes(cfg.pwm_max_us, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, 'cfg.pwm_max_us');
if cfg.pwm_max_us <= cfg.pwm_min_us
    error('uav:ardupilot:pwm_to_motor_radps:PwmRange', ...
        'Ожидалось, что cfg.pwm_max_us > cfg.pwm_min_us.');
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
