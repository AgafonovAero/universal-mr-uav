function servo = unpack_servo_outputs(packet, cfg)
%UNPACK_SERVO_OUTPUTS Разобрать канонический пакет команд ШИМ.
% Назначение:
%   Проверяет и нормализует один пакет команд исполнительных органов в
%   детерминированную структуру, используемую в проверочных сценариях.
%   Функция допускает наличие дополнительных каналов ШИМ и возвращает
%   сообщение вместо аварийного завершения при некорректном входе.
%
% Входы:
%   packet - скалярная структура с полем `packet.pwm_us` размерности
%            `4x1` или `Nx1`
%   cfg    - необязательная конфигурация сопряжения с ArduPilot
%
% Выходы:
%   servo - скалярная структура с полями `pwm_us`, `motor_pwm_us`,
%           `valid` и `message`
%
% Единицы измерения:
%   значения ШИМ задаются в микросекундах
%
% Допущения:
%   В качестве команд двигателей используются только первые
%   `cfg.motor_count` каналов.

if nargin < 2 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_validate_cfg(cfg);
servo = local_empty_servo(cfg.motor_count);

if ~isstruct(packet) || ~isscalar(packet)
    servo.message = "Ожидалась скалярная структура packet.";
    return;
end

if ~isfield(packet, 'pwm_us')
    servo.message = "Ожидалось наличие поля packet.pwm_us.";
    return;
end

pwm_us = packet.pwm_us;
if ~isnumeric(pwm_us) || ~isreal(pwm_us)
    servo.message = "Ожидался вещественный числовой вектор packet.pwm_us.";
    return;
end

pwm_us = pwm_us(:);
if any(~isfinite(pwm_us))
    servo.message = "Ожидались только конечные значения в packet.pwm_us.";
    return;
end

if numel(pwm_us) < cfg.motor_count
    servo.message = sprintf( ...
        'Ожидалось не менее %d каналов ШИМ, получено %d.', ...
        cfg.motor_count, numel(pwm_us));
    return;
end

servo.pwm_us = pwm_us;
servo.motor_pwm_us = pwm_us(1:cfg.motor_count);
servo.valid = true;

if numel(pwm_us) == cfg.motor_count
    servo.message = "Пакет команд ШИМ успешно разобран.";
else
    servo.message = sprintf( ...
        'Пакет команд ШИМ успешно разобран; используются первые %d каналов из %d.', ...
        cfg.motor_count, numel(pwm_us));
end
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить минимальный состав конфигурации.

if ~isstruct(cfg) || ~isscalar(cfg) || ~isfield(cfg, 'motor_count')
    error('uav:ardupilot:unpack_servo_outputs:CfgType', ...
        'Ожидалось наличие поля cfg.motor_count.');
end

validateattributes(cfg.motor_count, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'integer', 'positive'}, ...
    mfilename, 'cfg.motor_count');
end

function servo = local_empty_servo(motor_count)
%LOCAL_EMPTY_SERVO Создать детерминированную пустую структуру сервоприводов.

servo = struct();
servo.pwm_us = zeros(motor_count, 1);
servo.motor_pwm_us = zeros(motor_count, 1);
servo.valid = false;
servo.message = "";
end
