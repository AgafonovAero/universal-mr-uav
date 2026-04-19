function packet = make_loopback_servo_packet(mode, time_s, cfg)
%MAKE_LOOPBACK_SERVO_PACKET Построить тестовый пакет команд ШИМ.
% Назначение:
%   Формирует детерминированный заменитель будущих команд ШИМ от
%   `ArduPilot`. В TASK-10 эта вспомогательная функция используется для
%   проверки границы сопряжения без реального JSON/UDP-обмена.
%
% Входы:
%   mode   - режим `"hover"` или `"yaw_step"`
%   time_s - время моделирования [s]
%   cfg    - необязательная конфигурация сопряжения с ArduPilot
%
% Выходы:
%   packet - скалярная структура с полями `mode`, `time_s`, `pwm_us` и
%            `message`
%
% Единицы измерения:
%   `time_s` [s], `pwm_us` [us]
%
% Допущения:
%   Тестовый пакет не представляет реальную логику режимов или взведения
%   `ArduPilot`.

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
        message = "Проверочный пакет режима зависания с постоянными командами ШИМ.";
    case "yaw_step"
        pwm_us = pwm_hover_us;
        if time_s >= cfg.loopback_yaw_step_time_s
            yaw_sign = local_default_yaw_signs(cfg.motor_count);
            pwm_us = pwm_hover_us + cfg.loopback_yaw_delta_us .* yaw_sign;
            message = "Проверочный пакет после синтетического ступенчатого изменения ШИМ по рысканию.";
        else
            message = "Проверочный пакет до синтетического ступенчатого изменения ШИМ по рысканию.";
        end
    otherwise
        error('uav:ardupilot:make_loopback_servo_packet:UnsupportedMode', ...
            'Неподдерживаемый режим проверочного прогона "%s".', mode_name);
end

pwm_us = min(max(pwm_us, cfg.pwm_min_us), cfg.pwm_max_us);

packet = struct();
packet.mode = mode_name;
packet.time_s = time_s;
packet.pwm_us = pwm_us(:);
packet.message = message;
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить конфигурацию тестового пакета.

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
            'Ожидалось наличие поля cfg.%s.', field_name);
    end
end
end

function mode_name = local_normalize_mode_name(mode_value)
%LOCAL_NORMALIZE_MODE_NAME Нормализовать обозначение режима прогона.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
elseif ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
else
    error('uav:ardupilot:make_loopback_servo_packet:ModeType', ...
        'Ожидалась строка или символьный вектор в аргументе mode.');
end
end

function yaw_sign = local_default_yaw_signs(motor_count)
%LOCAL_DEFAULT_YAW_SIGNS Вернуть детерминированный знак для рыскания.

yaw_sign = (-1.0) .^ ((0:(motor_count - 1)).');
end
