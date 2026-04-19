function out = decode_sitl_output_packet(raw_bytes, cfg)
%DECODE_SITL_OUTPUT_PACKET Разобрать двоичный пакет выходов ArduPilot.
% Назначение:
%   Разбирает двоичный пакет, формируемый `ArduPilot JSON SITL`, и
%   возвращает нормализованную структуру с длительностями импульсов ШИМ,
%   частотой кадров и номером кадра. Функция не завершает работу с
%   ошибкой на пустом, коротком или неизвестном пакете.
%
% Входы:
%   raw_bytes - вектор байтов принятого двоичного пакета
%   cfg       - необязательная конфигурация средства сопряжения
%
% Выходы:
%   out - структура с полями valid, magic, frame_rate_hz, frame_count,
%         pwm_us, motor_pwm_us и message
%
% Единицы измерения:
%   frame_rate_hz - в герцах, pwm_us - в микросекундах
%
% Допущения:
%   Порядок байтов соответствует формату `little-endian`, используемому
%   на целевых узлах `ArduPilot JSON SITL`.

if nargin < 2 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

out = local_make_empty_output();
cfg = local_validate_cfg(cfg);

if nargin < 1 || isempty(raw_bytes)
    out.message = "Двоичный пакет отсутствует.";
    return;
end

if ~isnumeric(raw_bytes) && ~islogical(raw_bytes)
    out.message = "Ожидался числовой вектор байтов.";
    return;
end

raw_bytes = uint8(raw_bytes(:));
if numel(raw_bytes) < 8
    out.message = "Двоичный пакет слишком короток для чтения заголовка.";
    return;
end

magic = double(typecast(raw_bytes(1:2), 'uint16'));
frame_rate_hz = double(typecast(raw_bytes(3:4), 'uint16'));
frame_count = double(typecast(raw_bytes(5:8), 'uint32'));

[channel_count, header_message] = local_channel_count_from_magic(magic, cfg);
if channel_count == 0
    out.magic = magic;
    out.message = header_message;
    return;
end

expected_length = 8 + 2 * channel_count;
if numel(raw_bytes) < expected_length
    out.magic = magic;
    out.frame_rate_hz = frame_rate_hz;
    out.frame_count = frame_count;
    out.message = sprintf([ ...
        'Длина пакета недостаточна: ожидалось не менее %d байт, ', ...
        'получено %d байт.'], expected_length, numel(raw_bytes));
    return;
end

pwm_bytes = raw_bytes(9:expected_length);
pwm_values = double(typecast(pwm_bytes, 'uint16'));

out.valid = true;
out.magic = magic;
out.frame_rate_hz = frame_rate_hz;
out.frame_count = frame_count;
out.pwm_us = pwm_values(:);
out.motor_pwm_us = local_select_motor_channels(out.pwm_us, cfg);
out.message = "Двоичный пакет ArduPilot успешно разобран.";
end

function out = local_make_empty_output()
%LOCAL_MAKE_EMPTY_OUTPUT Построить пустую структуру результата.

out = struct();
out.valid = false;
out.magic = 0;
out.frame_rate_hz = 0;
out.frame_count = 0;
out.pwm_us = zeros(0, 1);
out.motor_pwm_us = zeros(0, 1);
out.message = "";
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить конфигурацию средства сопряжения.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:decode_sitl_output_packet:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

required_fields = { ...
    'motor_count', ...
    'motor_order', ...
    'sitl_magic_16', ...
    'sitl_magic_32', ...
    'sitl_channel_count_16', ...
    'sitl_channel_count_32'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name)
        error('uav:ardupilot:decode_sitl_output_packet:MissingCfgField', ...
            'Ожидалось наличие поля cfg.%s.', field_name);
    end
end

validateattributes(cfg.motor_count, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'integer', 'positive'}, ...
    mfilename, 'cfg.motor_count');
validateattributes(cfg.motor_order, {'numeric'}, ...
    {'real', 'finite', 'numel', cfg.motor_count}, ...
    mfilename, 'cfg.motor_order');
end

function [channel_count, message] = local_channel_count_from_magic(magic, cfg)
%LOCAL_CHANNEL_COUNT_FROM_MAGIC Определить число каналов по magic.

channel_count = 0;
message = "";

if magic == double(cfg.sitl_magic_16)
    channel_count = double(cfg.sitl_channel_count_16);
    return;
end

if magic == double(cfg.sitl_magic_32)
    channel_count = double(cfg.sitl_channel_count_32);
    return;
end

message = sprintf('Неизвестное значение magic: %u.', magic);
end

function motor_pwm_us = local_select_motor_channels(pwm_us, cfg)
%LOCAL_SELECT_MOTOR_CHANNELS Выбрать каналы двигателей из полного вектора.

motor_indices = cfg.motor_order(:);
motor_pwm_us = zeros(cfg.motor_count, 1);

for k = 1:cfg.motor_count
    channel_index = double(motor_indices(k));
    if channel_index < 1 || channel_index > numel(pwm_us)
        error('uav:ardupilot:decode_sitl_output_packet:MotorOrderRange', ...
            'Индекс cfg.motor_order(%d) выходит за пределы пакета ШИМ.', k);
    end

    motor_pwm_us(k) = pwm_us(channel_index);
end
end
