function [ctx, frame] = json_lockstep_read_frame(ctx)
%JSON_LOCKSTEP_READ_FRAME Принять очередной lockstep-кадр ArduPilot SITL.
% Назначение:
%   Считывает все доступные UDP-датаграммы ArduPilot, выбирает последний
%   валидный бинарный кадр, считает пропуски и дубли frame_count и
%   фиксирует сброс контроллера при уменьшении frame_count.
%
% Входы:
%   ctx - структура состояния lockstep-обмена
%
% Выходы:
%   ctx   - обновленная структура состояния обмена
%   frame - структура принятого кадра
%
% Единицы измерения:
%   frame_rate_hz - герцы;
%   motor_pwm_us  - микросекунды
%
% Допущения:
%   За один вызов возвращается не более одного кадра для одного шага
%   физики. Если в буфере накопилось несколько кадров, в обработку
%   передается только последний валидный кадр, а промежуточные считаются
%   пропущенными по frame_count.

ctx = local_validate_ctx(ctx);
frame = local_empty_frame(ctx.cfg);

if ~ctx.is_open || ~ctx.transport.is_open
    frame.message = "UDP-транспорт не открыт.";
    ctx.last_status = frame.message;
    return;
end

last_valid = local_empty_frame(ctx.cfg);

switch string(ctx.transport.method)
    case "udpport"
        while double(ctx.transport.handle.NumDatagramsAvailable) > 0
            datagram = read(ctx.transport.handle, 1, "uint8");
            if isempty(datagram)
                break;
            end

            raw_bytes = uint8(datagram.Data(:));
            sender_address = string(datagram.SenderAddress);
            sender_port = double(datagram.SenderPort);

            ctx.received_datagram_count = ctx.received_datagram_count + 1;
            ctx.transport.rx_count = ctx.transport.rx_count + 1;
            ctx.last_sender_address = sender_address;
            ctx.last_sender_port = sender_port;

            decoded = uav.ardupilot.decode_sitl_output_packet(raw_bytes, ctx.cfg);
            if ~decoded.valid
                ctx.invalid_rx_count = ctx.invalid_rx_count + 1;
                ctx.last_status = string(decoded.message);
                ctx.last_error = string(decoded.message);
                continue;
            end

            candidate = local_update_frame_stats(ctx, decoded, sender_address, sender_port);
            if candidate.duplicate
                ctx.duplicate_frame_count = ctx.duplicate_frame_count + 1;
                ctx.last_status = "Получен дублирующий кадр SITL.";
                continue;
            end
            if candidate.controller_reset
                ctx.controller_reset_count = ctx.controller_reset_count + 1;
            end
            ctx.missed_frame_count = ctx.missed_frame_count + double(candidate.missed_count);
            last_valid = candidate;
            ctx = local_update_ctx_from_frame(ctx, last_valid);
        end

    case "udp"
        bytes_available = double(get(ctx.transport.handle, 'BytesAvailable'));
        if bytes_available > 0
            raw_bytes = fread(ctx.transport.handle, bytes_available, 'uint8');
            raw_bytes = uint8(raw_bytes(:));
            ctx.received_datagram_count = ctx.received_datagram_count + 1;
            ctx.transport.rx_count = ctx.transport.rx_count + 1;
            decoded = uav.ardupilot.decode_sitl_output_packet(raw_bytes, ctx.cfg);
            if ~decoded.valid
                ctx.invalid_rx_count = ctx.invalid_rx_count + 1;
                ctx.last_status = string(decoded.message);
                ctx.last_error = string(decoded.message);
            else
                candidate = local_update_frame_stats( ...
                    ctx, decoded, string(ctx.transport.remote_ip), double(ctx.transport.remote_port));
                if candidate.duplicate
                    ctx.duplicate_frame_count = ctx.duplicate_frame_count + 1;
                    ctx.last_status = "Получен дублирующий кадр SITL.";
                else
                    if candidate.controller_reset
                        ctx.controller_reset_count = ctx.controller_reset_count + 1;
                    end
                    ctx.missed_frame_count = ctx.missed_frame_count + double(candidate.missed_count);
                    last_valid = candidate;
                    ctx = local_update_ctx_from_frame(ctx, last_valid);
                end
            end
        end

    otherwise
        error('uav:ardupilot:json_lockstep_read_frame:Method', ...
            'Неподдерживаемый метод UDP: %s.', string(ctx.transport.method));
end

if last_valid.valid
    frame = last_valid;
    ctx.last_status = "Валидный кадр SITL принят.";
else
    frame.message = "Валидный кадр SITL не получен.";
    ctx.last_status = frame.message;
end
end

function ctx = local_update_ctx_from_frame(ctx, frame)
ctx.valid_rx_count = ctx.valid_rx_count + 1;
ctx.last_magic = double(frame.magic);
ctx.last_frame_rate_hz = double(frame.frame_rate_hz);
ctx.last_frame_count = double(frame.frame_count);
ctx.last_sender_address = string(frame.sender_address);
ctx.last_sender_port = double(frame.sender_port);
ctx.last_motor_pwm_us = double(frame.motor_pwm_us(:));
ctx.transport.remote_ip = string(frame.sender_address);
ctx.transport.remote_port = double(frame.sender_port);
ctx.transport.has_valid_remote = true;
ctx.last_valid_frame_seen = true;
ctx.last_reset_required = logical(frame.controller_reset);
end

function frame = local_update_frame_stats(ctx, decoded, sender_address, sender_port)
frame = local_empty_frame(ctx.cfg);
frame.valid = true;
frame.magic = decoded.magic;
frame.frame_rate_hz = decoded.frame_rate_hz;
frame.frame_count = decoded.frame_count;
frame.pwm_us = decoded.pwm_us;
frame.motor_pwm_us = decoded.motor_pwm_us;
frame.sender_address = string(sender_address);
frame.sender_port = double(sender_port);
frame.message = string(decoded.message);

if ~ctx.last_valid_frame_seen
    frame.controller_reset = false;
    frame.duplicate = false;
    frame.missed_count = 0;
    return;
end

if double(decoded.frame_count) < double(ctx.last_frame_count)
    frame.controller_reset = true;
    frame.duplicate = false;
    frame.missed_count = 0;
    return;
end

if double(decoded.frame_count) == double(ctx.last_frame_count)
    frame.controller_reset = false;
    frame.duplicate = true;
    frame.missed_count = 0;
    return;
end

missed_count = max(double(decoded.frame_count) - double(ctx.last_frame_count) - 1.0, 0.0);
frame.controller_reset = false;
frame.duplicate = false;
frame.missed_count = missed_count;
end

function ctx = local_validate_ctx(ctx)
if ~isstruct(ctx) || ~isscalar(ctx)
    error('uav:ardupilot:json_lockstep_read_frame:CtxType', ...
        'Ожидалась скалярная структура ctx.');
end

required_fields = { ...
    'cfg', 'transport', 'is_open', 'received_datagram_count', 'valid_rx_count', ...
    'duplicate_frame_count', 'missed_frame_count', 'invalid_rx_count', ...
    'json_response_tx_count', 'controller_reset_count', 'last_magic', ...
    'last_frame_rate_hz', 'last_frame_count', 'last_sender_address', ...
    'last_sender_port', 'last_motor_pwm_us', 'last_motor_cmd_radps', ...
    'last_status', 'last_error', 'last_valid_frame_seen', 'last_reset_required'};

for idx = 1:numel(required_fields)
    field_name = required_fields{idx};
    if ~isfield(ctx, field_name)
        error('uav:ardupilot:json_lockstep_read_frame:MissingField', ...
            'Ожидалось поле ctx.%s.', field_name);
    end
end
end

function frame = local_empty_frame(cfg)
frame = struct();
frame.valid = false;
frame.magic = 0;
frame.frame_rate_hz = 0;
frame.frame_count = 0;
frame.pwm_us = zeros(0, 1);
frame.motor_pwm_us = nan(double(cfg.motor_count), 1);
frame.sender_address = "";
frame.sender_port = 0;
frame.duplicate = false;
frame.missed_count = 0;
frame.controller_reset = false;
frame.message = "";
end
