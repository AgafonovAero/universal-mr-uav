function [ctx, tx_info] = json_lockstep_write_frame(ctx, frame_or_text)
%JSON_LOCKSTEP_WRITE_FRAME Передать один JSON-ответ в lockstep-цикле.
% Назначение:
%   Сериализует JSON-кадр и отправляет его только в ответ на ранее принятый
%   валидный бинарный кадр ArduPilot. Адрес и порт назначения берутся из
%   последнего валидного отправителя.
%
% Входы:
%   ctx           - структура состояния lockstep-обмена
%   frame_or_text - структура JSON-кадра или готовая строка JSON
%
% Выходы:
%   ctx     - обновленная структура состояния обмена
%   tx_info - структура результата передачи
%
% Единицы измерения:
%   tx_info.payload_bytes - байты
%
% Допущения:
%   Передача без принятого валидного кадра считается нарушением lockstep и
%   подавляется.

ctx = local_validate_ctx(ctx);
tx_info = struct( ...
    'attempted', false, ...
    'sent', false, ...
    'payload_bytes', 0, ...
    'message', "");

if ~ctx.is_open || ~ctx.transport.is_open
    tx_info.message = "UDP-транспорт не открыт.";
    ctx.last_status = tx_info.message;
    return;
end

if ~ctx.last_valid_frame_seen
    tx_info.message = "Нет валидного входного кадра, JSON-ответ подавлен.";
    ctx.last_status = tx_info.message;
    return;
end

if isstruct(frame_or_text)
    json_text = newline + string(jsonencode(frame_or_text)) + newline;
elseif isstring(frame_or_text) || ischar(frame_or_text)
    json_text = string(frame_or_text);
    if ~startsWith(json_text, newline)
        json_text = newline + json_text;
    end
    if ~endsWith(json_text, newline)
        json_text = json_text + newline;
    end
else
    error('uav:ardupilot:json_lockstep_write_frame:FrameType', ...
        'Ожидалась структура JSON-кадра или строка JSON.');
end

payload_bytes = unicode2native(char(json_text), 'UTF-8');
tx_info.attempted = true;
tx_info.payload_bytes = numel(payload_bytes);

try
    switch string(ctx.transport.method)
        case "udpport"
            write( ...
                ctx.transport.handle, ...
                uint8(payload_bytes(:)), ...
                "uint8", ...
                char(ctx.last_sender_address), ...
                double(ctx.last_sender_port));
        case "udp"
            fwrite(ctx.transport.handle, uint8(payload_bytes(:)), 'uint8');
        otherwise
            error('uav:ardupilot:json_lockstep_write_frame:Method', ...
                'Неподдерживаемый метод UDP: %s.', string(ctx.transport.method));
    end

    ctx.transport.tx_count = ctx.transport.tx_count + 1;
    ctx.json_response_tx_count = ctx.json_response_tx_count + 1;
    tx_info.sent = true;
    tx_info.message = "JSON-ответ передан.";
    ctx.last_status = tx_info.message;
catch send_error
    tx_info.sent = false;
    tx_info.message = "Не удалось передать JSON-ответ: " + string(send_error.message);
    ctx.last_status = tx_info.message;
    ctx.last_error = string(send_error.message);
end
end

function ctx = local_validate_ctx(ctx)
if ~isstruct(ctx) || ~isscalar(ctx)
    error('uav:ardupilot:json_lockstep_write_frame:CtxType', ...
        'Ожидалась скалярная структура ctx.');
end
end
