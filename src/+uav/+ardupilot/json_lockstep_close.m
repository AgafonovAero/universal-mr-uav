function ctx = json_lockstep_close(ctx)
%JSON_LOCKSTEP_CLOSE Закрыть официальный lockstep-транспорт ArduPilot JSON.
% Назначение:
%   Закрывает UDP-транспорт и помечает lockstep-контур как завершенный.
%
% Входы:
%   ctx - структура состояния lockstep-обмена
%
% Выходы:
%   ctx - обновленная структура состояния после закрытия

if nargin < 1 || isempty(ctx)
    ctx = struct();
    return;
end

if ~isstruct(ctx) || ~isscalar(ctx)
    error('uav:ardupilot:json_lockstep_close:CtxType', ...
        'Ожидалась скалярная структура ctx.');
end

if isfield(ctx, 'transport')
    ctx.transport = uav.ardupilot.json_udp_close(ctx.transport);
    ctx.is_open = false;
    if isfield(ctx.transport, 'message')
        ctx.last_status = string(ctx.transport.message);
    end
else
    ctx.is_open = false;
end
end
