function transport = json_udp_close(transport)
%JSON_UDP_CLOSE Закрыть средство обмена по UDP.
% Назначение:
%   Безопасно закрывает средство обмена `UDP` и возвращает обновленную
%   структуру транспортного уровня.
%
% Входы:
%   transport - структура транспортного уровня
%
% Выходы:
%   transport - обновленная структура после закрытия
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   Функция допускает пустой, уже закрытый или частично инициализированный
%   транспортный объект.

if nargin < 1 || isempty(transport)
    transport = struct();
    transport.is_open = false;
    transport.method = "";
    transport.message = "Средство обмена не было открыто.";
    transport.handle = [];
    return;
end

if ~isstruct(transport) || ~isscalar(transport)
    error('uav:ardupilot:json_udp_close:TransportType', ...
        'Ожидалась скалярная структура transport.');
end

if ~isfield(transport, 'is_open')
    transport.is_open = false;
end

if ~isfield(transport, 'method')
    transport.method = "";
end

if ~isfield(transport, 'handle')
    transport.handle = [];
end

if isempty(transport.handle)
    transport.is_open = false;
    transport.message = "Средство обмена уже закрыто.";
    return;
end

try
    switch string(transport.method)
        case "udp"
            if isvalid(transport.handle)
                fclose(transport.handle);
                delete(transport.handle);
            end
        otherwise
            % Для udpport достаточно удалить последнюю ссылку на объект.
    end
    transport.handle = [];
    transport.is_open = false;
    transport.message = "Средство обмена UDP закрыто.";
catch close_error
    transport.handle = [];
    transport.is_open = false;
    transport.message = "Средство обмена закрыто с предупреждением: " + ...
        string(close_error.message);
end
end
