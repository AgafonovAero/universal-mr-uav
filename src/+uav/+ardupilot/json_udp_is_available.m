function info = json_udp_is_available()
%JSON_UDP_IS_AVAILABLE Проверить доступность средства UDP в MATLAB.
% Назначение:
%   Проверяет, доступно ли в текущей версии MATLAB средство обмена
%   данными по `UDP`. Приоритет отдается механизму `udpport`, затем
%   проверяется устаревший механизм `udp`.
%
% Входы:
%   none
%
% Выходы:
%   info - структура с полями is_available, method и message
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   Проверка ограничивается наличием интерфейса в MATLAB и не подтверждает
%   доступность конкретного удаленного узла.

has_udpport = (exist('udpport', 'file') == 2) || ...
    (exist('udpport', 'builtin') == 5);
has_udp = (exist('udp', 'file') == 2) || ...
    (exist('udp', 'class') == 8);

info = struct();
info.is_available = false;
info.method = "";
info.message = "";

if has_udpport
    info.is_available = true;
    info.method = "udpport";
    info.message = "Доступен механизм UDP `udpport`.";
    return;
end

if has_udp
    info.is_available = true;
    info.method = "udp";
    info.message = "Доступен устаревший механизм UDP `udp`.";
    return;
end

info.message = "В текущей среде MATLAB средство UDP не обнаружено.";
end
