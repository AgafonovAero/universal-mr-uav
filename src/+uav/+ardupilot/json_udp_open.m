function transport = json_udp_open(cfg)
%JSON_UDP_OPEN Открыть средство обмена данными по UDP.
% Назначение:
%   Открывает средство обмена `UDP` в MATLAB и подготавливает структуру
%   транспортного уровня для последующих шагов приема и передачи.
%
% Входы:
%   cfg - конфигурация средства сопряжения
%
% Выходы:
%   transport - структура транспортного уровня
%
% Единицы измерения:
%   порты - целые числа, время ожидания - секунды
%
% Допущения:
%   На текущем этапе транспорт открывается для обмена на одном локальном
%   узле без подтверждения штатной эксплуатации.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_validate_cfg(cfg);
availability = uav.ardupilot.json_udp_is_available();

transport = struct();
transport.is_open = false;
transport.method = availability.method;
transport.message = availability.message;
transport.local_ip = string(cfg.udp_local_ip);
transport.local_port = double(cfg.udp_local_port);
transport.remote_ip = string(cfg.udp_remote_ip);
transport.remote_port = double(cfg.udp_remote_port);
transport.timeout_s = double(cfg.udp_timeout_s);
transport.rx_count = 0;
transport.tx_count = 0;
transport.handle = [];

if ~availability.is_available
    transport.message = "Средство UDP недоступно в данной среде MATLAB.";
    return;
end

try
    switch availability.method
        case "udpport"
            handle = udpport( ...
                "byte", ...
                "IPV4", ...
                "LocalPort", double(cfg.udp_local_port), ...
                "Timeout", double(cfg.udp_timeout_s));
        case "udp"
            handle = udp( ...
                char(cfg.udp_remote_ip), ...
                double(cfg.udp_remote_port), ...
                'LocalPort', double(cfg.udp_local_port), ...
                'Timeout', double(cfg.udp_timeout_s));
            fopen(handle);
        otherwise
            error('uav:ardupilot:json_udp_open:UnsupportedMethod', ...
                'Неподдерживаемый метод UDP: %s.', availability.method);
    end

    transport.handle = handle;
    transport.is_open = true;
    transport.message = sprintf( ...
        'Средство UDP открыто методом %s на локальном порту %d.', ...
        availability.method, cfg.udp_local_port);
catch open_error
    transport.is_open = false;
    transport.handle = [];
    transport.message = "Не удалось открыть средство UDP: " + ...
        string(open_error.message);
end
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить конфигурацию транспортного уровня.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:json_udp_open:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

required_fields = { ...
    'udp_local_ip', ...
    'udp_local_port', ...
    'udp_remote_ip', ...
    'udp_remote_port', ...
    'udp_timeout_s'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name)
        error('uav:ardupilot:json_udp_open:MissingCfgField', ...
            'Ожидалось наличие поля cfg.%s.', field_name);
    end
end
end
