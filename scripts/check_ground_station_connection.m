%% CHECK_GROUND_STATION_CONNECTION Проверить состояние наземной станции управления.
% Назначение:
%   Печатает сведения о состоянии Mission Planner, QGroundControl,
%   порта MAVLink и связанных UDP-портов стенда.
%
% Входы:
%   none
%
% Выходы:
%   ground_station_connection_status - структура результата в базовом
%   рабочем пространстве MATLAB
%
% Единицы измерения:
%   сетевые порты задаются целыми числами
%
% Допущения:
%   Сценарий не требует фактического запуска наземной станции управления и
%   корректно завершается при ее отсутствии.

cfg = uav.setup.default_stand_config();
status_info = uav.setup.stand_status(cfg);

result = struct();
result.cfg = cfg;
result.status_info = status_info;
result.mission_planner_installed = status_info.ground_stations.mission_planner.is_installed;
result.mission_planner_running = status_info.ground_stations.mission_planner.is_running;
result.qgroundcontrol_installed = status_info.ground_stations.qgroundcontrol.is_installed;
result.qgroundcontrol_running = status_info.ground_stations.qgroundcontrol.is_running;
result.mavlink_port_busy = local_port_busy(status_info, cfg.mavlink_udp_port);
result.json_local_port_busy = local_port_busy(status_info, cfg.json_udp_local_port);
result.json_remote_port_busy = local_port_busy(status_info, cfg.json_udp_remote_port);

assignin('base', 'ground_station_connection_status', result);

fprintf('Диагностика наземной станции управления\n');
fprintf('  Mission Planner установлен            : %s\n', ...
    local_bool_text(result.mission_planner_installed));
fprintf('  Mission Planner запущен               : %s\n', ...
    local_bool_text(result.mission_planner_running));
fprintf('  QGroundControl установлен             : %s\n', ...
    local_bool_text(result.qgroundcontrol_installed));
fprintf('  QGroundControl запущен                : %s\n', ...
    local_bool_text(result.qgroundcontrol_running));
fprintf('  порт MAVLink UDP %d занят             : %s\n', ...
    cfg.mavlink_udp_port, ...
    local_bool_text(result.mavlink_port_busy));
fprintf('  локальный JSON/UDP порт %d занят      : %s\n', ...
    cfg.json_udp_local_port, ...
    local_bool_text(result.json_local_port_busy));
fprintf('  удаленный JSON/UDP порт %d занят      : %s\n', ...
    cfg.json_udp_remote_port, ...
    local_bool_text(result.json_remote_port_busy));

if result.mission_planner_running
    fprintf('  инструкция Mission Planner            : Mission Planner -> UDP -> Connect -> port %d\n', ...
        cfg.mavlink_udp_port);
else
    fprintf('  инструкция Mission Planner            : запустите Mission Planner и выберите UDP-подключение на порту %d\n', ...
        cfg.mavlink_udp_port);
end

if result.qgroundcontrol_running
    fprintf('  инструкция QGroundControl             : при наличии пакетов HEARTBEAT станция обычно обнаруживает аппарат автоматически\n');
else
    fprintf('  инструкция QGroundControl             : при запуске ожидается автоматическое обнаружение потока MAVLink UDP при наличии HEARTBEAT\n');
end

function is_busy = local_port_busy(status_info, port_value)
%LOCAL_PORT_BUSY Проверить занятость заданного порта.

is_busy = false;

for idx = 1:numel(status_info.ports.entries)
    entry = status_info.ports.entries(idx);
    if entry.port == port_value
        is_busy = entry.is_busy;
        return;
    end
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end
