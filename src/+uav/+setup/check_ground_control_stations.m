function info = check_ground_control_stations(cfg)
%CHECK_GROUND_CONTROL_STATIONS Проверить Mission Planner и QGroundControl.
% Назначение:
%   Анализирует наличие исполняемых файлов наземных станций управления,
%   наличие соответствующих процессов и определяет, какая станция может
%   быть использована в составе стенда на текущем узле Windows.
%
% Входы:
%   cfg - необязательная конфигурация однокнопочного стенда
%
% Выходы:
%   info - структура со сведениями о Mission Planner и QGroundControl
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Проверка ориентирована на Windows и использует известные каталоги
%   установки, а также список процессов.

if nargin < 1 || isempty(cfg)
    cfg = uav.setup.default_stand_config();
end

info = struct();
info.mission_planner = local_make_station_info( ...
    "MissionPlanner.exe", ...
    "Mission Planner", ...
    [ ...
        fullfile(getenv("ProgramFiles"), "Mission Planner", "MissionPlanner.exe"); ...
        fullfile(getenv("ProgramFiles(x86)"), "Mission Planner", "MissionPlanner.exe"); ...
        fullfile(getenv("LOCALAPPDATA"), "Mission Planner", "MissionPlanner.exe")]);
info.qgroundcontrol = local_make_station_info( ...
    "QGroundControl.exe", ...
    "QGroundControl", ...
    [ ...
        fullfile(getenv("ProgramFiles"), "QGroundControl", "QGroundControl.exe"); ...
        fullfile(getenv("ProgramFiles(x86)"), "QGroundControl", "QGroundControl.exe"); ...
        fullfile(getenv("LOCALAPPDATA"), "QGroundControl", "QGroundControl.exe"); ...
        fullfile(getenv("LOCALAPPDATA"), "Programs", "QGroundControl", "QGroundControl.exe")]);

info.preferred_station = "";
for idx = 1:numel(cfg.ground_station_order)
    candidate = string(cfg.ground_station_order(idx));
    switch candidate
        case "MissionPlanner"
            if info.mission_planner.is_installed
                info.preferred_station = candidate;
                break;
            end
        case "QGroundControl"
            if info.qgroundcontrol.is_installed
                info.preferred_station = candidate;
                break;
            end
    end
end

info.messages = [ ...
    local_station_message("Mission Planner", info.mission_planner); ...
    local_station_message("QGroundControl", info.qgroundcontrol)];
if strlength(info.preferred_station) > 0
    info.messages(end + 1, 1) = ...
        "Предпочтительная доступная наземная станция управления: " ...
        + info.preferred_station;
else
    info.messages(end + 1, 1) = ...
        "Подходящая установленная наземная станция управления не обнаружена.";
end
end

function station = local_make_station_info(process_name, display_name, paths)
%LOCAL_MAKE_STATION_INFO Собрать сведения об одной наземной станции.

paths = string(paths(:));
existing_path = "";
for idx = 1:numel(paths)
    if strlength(paths(idx)) == 0
        continue;
    end
    if isfile(char(paths(idx)))
        existing_path = paths(idx);
        break;
    end
end

[is_running, process_details] = local_is_process_running(process_name);

station = struct();
station.display_name = string(display_name);
station.process_name = string(process_name);
station.is_installed = strlength(existing_path) > 0;
station.executable_path = existing_path;
station.is_running = logical(is_running);
station.process_details = process_details;
station.candidate_paths = paths;
end

function [is_running, details] = local_is_process_running(process_name)
%LOCAL_IS_PROCESS_RUNNING Проверить наличие процесса по имени.

details = strings(0, 1);
if ~ispc
    is_running = false;
    return;
end

command_text = sprintf('tasklist /FI "IMAGENAME eq %s" /FO CSV /NH', process_name);
[status, output] = system(command_text);
output_text = string(strtrim(output));

if status ~= 0 || strlength(output_text) == 0 ...
        || contains(lower(output_text), "no tasks are running")
    is_running = false;
    return;
end

is_running = contains(output_text, erase(string(process_name), ".exe"));
if is_running
    details = splitlines(output_text);
    details = details(strlength(strtrim(details)) > 0);
end
end

function message = local_station_message(display_name, station)
%LOCAL_STATION_MESSAGE Сформировать диагностическое сообщение по станции.

if station.is_installed
    install_text = "обнаружена";
else
    install_text = "не обнаружена";
end

if station.is_running
    run_text = "процесс запущен";
else
    run_text = "процесс не запущен";
end

path_text = station.executable_path;
if strlength(path_text) == 0
    path_text = "<не найдено>";
end

message = string(display_name) + ": " + install_text ...
    + ", " + run_text + ", путь: " + path_text;
end
