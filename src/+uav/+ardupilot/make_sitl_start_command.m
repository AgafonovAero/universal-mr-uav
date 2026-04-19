function cmd = make_sitl_start_command(cfg)
%MAKE_SITL_START_COMMAND Построить рекомендуемую команду запуска SITL.
% Назначение:
%   Формирует строку команды запуска `ArduPilot` в режиме `JSON`
%   для последующей ручной проверки обмена данными с данным репозиторием.
%
% Входы:
%   cfg - конфигурация средства сопряжения
%
% Выходы:
%   cmd - структура с полями command_text и message
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   Функция не запускает `ArduPilot`, а только подготавливает
%   рекомендуемую командную строку.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:make_sitl_start_command:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

if ~isfield(cfg, 'ardupilot_vehicle') || strlength(string(cfg.ardupilot_vehicle)) == 0
    cfg.ardupilot_vehicle = "ArduCopter";
end

if ~isfield(cfg, 'udp_local_ip') || strlength(string(cfg.udp_local_ip)) == 0
    cfg.udp_local_ip = "127.0.0.1";
end

cmd = struct();
cmd.command_text = sprintf( ...
    'sim_vehicle.py -v %s --model JSON:%s --console --map', ...
    char(cfg.ardupilot_vehicle), ...
    char(string(cfg.udp_local_ip)));

if ispc
    cmd.message = [ ...
        "Команда сформирована для запуска из среды Windows или WSL. " + ...
        "При запуске из WSL адрес узла Windows может отличаться от 127.0.0.1; " + ...
        "при необходимости замените IP-адрес в параметре JSON-модели."];
else
    cmd.message = "Команда сформирована для локального запуска `ArduPilot` в режиме JSON.";
end
end
