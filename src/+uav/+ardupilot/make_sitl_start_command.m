function cmd = make_sitl_start_command(cfg)
%MAKE_SITL_START_COMMAND Построить рекомендуемую команду запуска SITL.
% Назначение:
%   Формирует командную строку ручного запуска `ArduPilot SITL`
%   в режиме `JSON` с выводом `MAVLink` на локальный UDP-порт.
%   Функция не выполняет эту команду автоматически.
%
% Входы:
%   cfg - конфигурация средства сопряжения
%
% Выходы:
%   cmd - структура с текстами команд и поясняющим сообщением
%
% Единицы измерения:
%   порты задаются целыми числами
%
% Допущения:
%   Команда предназначена для ручного выполнения оператором после
%   подготовки внешней среды `ArduPilot`.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_normalize_cfg(cfg);

json_target = "JSON:" + cfg.udp_local_ip;
mavlink_target = "udp:" + cfg.mavlink_udp_ip + ":" + string(cfg.mavlink_udp_port);

command_parts = [ ...
    "sim_vehicle.py"; ...
    "-v"; ...
    cfg.sitl_vehicle; ...
    "-f"; ...
    cfg.sitl_frame; ...
    "--model"; ...
    json_target; ...
    "--out"; ...
    mavlink_target];

if cfg.sitl_enable_console
    command_parts(end + 1, 1) = "--console"; %#ok<AGROW>
end

if cfg.sitl_enable_map
    command_parts(end + 1, 1) = "--map"; %#ok<AGROW>
end

cmd = struct();
cmd.command_text = strjoin(command_parts, " ");

if strlength(cfg.ardupilot_root) > 0
    cmd.run_from_root_command = "cd """ + cfg.ardupilot_root + """ && " ...
        + cmd.command_text;
else
    cmd.run_from_root_command = cmd.command_text;
end

if strlength(cfg.wsl_distro_name) > 0
    escaped_command = replace(cmd.run_from_root_command, "'", "''");
    cmd.wsl_command_text = "wsl -d " + cfg.wsl_distro_name ...
        + " -- bash -lc '" + escaped_command + "'";
else
    cmd.wsl_command_text = "wsl -- bash -lc '" ...
        + cmd.run_from_root_command + "'";
end

if ispc
    cmd.message = [ ...
        "Команда запуска сформирована для ручного запуска ArduPilot SITL." ...
        + newline + ...
        "При работе через WSL адрес узла Windows может отличаться от 127.0.0.1." ...
        + newline + ...
        "При необходимости скорректируйте адрес JSON и поток MAVLink " ...
        + mavlink_target + "."];
else
    cmd.message = ...
        "Команда запуска сформирована для ручного запуска ArduPilot SITL в текущей среде.";
end
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать конфигурацию запуска SITL.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:make_sitl_start_command:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

default_cfg = uav.ardupilot.default_json_config();
required_fields = { ...
    'udp_local_ip', ...
    'mavlink_udp_ip', ...
    'mavlink_udp_port', ...
    'ardupilot_root', ...
    'wsl_distro_name', ...
    'sitl_vehicle', ...
    'sitl_frame', ...
    'sitl_enable_console', ...
    'sitl_enable_map'};

for idx = 1:numel(required_fields)
    field_name = required_fields{idx};
    if ~isfield(cfg, field_name) || isempty(cfg.(field_name))
        cfg.(field_name) = default_cfg.(field_name);
    end
end

cfg.udp_local_ip = string(cfg.udp_local_ip);
cfg.mavlink_udp_ip = string(cfg.mavlink_udp_ip);
cfg.mavlink_udp_port = double(cfg.mavlink_udp_port);
cfg.ardupilot_root = string(cfg.ardupilot_root);
cfg.wsl_distro_name = string(cfg.wsl_distro_name);
cfg.sitl_vehicle = string(cfg.sitl_vehicle);
cfg.sitl_frame = string(cfg.sitl_frame);
cfg.sitl_enable_console = logical(cfg.sitl_enable_console);
cfg.sitl_enable_map = logical(cfg.sitl_enable_map);
end
