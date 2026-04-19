function cmd = make_sitl_start_command(cfg)
%MAKE_SITL_START_COMMAND Построить рекомендуемую команду запуска SITL.
% Назначение:
%   Формирует командную строку запуска `ArduPilot SITL` в режиме `JSON`
%   без автоматического выполнения этой команды. Функция используется
%   сценариями подготовки среды и проверки первого двоичного пакета.
%
% Входы:
%   cfg - конфигурация средства сопряжения
%
% Выходы:
%   cmd - структура с текстом команды и поясняющим сообщением
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   Команда предназначена для ручного выполнения оператором после
%   подготовки внешней среды `ArduPilot`.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_normalize_cfg(cfg);
json_target = "JSON:" + cfg.udp_local_ip;

cmd = struct();
cmd.command_text = sprintf( ...
    'sim_vehicle.py -v %s -f %s --model %s --console --map', ...
    char(cfg.sitl_vehicle), ...
    char(cfg.sitl_frame), ...
    char(json_target));

if strlength(cfg.ardupilot_root) > 0
    cmd.run_from_root_command = sprintf( ...
        'cd "%s" && %s', ...
        char(cfg.ardupilot_root), ...
        cmd.command_text);
else
    cmd.run_from_root_command = cmd.command_text;
end

if strlength(cfg.wsl_distro_name) > 0
    cmd.wsl_command_text = sprintf( ...
        'wsl -d %s -- bash -lc ''%s''', ...
        char(cfg.wsl_distro_name), ...
        strrep(cmd.run_from_root_command, '''', ''''''));
else
    cmd.wsl_command_text = ...
        "wsl -- bash -lc '" + string(cmd.run_from_root_command) + "'";
end

if ispc
    cmd.message = [ ...
        "Команда запуска сформирована для ручного запуска ArduPilot SITL." + newline + ...
        "При работе через WSL адрес узла Windows может отличаться от 127.0.0.1." + newline + ...
        "При необходимости скорректируйте IP-адрес в параметре JSON:..." ];
else
    cmd.message = ...
        "Команда запуска сформирована для ручного запуска ArduPilot SITL в текущей среде.";
end
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать конфигурацию запуска.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:make_sitl_start_command:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

default_cfg = uav.ardupilot.default_json_config();
required_fields = { ...
    'udp_local_ip', ...
    'ardupilot_root', ...
    'wsl_distro_name', ...
    'sitl_vehicle', ...
    'sitl_frame'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name) || isempty(cfg.(field_name))
        cfg.(field_name) = default_cfg.(field_name);
    end
end

cfg.udp_local_ip = string(cfg.udp_local_ip);
cfg.ardupilot_root = string(cfg.ardupilot_root);
cfg.wsl_distro_name = string(cfg.wsl_distro_name);
cfg.sitl_vehicle = string(cfg.sitl_vehicle);
cfg.sitl_frame = string(cfg.sitl_frame);
end
