function python_command_wsl = resolve_wsl_python(cfg)
%RESOLVE_WSL_PYTHON Определить Python с pymavlink внутри WSL.
% Назначение:
%   Возвращает путь к интерпретатору Python в выбранном дистрибутиве WSL,
%   предпочитая уже подготовленную виртуальную среду ArduPilot.
%
% Входы:
%   cfg - структура конфигурации `uav.ardupilot.default_json_config`
%
% Выходы:
%   python_command_wsl - путь или имя команды Python внутри WSL
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   В WSL уже установлен `python3`, а для полноценной диагностики
%   предпочтительно доступен `pymavlink`.

preferred_command = "/home/oaleg/venv-ardupilot/bin/python3";
probe_command = sprintf([ ...
    'wsl -d %s -- bash -lc ', ...
    '''if [ -x %s ]; then printf %s; else command -v python3; fi'''], ...
    char(cfg.wsl_distro_name), ...
    preferred_command, ...
    preferred_command);
[status_code, raw_output] = system(probe_command);
raw_output = strtrim(string(raw_output));

if status_code == 0 && strlength(raw_output) > 0
    python_command_wsl = raw_output;
else
    python_command_wsl = "python3";
end
end
