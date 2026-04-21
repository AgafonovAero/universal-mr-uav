function wsl_path = windows_to_wsl_path(path_value)
%WINDOWS_TO_WSL_PATH Преобразовать путь Windows в путь WSL.
% Назначение:
%   Переводит абсолютный путь Windows вида `D:\path\to\file` в путь
%   пространства `/mnt/d/path/to/file`, пригодный для команд `bash` внутри
%   `WSL`.
%
% Входы:
%   path_value - абсолютный путь Windows
%
% Выходы:
%   wsl_path - путь, пригодный для `WSL`
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Ожидается локальный путь Windows на диске с буквенным обозначением.

path_value = char(string(path_value));
if numel(path_value) < 3 || path_value(2) ~= ':'
    error('uav:ardupilot:windows_to_wsl_path:PathFormat', ...
        'Ожидался абсолютный путь Windows, получено: %s', path_value);
end

drive_letter = lower(path_value(1));
tail_path = strrep(path_value(3:end), '\', '/');
if startsWith(tail_path, '/')
    tail_path = tail_path(2:end);
end

wsl_path = string("/mnt/" + drive_letter + "/" + tail_path);
end
