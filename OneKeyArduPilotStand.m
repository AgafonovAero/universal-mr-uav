function result = OneKeyArduPilotStand(mode, varargin)
%ONEKEYARDUPILOTSTAND Тонкий входной сценарий однокнопочного стенда.
% Назначение:
%   Подготавливает пути MATLAB и передает управление основной логике
%   `scripts/onekey_ardupilot_stand.m`.
%
% Входы:
%   mode     - строковый режим работы: check, install, start, stop,
%              status или full
%   varargin - необязательная структура с переопределением конфигурации
%
% Выходы:
%   result - структура результата выбранного режима
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Основная логика стенда реализована отдельно, чтобы входной сценарий
%   оставался коротким и прозрачным для сопровождения.

repo_root = fileparts(mfilename('fullpath'));

src_path = fullfile(repo_root, 'src');
if exist('uav.setup.default_stand_config', 'file') ~= 2
    addpath(src_path);
end

scripts_path = fullfile(repo_root, 'scripts');
if exist('onekey_ardupilot_stand', 'file') ~= 2
    addpath(scripts_path);
end

result = onekey_ardupilot_stand(mode, varargin{:});
end
