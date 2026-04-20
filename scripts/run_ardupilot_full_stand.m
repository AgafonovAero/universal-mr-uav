%% RUN_ARDUPILOT_FULL_STAND Выполнить полный dry-run однокнопочного стенда.
% Назначение:
%   Запускает режим `full` сценария `OneKeyArduPilotStand` и сохраняет
%   результат в базовом рабочем пространстве MATLAB.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_full_stand - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   По умолчанию сценарий выполняется без фактического запуска внешней
%   среды и используется как dry-run проверка стенда.

result = OneKeyArduPilotStand("full");
assignin('base', 'ardupilot_full_stand', result);
