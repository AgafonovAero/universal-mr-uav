%% INSPECT_ARDUPILOT_SITL_ENV Проверить локальную готовность к ArduPilot SITL.
% Назначение:
%   Печатает краткий отчет о наличии локальных средств, необходимых для
%   следующего этапа сопряжения с `ArduPilot SITL`. Сценарий не
%   завершается ошибкой только из-за отсутствия `ArduPilot`.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_sitl_env - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   TASK-10 проверяет только готовность локальной среды и состояние
%   заготовки средства сопряжения для проверочных замкнутых прогонов.

cfg = uav.ardupilot.default_json_config();
info = uav.ardupilot.inspect_sitl_environment(cfg);

assignin('base', 'ardupilot_sitl_env', info);

fprintf('Проверка локальной среды для ArduPilot SITL:\n');
fprintf('  готовность к реальному SITL : %s\n', local_bool_text(info.is_ready));
fprintf('  наличие WSL                 : %s\n', local_bool_text(info.has_wsl));
fprintf('  наличие Python              : %s\n', local_bool_text(info.has_python));
fprintf('  наличие sim_vehicle.py      : %s\n', local_bool_text(info.has_sim_vehicle));

if strlength(string(info.ardupilot_root)) > 0
    fprintf('  каталог ArduPilot           : %s\n', char(info.ardupilot_root));
else
    fprintf('  каталог ArduPilot           : <не найден>\n');
end

fprintf('  сообщения:\n');
for k = 1:numel(info.messages)
    fprintf('    - %s\n', char(info.messages(k)));
end

function text_value = local_bool_text(value)
%LOCAL_BOOL_TEXT Преобразовать логическое значение в краткую строку.

if logical(value)
    text_value = 'да';
else
    text_value = 'нет';
end
end
