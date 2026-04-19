%% PRINT_ARDUPILOT_SITL_SETUP_INSTRUCTIONS Напечатать инструкции подготовки среды.
% Назначение:
%   Формирует и выводит текущее состояние локальной среды, перечень
%   недостающих компонентов и команды ручной подготовки `ArduPilot SITL`
%   вне репозитория `universal-mr-uav`.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_sitl_setup_instructions - структура с конфигурацией,
%   диагностикой среды и перечнем команд подготовки
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   Сценарий не выполняет установку автоматически и не изменяет внешнюю
%   среду без участия оператора.

cfg = uav.ardupilot.default_json_config();
env_info = uav.ardupilot.inspect_sitl_environment(cfg);
setup = uav.ardupilot.make_sitl_setup_commands(cfg, env_info);

result = struct();
result.cfg = cfg;
result.env_info = env_info;
result.setup = setup;

assignin('base', 'ardupilot_sitl_setup_instructions', result);

fprintf('Подготовка рабочей среды ArduPilot SITL\n');
fprintf('  готовность локальной среды                : %s\n', ...
    local_bool_text(env_info.is_ready));
fprintf('  доступность WSL                           : %s\n', ...
    local_bool_text(env_info.has_wsl));
fprintf('  доступность Python                        : %s\n', ...
    local_bool_text(env_info.has_python));
fprintf('  доступность Git                           : %s\n', ...
    local_bool_text(env_info.has_git));
fprintf('  наличие sim_vehicle.py                    : %s\n', ...
    local_bool_text(env_info.has_sim_vehicle));
fprintf('  каталог ArduPilot                         : %s\n', ...
    char(local_display_or_none(env_info.ardupilot_root)));
fprintf('  команда запуска                           : %s\n', ...
    char(env_info.start_command));

fprintf('\nНедостающие элементы среды\n');
if isempty(env_info.missing_items)
    fprintf('  отсутствующих элементов не выявлено.\n');
else
    for k = 1:numel(env_info.missing_items)
        fprintf('  - %s\n', char(env_info.missing_items(k)));
    end
end

fprintf('\nКоманды подготовки WSL в Windows\n');
local_print_list(setup.windows_commands);

fprintf('\nКоманды внутри WSL\n');
local_print_list(setup.wsl_commands);

fprintf('\nКоманды получения исходных текстов ArduPilot\n');
local_print_list(setup.ardupilot_clone_commands);

fprintf('\nКоманды предварительной сборки ArduPilot SITL\n');
local_print_list(setup.ardupilot_build_commands);

fprintf('\nКоманды запуска ArduCopter в режиме JSON\n');
local_print_list(setup.sitl_start_commands);

fprintf('\nЧто должен сделать оператор перед запуском handshake\n');
fprintf('  1. Подготовить WSL и установить необходимые пакеты.\n');
fprintf('  2. Получить ArduPilot вне репозитория universal-mr-uav.\n');
fprintf('  3. Запустить ArduCopter в режиме JSON одной из приведенных команд.\n');
fprintf('  4. После запуска SITL выполнить scripts/run_ardupilot_wait_for_packet.m.\n');
fprintf('  5. Затем выполнить scripts/run_ardupilot_json_udp_handshake.m.\n');

fprintf('\nПояснения\n');
local_print_list(setup.notes);

function local_print_list(items)
%LOCAL_PRINT_LIST Напечатать массив строк с маркерами списка.

for k = 1:numel(items)
    fprintf('  - %s\n', char(items(k)));
end
end

function value = local_display_or_none(text_value)
%LOCAL_DISPLAY_OR_NONE Вернуть строку или обозначение отсутствия значения.

text_value = string(text_value);
if strlength(text_value) == 0
    value = "<не задан>";
else
    value = text_value;
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end
