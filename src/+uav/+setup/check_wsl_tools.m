function info = check_wsl_tools(cfg)
%CHECK_WSL_TOOLS Проверить средства внутри WSL для однокнопочного стенда.
% Назначение:
%   Анализирует наличие команды WSL, доступных дистрибутивов и выбранного
%   дистрибутива Ubuntu. При наличии целевого дистрибутива выполняет
%   безопасные диагностические команды для проверки Git, Python,
%   каталога ArduPilot и файла `sim_vehicle.py`.
%
% Входы:
%   cfg - необязательная конфигурация стенда
%
% Выходы:
%   info - структура со сведениями о состоянии WSL
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Отсутствие WSL, Ubuntu, ArduPilot или `sim_vehicle.py` не считается
%   аварией функции и фиксируется в диагностических сообщениях.

if nargin < 1 || isempty(cfg)
    cfg = uav.setup.default_stand_config();
end

info = struct();
info.has_wsl_command = false;
info.available_distros = strings(0, 1);
info.target_distro = string(cfg.wsl_distro_name);
info.has_target_distro = false;
info.has_git = false;
info.has_python = false;
info.has_ardupilot_root = false;
info.ardupilot_root = "";
info.has_sim_vehicle = false;
info.sim_vehicle_path = "";
info.messages = strings(0, 1);
info.is_ready = false;

[status_list, output_list] = system('wsl.exe -l -q');
if status_list == 0
    info.has_wsl_command = true;
    output_list = erase(string(output_list), char(0));
    distros = splitlines(string(output_list));
    distros = strip(distros);
    distros = distros(strlength(distros) > 0);
    info.available_distros = distros;
else
    [status_where, ~] = system('where wsl.exe');
    info.has_wsl_command = status_where == 0;
end

if info.has_wsl_command
    info.messages(end + 1, 1) = "Команда WSL обнаружена.";
else
    info.messages(end + 1, 1) = "Команда WSL не обнаружена.";
    return;
end

if ~isempty(info.available_distros)
    info.messages(end + 1, 1) = ...
        "Доступные дистрибутивы WSL: " + strjoin(info.available_distros, ", ");
else
    info.messages(end + 1, 1) = ...
        "Список установленных дистрибутивов WSL не содержит записей.";
end

if any(strcmpi(info.available_distros, info.target_distro))
    info.has_target_distro = true;
    info.messages(end + 1, 1) = ...
        "Целевой дистрибутив WSL обнаружен: " + info.target_distro;
else
    probe_command = "wsl.exe -d " + info.target_distro + " -- bash -lc ""printf ok""";
    [probe_status, probe_output] = system(char(probe_command));
    probe_output = erase(string(probe_output), char(0));
    if probe_status == 0 && contains(probe_output, "ok")
        info.has_target_distro = true;
        info.available_distros = unique([info.available_distros; info.target_distro], "stable");
        info.messages(end + 1, 1) = ...
            "Целевой дистрибутив WSL подтвержден прямым вызовом: " + info.target_distro;
    else
        info.messages(end + 1, 1) = ...
            "Целевой дистрибутив WSL не обнаружен: " + info.target_distro;
        return;
    end
end

git_result = uav.setup.run_wsl_command( ...
    "command -v git", ...
    'DistroName', ...
    info.target_distro);
info.has_git = git_result.success ...
    && strlength(strtrim(git_result.output_text)) > 0;

python_result = uav.setup.run_wsl_command( ...
    "(command -v python3 || command -v python)", ...
    'DistroName', ...
    info.target_distro);
info.has_python = python_result.success ...
    && strlength(strtrim(python_result.output_text)) > 0;

root_path = local_escape_single_quotes(local_expand_wsl_home_path(string(cfg.ardupilot_root)));
root_command = ...
    "if [ -d '" + root_path + "' ]; then printf '%s' '" + root_path + "'; fi";
root_result = uav.setup.run_wsl_command( ...
    root_command, ...
    'DistroName', ...
    info.target_distro);
if root_result.success && strlength(strtrim(root_result.output_text)) > 0
    info.has_ardupilot_root = true;
    info.ardupilot_root = strtrim(root_result.output_text);
end

sim_vehicle_path = local_expand_wsl_home_path(string(cfg.ardupilot_root)) ...
    + "/Tools/autotest/sim_vehicle.py";
sim_vehicle_path = local_escape_single_quotes(sim_vehicle_path);
sim_command = ...
    "if [ -f '" + sim_vehicle_path + "' ]; then printf '%s' '" ...
    + sim_vehicle_path + "'; fi";
sim_result = uav.setup.run_wsl_command( ...
    sim_command, ...
    'DistroName', ...
    info.target_distro);
if sim_result.success && strlength(strtrim(sim_result.output_text)) > 0
    info.has_sim_vehicle = true;
    info.sim_vehicle_path = strtrim(sim_result.output_text);
end

info.messages = [ ...
    info.messages; ...
    "Git внутри WSL: " + local_bool_text(info.has_git); ...
    "Python внутри WSL: " + local_bool_text(info.has_python); ...
    "Каталог ArduPilot внутри WSL: " + local_bool_text(info.has_ardupilot_root); ...
    "sim_vehicle.py внутри WSL: " + local_bool_text(info.has_sim_vehicle) ...
];

info.is_ready = info.has_target_distro ...
    && info.has_git ...
    && info.has_python ...
    && info.has_ardupilot_root ...
    && info.has_sim_vehicle;
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function escaped_value = local_escape_single_quotes(value)
%LOCAL_ESCAPE_SINGLE_QUOTES Подготовить путь для использования в bash.

value = string(value);
escaped_value = replace(value, "'", "'\''");
end

function expanded_value = local_expand_wsl_home_path(value)
%LOCAL_EXPAND_WSL_HOME_PATH Раскрыть домашний каталог в WSL-пути.

value = string(value);
if startsWith(value, "~/")
    expanded_value = "$HOME/" + extractAfter(value, 2);
elseif value == "~"
    expanded_value = "$HOME";
else
    expanded_value = value;
end
end
