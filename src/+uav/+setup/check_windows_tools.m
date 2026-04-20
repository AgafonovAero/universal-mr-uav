function info = check_windows_tools(cfg)
%CHECK_WINDOWS_TOOLS Проверить средства Windows для стенда TASK-14.
% Назначение:
%   Анализирует Windows-узел и возвращает сведения о наличии MATLAB,
%   PowerShell, Git, Python, GitHub CLI, WSL и наземных станций управления,
%   необходимых для однокнопочного сценария стенда.
%
% Входы:
%   cfg - необязательная конфигурация стенда
%
% Выходы:
%   info - структура со сведениями о средствах Windows
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Проверка ориентирована на текущий Windows-узел и не выполняет
%   установки программных средств.

if nargin < 1 || isempty(cfg)
    cfg = uav.setup.default_stand_config();
end

info = struct();
info.is_windows = ispc;
info.windows_version = string(system_dependent('getos'));
info.has_powershell = ispc;
info.git = local_probe_tool(["git.exe"; "git"]);
info.python = local_probe_tool(["python.exe"; "python"]);
info.github_cli = local_probe_tool(["gh.exe"; "gh"]);
info.matlab = local_probe_tool(["matlab.exe"; "matlab"]);
info.wsl = local_probe_tool(["wsl.exe"]);
info.ground_stations = uav.setup.check_ground_control_stations(cfg);

info.is_ready = info.is_windows ...
    && info.has_powershell ...
    && info.git.is_available ...
    && info.python.is_available ...
    && info.matlab.is_available ...
    && info.wsl.is_available;

info.messages = [ ...
    "Windows: " + local_bool_text(info.is_windows); ...
    "PowerShell: " + local_bool_text(info.has_powershell); ...
    "Git: " + local_tool_message(info.git); ...
    "Python: " + local_tool_message(info.python); ...
    "GitHub CLI: " + local_tool_message(info.github_cli); ...
    "MATLAB: " + local_tool_message(info.matlab); ...
    "WSL command: " + local_tool_message(info.wsl); ...
    info.ground_stations.messages(:)];
end

function tool_info = local_probe_tool(names)
%LOCAL_PROBE_TOOL Проверить наличие системного средства.

tool_info = struct();
tool_info.is_available = false;
tool_info.name = "";
tool_info.path = "";

for idx = 1:numel(names)
    command_name = string(names(idx));
    [status, output] = system(char("where " + command_name));
    if status == 0
        output_lines = splitlines(string(output));
        output_lines = strip(output_lines);
        output_lines = output_lines(strlength(output_lines) > 0);
        if ~isempty(output_lines)
            tool_info.is_available = true;
            tool_info.name = command_name;
            tool_info.path = output_lines(1);
            return;
        end
    end
end
end

function text_value = local_tool_message(tool_info)
%LOCAL_TOOL_MESSAGE Сформировать сообщение по средству Windows.

if tool_info.is_available
    text_value = "обнаружено, путь: " + tool_info.path;
else
    text_value = "не обнаружено";
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
