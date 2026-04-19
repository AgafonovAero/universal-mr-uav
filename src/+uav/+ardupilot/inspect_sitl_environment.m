function info = inspect_sitl_environment(cfg)
%INSPECT_SITL_ENVIRONMENT Проверить готовность среды ArduPilot SITL.
% Назначение:
%   Анализирует локальную вычислительную среду и определяет, можно ли
%   перейти к ручной подготовке `ArduPilot SITL` и проверке первого
%   двустороннего обмена по `JSON` и `UDP`.
%   Функция не завершает работу аварийно при отсутствии `WSL`,
%   `sim_vehicle.py` или внешнего каталога `ArduPilot`, а возвращает
%   структурированный диагностический результат.
%
% Входы:
%   cfg - необязательная структура конфигурации средства сопряжения
%
% Выходы:
%   info - скалярная структура с признаками готовности и пояснениями
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Под готовностью к реальному обмену понимается наличие минимального
%   набора внешних средств для запуска `ArduPilot SITL` вне текущего
%   репозитория и приема двоичного пакета по `UDP`.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_normalize_cfg(cfg);
repo_root = local_repo_root();
script_paths = local_script_paths(repo_root, cfg);

messages = strings(0, 1);
missing_items = strings(0, 1);

[has_wsl, wsl_distros, wsl_messages] = local_detect_wsl(cfg);
messages = [messages; wsl_messages];

[has_python, python_message] = local_probe_command( ...
    "Python", ...
    local_python_probe_command(), ...
    "Исполняемый файл Python не найден в PATH.");
messages(end + 1, 1) = python_message;
if ~has_python
    missing_items(end + 1, 1) = "Python";
end

[has_git, git_message] = local_probe_command( ...
    "Git", ...
    local_git_probe_command(), ...
    "Исполняемый файл Git не найден в PATH.");
messages(end + 1, 1) = git_message;
if ~has_git
    missing_items(end + 1, 1) = "Git";
end

[has_github_cli, gh_message] = local_probe_command( ...
    "GitHub CLI", ...
    local_github_cli_probe_command(), ...
    "GitHub CLI не найден в PATH.");
messages(end + 1, 1) = gh_message;

[ardupilot_root, has_ardupilot_root, root_messages] = ...
    local_detect_ardupilot_root(cfg, has_wsl, wsl_distros);
messages = [messages; root_messages];

[has_sim_vehicle, sim_vehicle_path, sim_vehicle_messages] = ...
    local_detect_sim_vehicle(cfg, has_wsl, wsl_distros, ardupilot_root);
messages = [messages; sim_vehicle_messages];

start_cmd = uav.ardupilot.make_sitl_start_command(cfg);
start_command = string(start_cmd.command_text);
messages(end + 1, 1) = "Команда запуска SITL сформирована: " + start_command;
messages(end + 1, 1) = string(start_cmd.message);

if ispc && ~has_wsl
    missing_items(end + 1, 1) = "WSL";
end

if ispc && has_wsl && strlength(cfg.wsl_distro_name) > 0
    if isempty(wsl_distros) || ~any(strcmpi(wsl_distros, cfg.wsl_distro_name))
        missing_items(end + 1, 1) = ...
            "WSL distro """ + cfg.wsl_distro_name + """";
        messages(end + 1, 1) = ...
            "Запрошенный дистрибутив WSL не найден: " + cfg.wsl_distro_name;
    else
        messages(end + 1, 1) = ...
            "Запрошенный дистрибутив WSL найден: " + cfg.wsl_distro_name;
    end
end

if ~has_ardupilot_root
    missing_items(end + 1, 1) = "external ArduPilot root";
end

if ~has_sim_vehicle
    missing_items(end + 1, 1) = "sim_vehicle.py";
end

can_form_start_command = strlength(start_command) > 0;

is_ready = has_python && has_git && has_sim_vehicle && can_form_start_command;
if ispc
    is_ready = is_ready && has_wsl;
end

if is_ready
    messages(end + 1, 1) = ...
        "Локальная среда готова к ручному запуску ArduPilot SITL и проверке реального обмена.";
else
    messages(end + 1, 1) = ...
        "Локальная среда пока не готова к подтвержденному обмену с ArduPilot SITL.";
end

info = struct();
info.is_ready = logical(is_ready);
info.has_wsl = logical(has_wsl);
info.wsl_distros = wsl_distros;
info.has_python = logical(has_python);
info.has_git = logical(has_git);
info.has_github_cli = logical(has_github_cli);
info.has_sim_vehicle = logical(has_sim_vehicle);
info.has_ardupilot_root = logical(has_ardupilot_root);
info.ardupilot_root = ardupilot_root;
info.sim_vehicle_path = sim_vehicle_path;
info.start_command = start_command;
info.can_form_start_command = logical(can_form_start_command);
info.script_paths = script_paths;
info.missing_items = unique(missing_items, "stable");
info.messages = messages;
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать структуру конфигурации.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:inspect_sitl_environment:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

default_cfg = uav.ardupilot.default_json_config();
field_names = fieldnames(default_cfg);

for idx = 1:numel(field_names)
    field_name = field_names{idx};
    if ~isfield(cfg, field_name) || isempty(cfg.(field_name))
        cfg.(field_name) = default_cfg.(field_name);
    end
end

cfg.ardupilot_root = string(cfg.ardupilot_root);
cfg.wsl_distro_name = string(cfg.wsl_distro_name);
cfg.sitl_vehicle = string(cfg.sitl_vehicle);
cfg.sitl_frame = string(cfg.sitl_frame);
cfg.tools_windows_dir = string(cfg.tools_windows_dir);
cfg.tools_wsl_dir = string(cfg.tools_wsl_dir);
end

function repo_root = local_repo_root()
%LOCAL_REPO_ROOT Вернуть корневой каталог репозитория.

repo_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end

function script_paths = local_script_paths(repo_root, cfg)
%LOCAL_SCRIPT_PATHS Построить пути к сценариям подготовки среды.

windows_dir = fullfile(repo_root, char(cfg.tools_windows_dir));
wsl_dir = fullfile(repo_root, char(cfg.tools_wsl_dir));

script_paths = struct();
script_paths.test_windows_environment = fullfile( ...
    windows_dir, ...
    'Test-ArduPilotEnvironment.ps1');
script_paths.setup_wsl = fullfile( ...
    windows_dir, ...
    'Setup-WSLForArduPilot.ps1');
script_paths.invoke_wsl_setup = fullfile( ...
    windows_dir, ...
    'Invoke-ArduPilotWslSetup.ps1');
script_paths.start_json_sitl = fullfile( ...
    windows_dir, ...
    'Start-ArduPilotJsonSitl.ps1');
script_paths.wsl_setup_script = fullfile( ...
    wsl_dir, ...
    'setup_ardupilot_wsl.sh');
script_paths.wsl_start_script = fullfile( ...
    wsl_dir, ...
    'start_arducopter_json_sitl.sh');
end

function [has_wsl, distros, messages] = local_detect_wsl(cfg)
%LOCAL_DETECT_WSL Проверить наличие WSL и список дистрибутивов.

messages = strings(0, 1);
distros = strings(0, 1);

if ~ispc
    has_wsl = false;
    messages(end + 1, 1) = ...
        "Проверка WSL пропущена, так как текущий узел не относится к Windows.";
    return;
end

[status, output] = local_run_command("wsl.exe -l -v");
has_wsl = status == 0;

if ~has_wsl
    messages(end + 1, 1) = ...
        "WSL не обнаружен или не отвечает на команду `wsl.exe -l -v`.";
    return;
end

distros = local_parse_wsl_distro_names(output);
if isempty(distros)
    messages(end + 1, 1) = ...
        "WSL обнаружен, однако список дистрибутивов не был получен.";
else
    messages(end + 1, 1) = ...
        "WSL обнаружен. Доступные дистрибутивы: " + strjoin(distros, ", ");
end

if strlength(cfg.wsl_distro_name) > 0
    messages(end + 1, 1) = ...
        "Предпочтительный дистрибутив WSL: " + cfg.wsl_distro_name;
end
end

function names = local_parse_wsl_distro_names(output)
%LOCAL_PARSE_WSL_DISTRO_NAMES Разобрать имена дистрибутивов WSL.

lines = splitlines(string(output));
names = strings(0, 1);

for idx = 1:numel(lines)
    line = strip(lines(idx));
    if strlength(line) == 0
        continue;
    end

    if startsWith(lower(line), "windows subsystem")
        continue;
    end

    if contains(lower(line), "name") && contains(lower(line), "version")
        continue;
    end

    if startsWith(line, "*")
        line = strip(extractAfter(line, 1));
    end

    tokens = regexp(char(line), '^(?<name>.+?)\s{2,}.+$', 'names', 'once');
    if isempty(tokens)
        names(end + 1, 1) = line; %#ok<AGROW>
    else
        names(end + 1, 1) = string(strtrim(tokens.name)); %#ok<AGROW>
    end
end

names = unique(names, "stable");
end

function [has_command, message] = local_probe_command(name, command_text, fallback)
%LOCAL_PROBE_COMMAND Проверить доступность системной команды.

[status, output] = local_run_command(command_text);
has_command = status == 0;

if has_command
    if strlength(output) > 0
        message = name + ": обнаружен -> " + output;
    else
        message = name + ": обнаружен.";
    end
else
    message = name + ": " + fallback;
end
end

function [root_path, has_root, messages] = local_detect_ardupilot_root(cfg, has_wsl, distros)
%LOCAL_DETECT_ARDUPILOT_ROOT Найти внешний каталог ArduPilot.

messages = strings(0, 1);
candidate_roots = strings(0, 1);

if strlength(cfg.ardupilot_root) > 0
    candidate_roots(end + 1, 1) = cfg.ardupilot_root;
end

env_candidates = [ ...
    string(getenv("ARDUPILOT_ROOT")); ...
    string(getenv("ARDUPILOT_HOME")); ...
    string(getenv("ARDUPILOT_DIR"))];
env_candidates = strip(env_candidates);
env_candidates = env_candidates(strlength(env_candidates) > 0);
candidate_roots = [candidate_roots; env_candidates];

root_path = "";
has_root = false;

for idx = 1:numel(candidate_roots)
    candidate = strip(candidate_roots(idx));
    if strlength(candidate) == 0
        continue;
    end

    if local_is_windows_path(candidate)
        if isfolder(char(candidate))
            root_path = candidate;
            has_root = true;
            break;
        end
    elseif has_wsl && local_has_target_distro(cfg, distros)
        [exists_in_wsl, resolved_path] = local_wsl_directory_exists(cfg, candidate);
        if exists_in_wsl
            root_path = resolved_path;
            has_root = true;
            break;
        end
    end
end

if has_root
    messages(end + 1, 1) = ...
        "Внешний каталог ArduPilot обнаружен: " + root_path;
else
    messages(end + 1, 1) = ...
        "Внешний каталог ArduPilot не обнаружен.";
end
end

function [has_sim_vehicle, sim_vehicle_path, messages] = local_detect_sim_vehicle(cfg, has_wsl, distros, ardupilot_root)
%LOCAL_DETECT_SIM_VEHICLE Проверить наличие sim_vehicle.py.

messages = strings(0, 1);
sim_vehicle_path = "";

[status, output] = local_run_command(local_sim_vehicle_probe_command());
if status == 0
    has_sim_vehicle = true;
    sim_vehicle_path = string(strtrim(output));
    messages(end + 1, 1) = ...
        "sim_vehicle.py обнаружен в PATH: " + sim_vehicle_path;
    return;
end

candidate_files = strings(0, 1);

if strlength(ardupilot_root) > 0
    if local_is_windows_path(ardupilot_root)
        candidate_files(end + 1, 1) = fullfile( ...
            ardupilot_root, ...
            "Tools", ...
            "autotest", ...
            "sim_vehicle.py");
    elseif has_wsl && local_has_target_distro(cfg, distros)
        [exists_in_wsl, resolved_path] = local_wsl_file_exists( ...
            cfg, ...
            ardupilot_root + "/Tools/autotest/sim_vehicle.py");
        if exists_in_wsl
            has_sim_vehicle = true;
            sim_vehicle_path = resolved_path;
            messages(end + 1, 1) = ...
                "sim_vehicle.py обнаружен во внешнем каталоге ArduPilot: " + sim_vehicle_path;
            return;
        end
    end
end

for idx = 1:numel(candidate_files)
    candidate = string(candidate_files(idx));
    if isfile(char(candidate))
        has_sim_vehicle = true;
        sim_vehicle_path = candidate;
        messages(end + 1, 1) = ...
            "sim_vehicle.py обнаружен по локальному пути: " + sim_vehicle_path;
        return;
    end
end

if has_wsl && local_has_target_distro(cfg, distros)
    [status_wsl, output_wsl] = local_run_wsl_command(cfg, "command -v sim_vehicle.py");
    if status_wsl == 0 && strlength(output_wsl) > 0
        has_sim_vehicle = true;
        sim_vehicle_path = output_wsl;
        messages(end + 1, 1) = ...
            "sim_vehicle.py обнаружен в PATH выбранного дистрибутива WSL: " + sim_vehicle_path;
        return;
    end
end

has_sim_vehicle = false;
messages(end + 1, 1) = ...
    "sim_vehicle.py не найден ни в PATH, ни во внешнем каталоге ArduPilot.";
end

function flag = local_has_target_distro(cfg, distros)
%LOCAL_HAS_TARGET_DISTRO Проверить, выбран ли доступный дистрибутив WSL.

if strlength(cfg.wsl_distro_name) == 0
    flag = ~isempty(distros);
else
    flag = any(strcmpi(distros, cfg.wsl_distro_name));
end
end

function [exists_flag, resolved_path] = local_wsl_directory_exists(cfg, candidate)
%LOCAL_WSL_DIRECTORY_EXISTS Проверить каталог во внешней среде WSL.

candidate = local_expand_wsl_home(candidate);
bash_cmd = sprintf( ...
    'if [ -d "%s" ]; then cd "%s" && pwd; fi', ...
    char(candidate), ...
    char(candidate));
[status, output] = local_run_wsl_command(cfg, bash_cmd);

exists_flag = status == 0 && strlength(output) > 0;
if exists_flag
    resolved_path = output;
else
    resolved_path = "";
end
end

function [exists_flag, resolved_path] = local_wsl_file_exists(cfg, candidate)
%LOCAL_WSL_FILE_EXISTS Проверить файл во внешней среде WSL.

candidate = local_expand_wsl_home(candidate);
bash_cmd = sprintf( ...
    'if [ -f "%s" ]; then printf "%s" "%s"; fi', ...
    char(candidate), ...
    '%s', ...
    char(candidate));
[status, output] = local_run_wsl_command(cfg, bash_cmd);

exists_flag = status == 0 && strlength(output) > 0;
if exists_flag
    resolved_path = output;
else
    resolved_path = "";
end
end

function candidate = local_expand_wsl_home(candidate)
%LOCAL_EXPAND_WSL_HOME Заменить префикс `~/` на выражение `$HOME/`.

candidate = string(candidate);
if startsWith(candidate, "~/")
    candidate = "$HOME/" + extractAfter(candidate, 2);
elseif candidate == "~"
    candidate = "$HOME";
end
end

function tf = local_is_windows_path(path_value)
%LOCAL_IS_WINDOWS_PATH Проверить, что путь относится к файловой системе Windows.

path_value = string(path_value);
tf = contains(path_value, ":\") || startsWith(path_value, "\\");
end

function [status, output] = local_run_wsl_command(cfg, bash_command)
%LOCAL_RUN_WSL_COMMAND Выполнить команду в выбранном дистрибутиве WSL.

distro_name = string(cfg.wsl_distro_name);
if strlength(distro_name) == 0
    command_text = sprintf( ...
        'wsl.exe -- bash -lc "%s"', ...
        local_escape_cmd_quotes(bash_command));
else
    command_text = sprintf( ...
        'wsl.exe -d "%s" -- bash -lc "%s"', ...
        char(distro_name), ...
        local_escape_cmd_quotes(bash_command));
end

[status, output] = local_run_command(command_text);
end

function escaped = local_escape_cmd_quotes(text_value)
%LOCAL_ESCAPE_CMD_QUOTES Подготовить строку для передачи в `bash -lc`.

escaped = strrep(char(string(text_value)), '"', '\"');
end

function [status, output] = local_run_command(command_text)
%LOCAL_RUN_COMMAND Выполнить системную команду без аварийного завершения.

try
    [status, output] = system(char(command_text));
catch command_error
    status = 1;
    output = command_error.message;
end

output = string(strtrim(output));
if strlength(output) == 0
    output = "";
end
end

function command_text = local_python_probe_command()
%LOCAL_PYTHON_PROBE_COMMAND Вернуть команду проверки Python.

if ispc
    command_text = "where python";
else
    command_text = "command -v python3 || command -v python";
end
end

function command_text = local_git_probe_command()
%LOCAL_GIT_PROBE_COMMAND Вернуть команду проверки Git.

if ispc
    command_text = "where git";
else
    command_text = "command -v git";
end
end

function command_text = local_github_cli_probe_command()
%LOCAL_GITHUB_CLI_PROBE_COMMAND Вернуть команду проверки GitHub CLI.

if ispc
    command_text = "where gh";
else
    command_text = "command -v gh";
end
end

function command_text = local_sim_vehicle_probe_command()
%LOCAL_SIM_VEHICLE_PROBE_COMMAND Вернуть команду проверки sim_vehicle.py.

if ispc
    command_text = "where sim_vehicle.py";
else
    command_text = "command -v sim_vehicle.py";
end
end
