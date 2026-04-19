function info = inspect_sitl_environment(cfg)
%INSPECT_SITL_ENVIRONMENT Проверить локальную готовность к ArduPilot SITL.
% Назначение:
%   Анализирует локальную вычислительную среду на наличие средств,
%   необходимых для воспроизводимой проверки обмена с `ArduPilot SITL`.
%   Функция не завершает работу аварийно при отсутствии `WSL`,
%   `sim_vehicle.py` или локального каталога `ArduPilot`, а возвращает
%   структурированный диагностический результат.
%
% Входы:
%   cfg - optional config with fields `ardupilot_root`,
%         `wsl_distro_name`, `sitl_vehicle`, `sitl_frame`
%
% Выходы:
%   info - scalar struct with readiness flags and messages
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   На этапе TASK-12 под готовностью понимается наличие минимальных
%   средств для ручного запуска `ArduPilot SITL` вне текущего репозитория
%   и приема первого двоичного пакета по `UDP`.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_normalize_cfg(cfg);
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

[ardupilot_root, has_ardupilot_root, root_messages] = ...
    local_detect_ardupilot_root(cfg);
messages = [messages; root_messages];

[has_sim_vehicle, sim_vehicle_path, sim_vehicle_messages] = ...
    local_detect_sim_vehicle(cfg, ardupilot_root);
messages = [messages; sim_vehicle_messages];

start_cmd = uav.ardupilot.make_sitl_start_command(cfg);
start_command = string(start_cmd.command_text);
messages(end + 1, 1) = "Команда запуска SITL сформирована: " + start_command;
messages(end + 1, 1) = string(start_cmd.message);

if ispc && ~has_wsl
    missing_items(end + 1, 1) = "WSL";
end

if ~has_sim_vehicle && ~has_ardupilot_root
    missing_items(end + 1, 1) = "sim_vehicle.py или локальный каталог ArduPilot";
end

if strlength(cfg.wsl_distro_name) > 0
    if isempty(wsl_distros) || ~any(strcmpi(wsl_distros, cfg.wsl_distro_name))
        missing_items(end + 1, 1) = ...
            "Дистрибутив WSL """ + cfg.wsl_distro_name + """";
        messages(end + 1, 1) = ...
            "Запрошенный дистрибутив WSL не найден: " + cfg.wsl_distro_name;
    else
        messages(end + 1, 1) = ...
            "Запрошенный дистрибутив WSL найден: " + cfg.wsl_distro_name;
    end
end

if has_sim_vehicle
    messages(end + 1, 1) = ...
        "Исполняемый файл sim_vehicle.py доступен: " + sim_vehicle_path;
end

is_ready = has_python && (has_sim_vehicle || has_ardupilot_root);
if ispc
    is_ready = is_ready && has_wsl;
end

if is_ready
    messages(end + 1, 1) = ...
        "Локальная среда готова к ручной проверке реального обмена с ArduPilot SITL.";
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
info.has_sim_vehicle = logical(has_sim_vehicle);
info.ardupilot_root = ardupilot_root;
info.start_command = start_command;
info.missing_items = unique(missing_items, "stable");
info.messages = messages;
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать входную конфигурацию.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:inspect_sitl_environment:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

default_cfg = uav.ardupilot.default_json_config();
field_names = fieldnames(default_cfg);

for k = 1:numel(field_names)
    field_name = field_names{k};
    if ~isfield(cfg, field_name) || isempty(cfg.(field_name))
        cfg.(field_name) = default_cfg.(field_name);
    end
end

cfg.ardupilot_root = string(cfg.ardupilot_root);
cfg.wsl_distro_name = string(cfg.wsl_distro_name);
cfg.sitl_vehicle = string(cfg.sitl_vehicle);
cfg.sitl_frame = string(cfg.sitl_frame);
end

function [has_wsl, distros, messages] = local_detect_wsl(cfg)
%LOCAL_DETECT_WSL Проверить доступность WSL и список дистрибутивов.

messages = strings(0, 1);
distros = strings(0, 1);

if ~ispc
    has_wsl = false;
    messages(end + 1, 1) = ...
        "Проверка WSL пропущена, так как узел не относится к Windows.";
    return;
end

[status_wsl, output_wsl] = local_run_command("wsl.exe -l -q");
has_wsl = status_wsl == 0;

if has_wsl
    distros = splitlines(output_wsl);
    distros = strip(distros);
    distros = distros(strlength(distros) > 0);
    if isempty(distros)
        messages(end + 1, 1) = ...
            "WSL обнаружен, но список дистрибутивов не получен.";
    else
        messages(end + 1, 1) = ...
            "WSL обнаружен. Доступные дистрибутивы: " + strjoin(distros, ", ");
    end
else
    messages(end + 1, 1) = ...
        "WSL не обнаружен или не отвечает на команду `wsl.exe -l -q`.";
end

if strlength(cfg.wsl_distro_name) > 0
    messages(end + 1, 1) = ...
        "Предпочтительный дистрибутив WSL: " + cfg.wsl_distro_name;
end
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

function [ardupilot_root, has_root, messages] = local_detect_ardupilot_root(cfg)
%LOCAL_DETECT_ARDUPILOT_ROOT Определить локальный каталог ArduPilot.

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

ardupilot_root = "";
has_root = false;

for k = 1:numel(candidate_roots)
    candidate = strtrim(candidate_roots(k));
    if strlength(candidate) == 0
        continue;
    end

    if isfolder(char(candidate))
        ardupilot_root = candidate;
        has_root = true;
        break;
    end
end

if has_root
    messages(end + 1, 1) = ...
        "Локальный каталог ArduPilot обнаружен: " + ardupilot_root;
else
    messages(end + 1, 1) = ...
        "Локальный каталог ArduPilot не обнаружен.";
end
end

function [has_sim_vehicle, sim_vehicle_path, messages] = ...
        local_detect_sim_vehicle(cfg, ardupilot_root)
%LOCAL_DETECT_SIM_VEHICLE Проверить доступность sim_vehicle.py.

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
    candidate_files(end + 1, 1) = fullfile(ardupilot_root, "Tools", "autotest", "sim_vehicle.py");
end

if strlength(cfg.ardupilot_root) > 0
    candidate_files(end + 1, 1) = fullfile(cfg.ardupilot_root, "Tools", "autotest", "sim_vehicle.py");
end

has_sim_vehicle = false;
for k = 1:numel(candidate_files)
    candidate_file = string(candidate_files(k));
    if isfile(char(candidate_file))
        has_sim_vehicle = true;
        sim_vehicle_path = candidate_file;
        break;
    end
end

if has_sim_vehicle
    messages(end + 1, 1) = ...
        "sim_vehicle.py обнаружен по локальному пути: " + sim_vehicle_path;
else
    messages(end + 1, 1) = ...
        "sim_vehicle.py не найден ни в PATH, ни в локальном каталоге ArduPilot.";
end
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

function command_text = local_sim_vehicle_probe_command()
%LOCAL_SIM_VEHICLE_PROBE_COMMAND Вернуть команду проверки sim_vehicle.py.

if ispc
    command_text = "where sim_vehicle.py";
else
    command_text = "command -v sim_vehicle.py";
end
end
