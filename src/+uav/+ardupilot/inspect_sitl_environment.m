function info = inspect_sitl_environment(cfg)
%INSPECT_SITL_ENVIRONMENT Проверить локальную готовность к ArduPilot SITL.
% Назначение:
%   Проверяет локальную вычислительную среду на наличие минимальных
%   средств, необходимых для следующего этапа сопряжения с
%   `ArduPilot SITL`. Функция не завершает работу с ошибкой только из-за
%   отсутствия `WSL`, `Python` или `ArduPilot`.
%
% Входы:
%   cfg - optional adapter config with ardupilot_root
%
% Выходы:
%   info - scalar struct with readiness flags and diagnostic messages
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   TASK-10 ограничен проверкой локальной среды и подготовкой заготовки
%   средства сопряжения.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

cfg = local_normalize_cfg(cfg);
messages = strings(0, 1);

if ispc
    [status_wsl, output_wsl] = local_run_command("wsl.exe --status");
    has_wsl = status_wsl == 0;
    messages(end + 1, 1) = local_status_message( ...
        "WSL", has_wsl, output_wsl, ...
        "WSL не обнаружен или вернул ненулевой код завершения.");
else
    has_wsl = false;
    messages(end + 1, 1) = ...
        "Проверка WSL пропущена, так как узел не относится к Windows.";
end

[status_python, output_python] = local_run_command(local_python_probe_command());
has_python = status_python == 0;
messages(end + 1, 1) = local_status_message( ...
    "Python", has_python, output_python, ...
    "Исполняемый файл Python не найден в PATH.");

[status_sim_vehicle, output_sim_vehicle] = ...
    local_run_command(local_sim_vehicle_probe_command());
has_sim_vehicle = status_sim_vehicle == 0;
messages(end + 1, 1) = local_status_message( ...
    "sim_vehicle.py", has_sim_vehicle, output_sim_vehicle, ...
    "sim_vehicle.py не найден в PATH.");

[ardupilot_root, has_ardupilot_root, root_message] = ...
    local_detect_ardupilot_root(cfg);
messages(end + 1, 1) = root_message;

if ispc
    is_ready = has_wsl && has_python && ...
        (has_sim_vehicle || has_ardupilot_root);
else
    is_ready = has_python && (has_sim_vehicle || has_ardupilot_root);
end

if is_ready
    messages(end + 1, 1) = ...
        "Локальная среда выглядит готовой к следующему этапу с реальным ArduPilot SITL.";
else
    messages(end + 1, 1) = ...
        "Локальная среда пока не готова к реальному ArduPilot SITL; TASK-10 остается на уровне проверочных замкнутых прогонов.";
end

info = struct();
info.is_ready = logical(is_ready);
info.has_wsl = logical(has_wsl);
info.has_python = logical(has_python);
info.has_sim_vehicle = logical(has_sim_vehicle);
info.ardupilot_root = ardupilot_root;
info.messages = messages;
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать необязательную входную конфигурацию.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:inspect_sitl_environment:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

if ~isfield(cfg, 'ardupilot_root') || isempty(cfg.ardupilot_root)
    cfg.ardupilot_root = "";
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
%LOCAL_PYTHON_PROBE_COMMAND Вернуть команду проверки наличия Python.

if ispc
    command_text = "where python";
else
    command_text = "command -v python3 || command -v python";
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

function message = local_status_message(name, is_ok, output, fallback_text)
%LOCAL_STATUS_MESSAGE Сформировать диагностическую строку.

if is_ok
    if strlength(output) > 0
        message = name + ": обнаружен -> " + output;
    else
        message = name + ": обнаружен.";
    end
else
    message = name + ": " + fallback_text;
end
end

function [ardupilot_root, has_root, message] = local_detect_ardupilot_root(cfg)
%LOCAL_DETECT_ARDUPILOT_ROOT Определить возможный локальный каталог ArduPilot.

candidate_roots = strings(0, 1);

if isfield(cfg, 'ardupilot_root') && strlength(string(cfg.ardupilot_root)) > 0
    candidate_roots(end + 1, 1) = string(cfg.ardupilot_root);
end

env_candidates = [ ...
    string(getenv('ARDUPILOT_ROOT')); ...
    string(getenv('ARDUPILOT_HOME')); ...
    string(getenv('ARDUPILOT_DIR'))];
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
    message = "Локальный каталог ArduPilot: обнаружен -> " + ardupilot_root;
else
    message = "Локальный каталог ArduPilot: путь не задан или каталог не найден.";
end
end
