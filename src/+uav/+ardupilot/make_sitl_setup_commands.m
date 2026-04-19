function setup = make_sitl_setup_commands(cfg, env_info)
%MAKE_SITL_SETUP_COMMANDS Сформировать команды подготовки внешней среды.
% Назначение:
%   Формирует человекочитаемый перечень команд, которые оператор может
%   выполнить для подготовки `ArduPilot SITL` вне текущего репозитория.
%   Функция не выполняет установку автоматически и не изменяет систему.
%
% Входы:
%   cfg      - optional config of the ArduPilot exchange layer
%   env_info - optional result of inspect_sitl_environment
%
% Выходы:
%   setup - struct with Windows and WSL command groups
%
% Единицы измерения:
%   not applicable
%
% Допущения:
%   Команды приведены как инженерная инструкция. Перед выполнением
%   оператор должен осознанно проверить их применимость к своей системе.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

if nargin < 2 || isempty(env_info)
    env_info = uav.ardupilot.inspect_sitl_environment(cfg);
end

cfg = local_normalize_cfg(cfg);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);
clone_root = local_default_clone_root(cfg);
ardupilot_root = local_effective_root(cfg, clone_root);
build_target = local_vehicle_build_target(cfg.sitl_vehicle);

setup = struct();
setup.windows_commands = [ ...
    "wsl --status"; ...
    "wsl -l -v"; ...
    "wsl --install"; ...
    "wsl --update"];

setup.wsl_commands = [ ...
    "sudo apt update"; ...
    "sudo apt install -y git python3 python3-pip python3-venv gawk make"; ...
    "python3 --version"; ...
    "git --version"];

setup.ardupilot_clone_commands = [ ...
    "mkdir -p " + clone_root; ...
    "cd " + clone_root; ...
    "git clone https://github.com/ArduPilot/ardupilot.git"; ...
    "cd " + ardupilot_root; ...
    "git submodule update --init --recursive"];

setup.ardupilot_build_commands = [ ...
    "cd " + ardupilot_root; ...
    "./Tools/environment_install/install-prereqs-ubuntu.sh -y"; ...
    "./waf configure --board sitl"; ...
    "./waf " + build_target];

setup.sitl_start_commands = [ ...
    string(start_cmd.command_text); ...
    string(start_cmd.run_from_root_command); ...
    string(start_cmd.wsl_command_text)];

setup.notes = [ ...
    "Команды приведены только для ручной подготовки среды и не выполняются автоматически."; ...
    "Исходные тексты ArduPilot должны располагаться вне репозитория universal-mr-uav."; ...
    "Если WSL отсутствует, его необходимо установить и затем повторить проверку среды."; ...
    "После запуска ArduPilot SITL в режиме JSON следует отдельно запустить сценарий ожидания первого пакета."; ...
    "При работе через WSL может потребоваться замена IP-адреса 127.0.0.1 на адрес узла Windows."; ...
    "Текущий этап не подтверждает устойчивый автоматический полет и не является летной валидацией."];

if ~env_info.is_ready
    setup.notes(end + 1, 1) = ...
        "По результатам проверки среды отсутствуют следующие элементы: " + ...
        strjoin(env_info.missing_items, ", ");
end
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать конфигурацию.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:make_sitl_setup_commands:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

default_cfg = uav.ardupilot.default_json_config();
field_names = {'ardupilot_root', 'wsl_distro_name', 'sitl_vehicle', 'sitl_frame'};

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

function clone_root = local_default_clone_root(cfg)
%LOCAL_DEFAULT_CLONE_ROOT Выбрать типовой каталог размещения ArduPilot.

if strlength(cfg.wsl_distro_name) > 0 || ispc
    clone_root = "~/work";
else
    clone_root = "~/work";
end
end

function root_path = local_effective_root(cfg, clone_root)
%LOCAL_EFFECTIVE_ROOT Определить каталог ArduPilot для команд.

if strlength(cfg.ardupilot_root) > 0
    root_path = cfg.ardupilot_root;
else
    root_path = clone_root + "/ardupilot";
end
end

function target = local_vehicle_build_target(vehicle_name)
%LOCAL_VEHICLE_BUILD_TARGET Вернуть цель сборки `waf`.

vehicle_name = lower(string(vehicle_name));

switch vehicle_name
    case "arducopter"
        target = "copter";
    case "arduplane"
        target = "plane";
    otherwise
        target = "copter";
end
end
