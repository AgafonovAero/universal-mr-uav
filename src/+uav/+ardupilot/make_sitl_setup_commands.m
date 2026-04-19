function setup = make_sitl_setup_commands(cfg, env_info)
%MAKE_SITL_SETUP_COMMANDS Сформировать команды подготовки внешней среды.
% Назначение:
%   Формирует человекочитаемый перечень команд и путей к сценариям,
%   которые оператор может использовать для безопасной подготовки
%   `ArduPilot SITL` вне репозитория `universal-mr-uav`.
%   Функция ничего не устанавливает автоматически и не изменяет систему.
%
% Входы:
%   cfg      - необязательная конфигурация средства сопряжения
%   env_info - необязательный результат `inspect_sitl_environment`
%
% Выходы:
%   setup - структура с путями к сценариям, группами команд
%           и рекомендуемой последовательностью действий
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Команды сформированы как инженерная инструкция. Перед выполнением
%   оператор должен осознанно проверить их применимость к своей системе.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

if nargin < 2 || isempty(env_info)
    env_info = uav.ardupilot.inspect_sitl_environment(cfg);
end

cfg = local_normalize_cfg(cfg);
repo_root = local_repo_root();
script_paths = local_script_paths(repo_root, cfg);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);

wsl_root = local_effective_wsl_root(cfg);
wsl_start_script = local_windows_to_unix_script_path(script_paths.wsl_start_script);
wsl_setup_script = local_windows_to_unix_script_path(script_paths.wsl_setup_script);

setup = struct();
setup.script_paths = script_paths;
setup.recommended_sequence = [ ...
    "1. Run Test-ArduPilotEnvironment.ps1 to inspect Windows and WSL."; ...
    "2. If needed, run Setup-WSLForArduPilot.ps1 to prepare WSL."; ...
    "3. Run Invoke-ArduPilotWslSetup.ps1 to prepare the external ArduPilot tree inside WSL."; ...
    "4. Run Start-ArduPilotJsonSitl.ps1 to start ArduCopter in JSON mode."; ...
    "5. In MATLAB, run scripts/run_ardupilot_wait_for_packet.m."; ...
    "6. After a confirmed packet reception, run scripts/run_ardupilot_json_udp_handshake.m."];

setup.windows_commands = [ ...
    "powershell -ExecutionPolicy Bypass -File """ + string(script_paths.test_windows_environment) + """"; ...
    "powershell -ExecutionPolicy Bypass -File """ + string(script_paths.setup_wsl) + """"; ...
    "powershell -ExecutionPolicy Bypass -File """ + string(script_paths.invoke_wsl_setup) + """"; ...
    "powershell -ExecutionPolicy Bypass -File """ + string(script_paths.start_json_sitl) + """"];

setup.wsl_commands = [ ...
    "bash " + wsl_setup_script; ...
    "bash " + wsl_start_script + " --ip " + string(cfg.udp_local_ip)];

setup.ardupilot_clone_commands = [ ...
    "mkdir -p ~/src"; ...
    "cd ~/src"; ...
    "git clone https://github.com/ArduPilot/ardupilot.git"; ...
    "cd " + wsl_root; ...
    "git submodule update --init --recursive"];

setup.ardupilot_build_commands = [ ...
    "cd " + wsl_root; ...
    "bash Tools/environment_install/install-prereqs-ubuntu.sh -y"; ...
    "source ~/.profile"; ...
    "./waf configure --board sitl"; ...
    "./waf copter"];

setup.sitl_start_commands = [ ...
    string(start_cmd.command_text); ...
    string(start_cmd.run_from_root_command); ...
    string(start_cmd.wsl_command_text)];

setup.matlab_commands = [ ...
    string(cfg.matlab_wait_for_packet_script); ...
    string(cfg.matlab_handshake_script)];

setup.notes = [ ...
    "Команды приведены только для осознанного операторского выполнения и не запускаются автоматически из MATLAB."; ...
    "Исходные тексты ArduPilot должны располагаться вне репозитория universal-mr-uav."; ...
    "Действия, требующие прав администратора в Windows, должны выполняться только при явном параметре Execute."; ...
    "Подготовка внешнего каталога ArduPilot должна выполняться внутри WSL, а не в каталоге /mnt/c или /mnt/d."; ...
    "Критерий перехода к следующему этапу - фактический прием двоичного пакета от ArduPilot и ответная передача после него."; ...
    "При отсутствии входного двоичного пакета допустима только исходящая пробная передача строки JSON без заявления о подтвержденном двустороннем обмене."];

if ~env_info.is_ready
    setup.notes(end + 1, 1) = ...
        "По результатам проверки среды отсутствуют элементы: " ...
        + strjoin(env_info.missing_items, ", ");
end
end

function cfg = local_normalize_cfg(cfg)
%LOCAL_NORMALIZE_CFG Нормализовать конфигурацию подготовки среды.

if ~isstruct(cfg) || ~isscalar(cfg)
    error( ...
        'uav:ardupilot:make_sitl_setup_commands:CfgType', ...
        'Ожидалась скалярная структура cfg.');
end

default_cfg = uav.ardupilot.default_json_config();
field_names = { ...
    'ardupilot_root', ...
    'wsl_distro_name', ...
    'sitl_vehicle', ...
    'sitl_frame', ...
    'udp_local_ip', ...
    'tools_windows_dir', ...
    'tools_wsl_dir', ...
    'matlab_wait_for_packet_script', ...
    'matlab_handshake_script'};

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
cfg.udp_local_ip = string(cfg.udp_local_ip);
cfg.tools_windows_dir = string(cfg.tools_windows_dir);
cfg.tools_wsl_dir = string(cfg.tools_wsl_dir);
cfg.matlab_wait_for_packet_script = string(cfg.matlab_wait_for_packet_script);
cfg.matlab_handshake_script = string(cfg.matlab_handshake_script);
end

function repo_root = local_repo_root()
%LOCAL_REPO_ROOT Вернуть корневой каталог репозитория.

repo_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end

function script_paths = local_script_paths(repo_root, cfg)
%LOCAL_SCRIPT_PATHS Построить пути к операторским сценариям подготовки.

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

function wsl_root = local_effective_wsl_root(cfg)
%LOCAL_EFFECTIVE_WSL_ROOT Определить рекомендуемый каталог ArduPilot в WSL.

wsl_root = string(cfg.ardupilot_root);
if strlength(wsl_root) == 0 || contains(wsl_root, ":\")
    wsl_root = "~/src/ardupilot";
end
end

function unix_path = local_windows_to_unix_script_path(script_path)
%LOCAL_WINDOWS_TO_UNIX_SCRIPT_PATH Преобразовать путь Windows в путь WSL.

script_path = char(string(script_path));

if numel(script_path) >= 3 && script_path(2) == ':' ...
        && (script_path(3) == '\' || script_path(3) == '/')
    drive_letter = lower(string(script_path(1)));
    tail = string(script_path(3:end));
    tail = replace(tail, "\", "/");
    if startsWith(tail, "/")
        tail = extractAfter(tail, 1);
    end
    unix_path = "/mnt/" + drive_letter + "/" + tail;
else
    unix_path = replace(string(script_path), "\", "/");
end
end
