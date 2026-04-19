function result = onekey_ardupilot_stand(mode, varargin)
%ONEKEY_ARDUPILOT_STAND Выполнить однокнопочный сценарий стенда TASK-14.
% Назначение:
%   Реализует режимы check, install, start, stop, status и full для
%   подготовки и запуска стенда ArduPilot SITL, MATLAB-модели и наземных
%   станций управления.
%
% Входы:
%   mode     - строковый режим работы
%   varargin - необязательная структура с переопределением конфигурации
%
% Выходы:
%   result - структура результата режима
%
% Единицы измерения:
%   сетевые порты задаются целыми числами
%
% Допущения:
%   По умолчанию режимы install, start и stop работают в безопасном
%   режиме без фактического выполнения опасных действий.

cfg = local_parse_cfg(varargin{:});
mode = local_normalize_mode(mode);

switch mode
    case "check"
        [result, lines] = local_run_check_mode(cfg);
        log_path = cfg.log_check;
    case "install"
        [result, lines] = local_run_install_mode(cfg);
        log_path = cfg.log_install;
    case "start"
        [result, lines] = local_run_start_mode(cfg);
        log_path = cfg.log_start;
    case "stop"
        [result, lines] = local_run_stop_mode(cfg);
        log_path = cfg.log_stop_stand;
    case "status"
        [result, lines] = local_run_status_mode(cfg);
        log_path = cfg.log_status;
    case "full"
        [result, lines] = local_run_full_mode(cfg);
        log_path = cfg.log_full;
    otherwise
        error( ...
            'uav:setup:onekey_ardupilot_stand:Mode', ...
            'Неизвестный режим работы: %s.', ...
            mode);
end

result.mode = mode;
result.cfg = cfg;

local_write_utf8_text(fullfile(char(cfg.repo_root), char(log_path)), strjoin(lines, newline) + newline);
end

function cfg = local_parse_cfg(varargin)
%LOCAL_PARSE_CFG Подготовить конфигурацию однокнопочного сценария.

cfg = uav.setup.default_stand_config();

if nargin == 0
    return;
end

if nargin == 1 && isstruct(varargin{1}) && isscalar(varargin{1})
    cfg = local_apply_struct_override(cfg, varargin{1});
    return;
end

if mod(nargin, 2) ~= 0
    error( ...
        'uav:setup:onekey_ardupilot_stand:CfgOverride', ...
        ['Ожидалась одна скалярная структура с переопределением ' ...
        'конфигурации или набор пар имя-значение.']);
end

for idx = 1:2:nargin
    name = lower(string(varargin{idx}));
    value = varargin{idx + 1};

    switch name
        case "execute"
            execute_flag = logical(value);
            cfg.execute_install = execute_flag;
            cfg.execute_start = execute_flag;
            cfg.execute_stop = execute_flag;
        case "executeinstall"
            cfg.execute_install = logical(value);
        case "executestart"
            cfg.execute_start = logical(value);
        case "executestop"
            cfg.execute_stop = logical(value);
        case "groundstation"
            cfg = local_apply_ground_station(cfg, value);
        case "distroname"
            cfg.wsl_distro_name = string(value);
            cfg.ardupilot_cfg.wsl_distro_name = string(value);
        case "sitlip"
            cfg.json_udp_ip = string(value);
        case "allowgroundstationclose"
            cfg.allow_ground_station_close = logical(value);
        otherwise
            if isfield(cfg, char(name))
                cfg.(char(name)) = value;
            else
                error( ...
                    'uav:setup:onekey_ardupilot_stand:CfgOverrideField', ...
                    'Неизвестное переопределение конфигурации: %s.', ...
                    name);
            end
    end
end
end

function mode = local_normalize_mode(mode)
%LOCAL_NORMALIZE_MODE Нормализовать строку режима.

mode = lower(string(mode));
allowed = ["check"; "install"; "start"; "stop"; "status"; "full"];

if ~any(mode == allowed)
    error( ...
        'uav:setup:onekey_ardupilot_stand:ModeValue', ...
        'Режим %s не поддерживается.', ...
        mode);
end
end

function [result, lines] = local_run_check_mode(cfg)
%LOCAL_RUN_CHECK_MODE Выполнить режим check.

status_info = uav.setup.stand_status(cfg);

lines = strings(0, 1);
lines = local_add_header(lines, "OneKeyArduPilotStand(check)");
lines = local_add_line(lines, "Состояние внешней среды и стенда:");
lines = local_add_line(lines, "  WSL: " + local_bool_text(status_info.wsl.has_wsl_command));
lines = local_add_line(lines, "  целевой дистрибутив WSL: " + local_bool_text(status_info.wsl.has_target_distro));
lines = local_add_line(lines, "  Git в Windows: " + local_bool_text(status_info.windows.git.is_available));
lines = local_add_line(lines, "  Python в Windows: " + local_bool_text(status_info.windows.python.is_available));
lines = local_add_line(lines, "  MATLAB: " + local_bool_text(status_info.windows.matlab.is_available));
lines = local_add_line(lines, "  Git внутри WSL: " + local_bool_text(status_info.wsl.has_git));
lines = local_add_line(lines, "  Python внутри WSL: " + local_bool_text(status_info.wsl.has_python));
lines = local_add_line(lines, "  ArduPilot внутри WSL: " + local_bool_text(status_info.wsl.has_ardupilot_root));
lines = local_add_line(lines, "  sim_vehicle.py: " + local_bool_text(status_info.wsl.has_sim_vehicle));
lines = local_add_line(lines, "  Mission Planner: " + local_bool_text(status_info.ground_stations.mission_planner.is_installed));
lines = local_add_line(lines, "  QGroundControl: " + local_bool_text(status_info.ground_stations.qgroundcontrol.is_installed));
lines = local_add_line(lines, "  UDP в MATLAB: " + local_bool_text(status_info.udp.is_available));
lines = local_add_line(lines, "  сценарий ожидания пакета: " ...
    + local_bool_text(status_info.scripts.wait_for_packet_present));
lines = local_add_line(lines, "  сценарий проверки обмена: " ...
    + local_bool_text(status_info.scripts.handshake_present));
lines = local_add_line(lines, "");
lines = local_add_line(lines, "Диагностические сообщения:");
lines = [lines; "  - " + status_info.messages(:)];

result = struct();
result.success = true;
result.status_info = status_info;
result.requires_operator_action = false;
result.operator_actions = strings(0, 1);

fprintf('%s\n', char(strjoin(lines, newline)));
end

function [result, lines] = local_run_install_mode(cfg)
%LOCAL_RUN_INSTALL_MODE Выполнить режим install.

status_info = uav.setup.stand_status(cfg);
repo_root = char(cfg.repo_root);

[setup_result, setup_block] = local_run_powershell_block( ...
    cfg.powershell_setup_wsl, ...
    ["-DistroName"; cfg.wsl_distro_name], ...
    cfg.execute_install, ...
    fullfile(repo_root, char(cfg.log_install_ardupilot_wsl)));

[invoke_result, invoke_block] = local_run_powershell_block( ...
    cfg.powershell_invoke_wsl_setup, ...
    ["-DistroName"; cfg.wsl_distro_name], ...
    cfg.execute_install, ...
    fullfile(repo_root, char(cfg.log_install_ardupilot_wsl)));

[mission_result, mission_block] = local_run_powershell_block( ...
    cfg.powershell_install_mission_planner, ...
    strings(0, 1), ...
    cfg.execute_install, ...
    fullfile(repo_root, char(cfg.log_install_mission_planner)));

[qgc_result, qgc_block] = local_run_powershell_block( ...
    cfg.powershell_install_qgroundcontrol, ...
    strings(0, 1), ...
    cfg.execute_install, ...
    fullfile(repo_root, char(cfg.log_install_qgroundcontrol)));

lines = strings(0, 1);
lines = local_add_header(lines, "OneKeyArduPilotStand(install)");
lines = local_add_line(lines, "Режим install выполняется в безопасной последовательности.");
lines = [lines; setup_block; ""; invoke_block; ""; mission_block; ""; qgc_block];

operator_actions = local_collect_operator_actions(cfg, status_info);
requires_operator_action = ~isempty(operator_actions);

if requires_operator_action
    action_text = uav.setup.print_operator_action( ...
        "Для продолжения установки выполните перечисленные команды вручную или повторите режим install с разрешенным выполнением.", ...
        operator_actions);
    lines = [lines; ""; splitlines(string(action_text))];
end

result = struct();
result.success = all([setup_result.success; invoke_result.success; mission_result.success; qgc_result.success]);
result.status_info = status_info;
result.setup_result = setup_result;
result.invoke_result = invoke_result;
result.mission_planner_result = mission_result;
result.qgroundcontrol_result = qgc_result;
result.requires_operator_action = requires_operator_action;
result.operator_actions = operator_actions;

fprintf('%s\n', char(strjoin(lines, newline)));
end

function [result, lines] = local_run_start_mode(cfg)
%LOCAL_RUN_START_MODE Выполнить режим start.

repo_root = char(cfg.repo_root);
args = local_make_start_args(cfg);
[start_result, start_block] = local_run_powershell_block( ...
    cfg.powershell_start_stand, ...
    args, ...
    cfg.execute_start, ...
    fullfile(repo_root, char(cfg.log_start_stand)));

lines = strings(0, 1);
lines = local_add_header(lines, "OneKeyArduPilotStand(start)");
lines = local_add_line(lines, "Порт MAVLink UDP: " + string(cfg.mavlink_udp_port));
lines = local_add_line(lines, "Порт JSON/UDP приема MATLAB: " + string(cfg.json_udp_local_port));
lines = local_add_line(lines, "Порт JSON/UDP передачи MATLAB: " + string(cfg.json_udp_remote_port));
lines = [lines; start_block];

wait_result = [];
handshake_result = [];
ground_station_result = [];

if cfg.execute_start && start_result.success
    [wait_result, wait_lines] = local_execute_matlab_script( ...
        cfg, ...
        cfg.matlab_wait_for_packet_script, ...
        fullfile(repo_root, char(cfg.log_wait_after_start)), ...
        'ardupilot_wait_for_packet');
    [handshake_result, handshake_lines] = local_execute_matlab_script( ...
        cfg, ...
        cfg.matlab_handshake_script, ...
        fullfile(repo_root, char(cfg.log_handshake_after_start)), ...
        'ardupilot_json_udp_handshake');
    [ground_station_result, ground_lines] = local_execute_matlab_script( ...
        cfg, ...
        cfg.matlab_ground_station_check_script, ...
        fullfile(repo_root, char(cfg.log_ground_station_check)), ...
        'ground_station_connection_status');

    lines = [lines; ""; wait_lines; ""; handshake_lines; ""; ground_lines];
else
    lines = local_add_line(lines, "");
    lines = local_add_line(lines, "Запуск стенда выполнен в безопасном режиме без фактического старта.");
    lines = local_add_line(lines, "Двоичный пакет от ArduPilot: не проверялся.");
    lines = local_add_line(lines, "Ответная строка JSON: не проверялась.");
    lines = local_add_line(lines, "Подключение наземной станции управления: не проверялось.");
    lines = local_add_line(lines, "Mission Planner -> UDP -> Connect -> port 14550.");
    lines = local_add_line(lines, "QGroundControl при наличии пакетов HEARTBEAT обычно обнаруживает аппарат автоматически.");
end

result = struct();
result.success = start_result.success;
result.start_result = start_result;
result.wait_result = wait_result;
result.handshake_result = handshake_result;
result.ground_station_result = ground_station_result;
result.requires_operator_action = ~cfg.execute_start;
result.operator_actions = local_start_operator_actions(cfg, start_result);

if result.requires_operator_action
    action_text = uav.setup.print_operator_action( ...
        "Для фактического запуска стенда повторите режим start с разрешенным выполнением.", ...
        result.operator_actions);
    lines = [lines; ""; splitlines(string(action_text))];
end

fprintf('%s\n', char(strjoin(lines, newline)));
end

function [result, lines] = local_run_stop_mode(cfg)
%LOCAL_RUN_STOP_MODE Выполнить режим stop.

repo_root = char(cfg.repo_root);
args = ["-StatePath"; fullfile(repo_root, char(cfg.report_state))];
if cfg.allow_ground_station_close
    args(end + 1, 1) = "-AllowGroundStationClose"; %#ok<AGROW>
end

[stop_result, stop_block] = local_run_powershell_block( ...
    cfg.powershell_stop_stand, ...
    args, ...
    cfg.execute_stop, ...
    fullfile(repo_root, char(cfg.log_stop_stand)));

lines = strings(0, 1);
lines = local_add_header(lines, "OneKeyArduPilotStand(stop)");
lines = [lines; stop_block];

result = struct();
result.success = stop_result.success;
result.stop_result = stop_result;
result.requires_operator_action = ~cfg.execute_stop;
result.operator_actions = "Повторите режим stop с разрешенным выполнением, если требуется фактическая остановка процессов.";

fprintf('%s\n', char(strjoin(lines, newline)));
end

function [result, lines] = local_run_status_mode(cfg)
%LOCAL_RUN_STATUS_MODE Выполнить режим status.

status_info = uav.setup.stand_status(cfg);

lines = strings(0, 1);
lines = local_add_header(lines, "OneKeyArduPilotStand(status)");
lines = local_add_line(lines, "WSL: " + local_bool_text(status_info.wsl.has_wsl_command));
lines = local_add_line(lines, "Ubuntu в WSL: " + local_bool_text(status_info.wsl.has_target_distro));
lines = local_add_line(lines, "ArduPilot в WSL: " + local_bool_text(status_info.wsl.has_ardupilot_root));
lines = local_add_line(lines, "sim_vehicle.py: " + local_bool_text(status_info.wsl.has_sim_vehicle));
lines = local_add_line(lines, "Mission Planner установлен: " + local_bool_text(status_info.ground_stations.mission_planner.is_installed));
lines = local_add_line(lines, "QGroundControl установлен: " + local_bool_text(status_info.ground_stations.qgroundcontrol.is_installed));
lines = local_add_line(lines, "Mission Planner запущен: " + local_bool_text(status_info.ground_stations.mission_planner.is_running));
lines = local_add_line(lines, "QGroundControl запущен: " + local_bool_text(status_info.ground_stations.qgroundcontrol.is_running));
lines = local_add_line(lines, "UDP в MATLAB: " + local_bool_text(status_info.udp.is_available));
lines = local_add_line(lines, "Готовность к install: " + local_bool_text(status_info.ready_for_install));
lines = local_add_line(lines, "Готовность к start: " + local_bool_text(status_info.ready_for_start));
lines = local_add_line(lines, "");
lines = local_add_line(lines, "Порты:");

for idx = 1:numel(status_info.ports.entries)
    entry = status_info.ports.entries(idx);
    lines = local_add_line(lines, "  порт " + string(entry.port) + ": " ...
        + local_bool_text(entry.is_busy));
end

lines = local_add_line(lines, "");
lines = local_add_line(lines, "Связанные процессы:");
if isempty(status_info.processes.ardupilot_related)
    lines = local_add_line(lines, "  не обнаружены");
else
    lines = [lines; "  " + status_info.processes.ardupilot_related(:)];
end

lines = local_add_line(lines, "");
lines = local_add_line(lines, "Последние журналы:");
for idx = 1:numel(status_info.latest_logs)
    entry = status_info.latest_logs(idx);
    if entry.exists
        lines = local_add_line(lines, "  " + entry.name + " -> " + entry.timestamp);
    else
        lines = local_add_line(lines, "  " + entry.name + " -> отсутствует");
    end
end

result = struct();
result.success = true;
result.status_info = status_info;
result.requires_operator_action = false;
result.operator_actions = strings(0, 1);

fprintf('%s\n', char(strjoin(lines, newline)));
end

function [result, lines] = local_run_full_mode(cfg)
%LOCAL_RUN_FULL_MODE Выполнить режим full.

[check_result, check_lines] = local_run_check_mode(cfg);
[install_result, install_lines] = local_run_install_mode(cfg);

lines = strings(0, 1);
lines = local_add_header(lines, "OneKeyArduPilotStand(full)");
lines = [lines; "Результат режима check:"; check_lines; ""; "Результат режима install:"; install_lines];

operator_actions = local_collect_operator_actions(cfg, check_result.status_info);
requires_operator_action = ~isempty(operator_actions) || ~cfg.execute_start;

if requires_operator_action
    lines = local_add_line(lines, "");
    lines = local_add_line(lines, "Полный сценарий остановлен до фактического запуска стенда.");
    action_text = uav.setup.print_operator_action( ...
        "Для продолжения выполните указанные действия оператора, затем повторите режим full или start.", ...
        operator_actions);
    lines = [lines; splitlines(string(action_text))];

    result = struct();
    result.success = false;
    result.check_result = check_result;
    result.install_result = install_result;
    result.requires_operator_action = true;
    result.operator_actions = operator_actions;

    fprintf('%s\n', char(strjoin(lines, newline)));
    return;
end

[start_result, start_lines] = local_run_start_mode(cfg);
[status_result, status_lines] = local_run_status_mode(cfg);
lines = [lines; ""; "Результат режима start:"; start_lines; ""; "Результат режима status:"; status_lines];

result = struct();
result.success = start_result.success;
result.check_result = check_result;
result.install_result = install_result;
result.start_result = start_result;
result.status_result = status_result;
result.requires_operator_action = false;
result.operator_actions = strings(0, 1);

fprintf('%s\n', char(strjoin(lines, newline)));
end

function [script_result, block_lines] = local_run_powershell_block(script_rel_path, args, execute_flag, log_path)
%LOCAL_RUN_POWERSHELL_BLOCK Выполнить PowerShell-сценарий и собрать блок текста.

script_result = uav.setup.run_powershell_script( ...
    script_rel_path, ...
    args, ...
    'ExecuteFlag', execute_flag, ...
    'LogPath', log_path);

block_lines = strings(0, 1);
block_lines = local_add_line(block_lines, "PowerShell-сценарий: " + string(script_rel_path));
block_lines = local_add_line(block_lines, "  код завершения: " + string(script_result.status_code));
block_lines = local_add_line(block_lines, "  пояснение: " + script_result.message);
block_lines = local_add_line(block_lines, "  команда: " + script_result.command_text);

if isfile(log_path)
    log_text = splitlines(string(fileread(log_path)));
    block_lines = [block_lines; "  журнал:"; "    " + log_text(:)];
end
end

function [payload, block_lines] = local_execute_matlab_script(cfg, script_rel_path, log_path, base_var_name)
%LOCAL_EXECUTE_MATLAB_SCRIPT Выполнить вспомогательный MATLAB-сценарий.

script_abs_path = fullfile(char(cfg.repo_root), char(script_rel_path));
command_text = sprintf('run(''%s'');', strrep(script_abs_path, '\', '/'));
payload = struct();
payload.success = false;
payload.output_text = "";
payload.base_value = [];
payload.message = "";

try
    output_text = evalc(command_text);
    payload.output_text = string(output_text);

    if evalin('base', sprintf('exist(''%s'', ''var'')', base_var_name))
        payload.base_value = evalin('base', base_var_name);
        payload.success = true;
        payload.message = "MATLAB-сценарий завершился успешно.";
    else
        payload.message = ...
            "MATLAB-сценарий завершился без записи ожидаемой переменной в базовое рабочее пространство.";
    end
catch execution_error
    payload.output_text = string(execution_error.getReport('extended', 'hyperlinks', 'off'));
    payload.message = "MATLAB-сценарий завершился с ошибкой.";
end

block_lines = strings(0, 1);
block_lines = local_add_line(block_lines, "MATLAB-сценарий: " + string(script_rel_path));
block_lines = local_add_line(block_lines, "  статус: " + payload.message);
block_lines = [block_lines; splitlines(payload.output_text)];

local_write_utf8_text(log_path, payload.output_text + newline);
end

function args = local_make_start_args(cfg)
%LOCAL_MAKE_START_ARGS Построить аргументы запуска стенда.

args = [ ...
    "-DistroName"; cfg.wsl_distro_name; ...
    "-Ip"; cfg.json_udp_ip; ...
    "-MavlinkPort"; string(cfg.mavlink_udp_port); ...
    "-StatePath"; fullfile(char(cfg.repo_root), char(cfg.report_state)); ...
    "-LogPath"; fullfile(char(cfg.repo_root), char(cfg.log_start_stand))];

if cfg.launch_mission_planner
    args(end + 1, 1) = "-LaunchMissionPlanner"; %#ok<AGROW>
end

if cfg.launch_qgroundcontrol
    args(end + 1, 1) = "-LaunchQGroundControl"; %#ok<AGROW>
end
end

function actions = local_collect_operator_actions(cfg, status_info)
%LOCAL_COLLECT_OPERATOR_ACTIONS Сформировать перечень действий оператора.

actions = strings(0, 1);

if ~status_info.wsl.has_wsl_command
    actions(end + 1, 1) = ...
        "Запустите PowerShell от имени администратора и выполните " ...
        + "tools/ardupilot/windows/Setup-WSLForArduPilot.ps1 -Execute."; %#ok<AGROW>
end

if status_info.wsl.has_wsl_command && ~status_info.wsl.has_target_distro
    actions(end + 1, 1) = ...
        "Подготовьте дистрибутив Ubuntu через " ...
        + "tools/ardupilot/windows/Setup-WSLForArduPilot.ps1 -Execute."; %#ok<AGROW>
end

if status_info.wsl.has_target_distro && ~status_info.wsl.has_ardupilot_root
    actions(end + 1, 1) = ...
        "Подготовьте внешний каталог ArduPilot внутри WSL через " ...
        + "tools/ardupilot/windows/Invoke-ArduPilotWslSetup.ps1 -Execute."; %#ok<AGROW>
end

if ~status_info.ground_stations.mission_planner.is_installed
    actions(end + 1, 1) = ...
        "При необходимости установите Mission Planner через " ...
        + "tools/ardupilot/windows/Install-MissionPlanner.ps1 -Execute."; %#ok<AGROW>
end

if ~status_info.ground_stations.qgroundcontrol.is_installed
    actions(end + 1, 1) = ...
        "При необходимости установите QGroundControl через " ...
        + "tools/ardupilot/windows/Install-QGroundControl.ps1 -Execute."; %#ok<AGROW>
end

if isempty(actions)
    actions(end + 1, 1) = ...
        "Повторите выбранный режим с параметрами выполнения execute_* в переопределенной конфигурации."; %#ok<AGROW>
end
end

function actions = local_start_operator_actions(cfg, ~)
%LOCAL_START_OPERATOR_ACTIONS Сформировать действия оператора для режима start.

actions = [ ...
    "Подготовьте внешнюю среду через режим install или соответствующие PowerShell-сценарии."; ...
    "После фактического запуска SITL используйте Mission Planner -> UDP -> Connect -> port " + string(cfg.mavlink_udp_port) + "."; ...
    "Если запущен QGroundControl, убедитесь, что поток MAVLink HEARTBEAT действительно появился на UDP-порту."; ...
    "Для фактического старта повторите режим start с разрешенным выполнением."];
end

function lines = local_add_header(lines, title_text)
%LOCAL_ADD_HEADER Добавить заголовок блока.

lines = [lines; string(title_text); string(repmat('=', 1, 60))];
end

function lines = local_add_line(lines, line_text)
%LOCAL_ADD_LINE Добавить строку текста.

lines(end + 1, 1) = string(line_text); %#ok<AGROW>
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

path_value = string(path_value);
folder_path = fileparts(char(path_value));
if strlength(string(folder_path)) > 0 && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(char(path_value), 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:setup:onekey_ardupilot_stand:LogOpen', ...
        'Не удалось открыть журнал %s.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end

function cfg = local_apply_struct_override(cfg, override_cfg)
%LOCAL_APPLY_STRUCT_OVERRIDE Перенести поля скалярной структуры в конфигурацию.

field_names = fieldnames(override_cfg);
for idx = 1:numel(field_names)
    cfg.(field_names{idx}) = override_cfg.(field_names{idx});
end
end

function cfg = local_apply_ground_station(cfg, value)
%LOCAL_APPLY_GROUND_STATION Настроить выбор наземной станции управления.

station_name = lower(string(value));

switch station_name
    case ["missionplanner"; "mission_planner"]
        cfg.prefer_ground_station = "MissionPlanner";
        cfg.launch_mission_planner = true;
        cfg.launch_qgroundcontrol = false;
    case ["qgroundcontrol"; "qgc"]
        cfg.prefer_ground_station = "QGroundControl";
        cfg.launch_mission_planner = false;
        cfg.launch_qgroundcontrol = true;
    case ["both"; "all"]
        cfg.prefer_ground_station = "MissionPlanner";
        cfg.launch_mission_planner = true;
        cfg.launch_qgroundcontrol = true;
    otherwise
        error( ...
            'uav:setup:onekey_ardupilot_stand:GroundStation', ...
            'Неподдерживаемое значение GroundStation: %s.', ...
            station_name);
end
end
