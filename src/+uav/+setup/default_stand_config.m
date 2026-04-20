function cfg = default_stand_config()
%DEFAULT_STAND_CONFIG Вернуть типовую конфигурацию однокнопочного стенда.
% Назначение:
%   Формирует явную конфигурацию для сценариев TASK-14, объединяющих
%   подготовку внешней среды ArduPilot SITL, запуск математической модели
%   движения, запуск наземной станции управления и контроль журналов.
%   Конфигурация задает только параметры и пути; сама функция не запускает
%   внешние процессы и не выполняет административные действия.
%
% Входы:
%   none
%
% Выходы:
%   cfg - скалярная структура конфигурации однокнопочного стенда
%
% Единицы измерения:
%   сетевые порты задаются целыми числами;
%   интервалы ожидания задаются в секундах
%
% Допущения:
%   Стенд ориентирован на использование ArduCopter в режиме JSON, обмен
%   по UDP, наземные станции управления Mission Planner и QGroundControl,
%   а также существующие сценарии ожидания пакета и проверки обмена.

repo_root = local_repo_root();
ardupilot_cfg = uav.ardupilot.default_json_config();

cfg = struct();
cfg.mode = "status";

cfg.repo_root = string(repo_root);
cfg.ardupilot_cfg = ardupilot_cfg;

cfg.execute_install = false;
cfg.execute_start = false;
cfg.execute_stop = false;
cfg.allow_ground_station_close = false;

cfg.prefer_ground_station = "MissionPlanner";
cfg.ground_station_order = ["MissionPlanner"; "QGroundControl"];
cfg.launch_mission_planner = true;
cfg.launch_qgroundcontrol = true;

cfg.mavlink_udp_ip = string(ardupilot_cfg.mavlink_udp_ip);
cfg.mavlink_udp_port = ardupilot_cfg.mavlink_udp_port;
cfg.json_udp_ip = string(ardupilot_cfg.udp_local_ip);
cfg.json_udp_local_port = ardupilot_cfg.udp_local_port;
cfg.json_udp_remote_port = ardupilot_cfg.udp_remote_port;

cfg.wsl_distro_name = string(ardupilot_cfg.wsl_distro_name);
cfg.ardupilot_root = string(ardupilot_cfg.ardupilot_root);

cfg.matlab_wait_for_packet_script = "scripts/run_ardupilot_wait_for_packet.m";
cfg.matlab_handshake_script = "scripts/run_ardupilot_json_udp_handshake.m";
cfg.matlab_full_stand_script = "scripts/run_ardupilot_full_stand.m";
cfg.matlab_ground_station_check_script = ...
    "scripts/check_ground_station_connection.m";

cfg.powershell_test_environment = ...
    "tools/ardupilot/windows/Test-ArduPilotEnvironment.ps1";
cfg.powershell_setup_wsl = ...
    "tools/ardupilot/windows/Setup-WSLForArduPilot.ps1";
cfg.powershell_invoke_wsl_setup = ...
    "tools/ardupilot/windows/Invoke-ArduPilotWslSetup.ps1";
cfg.powershell_start_json_sitl = ...
    "tools/ardupilot/windows/Start-ArduPilotJsonSitl.ps1";
cfg.powershell_install_mission_planner = ...
    "tools/ardupilot/windows/Install-MissionPlanner.ps1";
cfg.powershell_install_qgroundcontrol = ...
    "tools/ardupilot/windows/Install-QGroundControl.ps1";
cfg.powershell_start_stand = ...
    "tools/ardupilot/windows/Start-ArduPilotStand.ps1";
cfg.powershell_stop_stand = ...
    "tools/ardupilot/windows/Stop-ArduPilotStand.ps1";
cfg.powershell_test_stand = ...
    "tools/ardupilot/windows/Test-ArduPilotStand.ps1";

cfg.wsl_setup_script = "tools/ardupilot/wsl/setup_ardupilot_wsl.sh";
cfg.wsl_start_sitl_script = ...
    "tools/ardupilot/wsl/start_arducopter_json_sitl.sh";
cfg.wsl_check_script = "tools/ardupilot/wsl/check_ardupilot_wsl.sh";

cfg.log_check = "artifacts/logs/task_14_onekey_check.txt";
cfg.log_install = "artifacts/logs/task_14_onekey_install_dryrun.txt";
cfg.log_start = "artifacts/logs/task_14_onekey_start_dryrun.txt";
cfg.log_status = "artifacts/logs/task_14_onekey_status.txt";
cfg.log_full = "artifacts/logs/task_14_full_stand_dryrun.txt";
cfg.log_bootstrap = "artifacts/logs/task_14_bootstrap.txt";
cfg.log_runtests = "artifacts/logs/task_14_runtests.txt";
cfg.log_install_mission_planner = ...
    "artifacts/logs/task_14_install_mission_planner.txt";
cfg.log_install_qgroundcontrol = ...
    "artifacts/logs/task_14_install_qgroundcontrol.txt";
cfg.log_install_ardupilot_wsl = ...
    "artifacts/logs/task_14_install_ardupilot_wsl.txt";
cfg.log_start_stand = "artifacts/logs/task_14_start_ardupilot_stand.txt";
cfg.log_stop_stand = "artifacts/logs/task_14_stop_ardupilot_stand.txt";
cfg.log_wait_after_start = ...
    "artifacts/logs/task_14_wait_for_packet_after_start.txt";
cfg.log_handshake_after_start = ...
    "artifacts/logs/task_14_handshake_after_start.txt";
cfg.log_ground_station_check = ...
    "artifacts/logs/task_14_ground_station_check.txt";
cfg.report_state = "artifacts/reports/task_14_stand_state.json";
cfg.report_summary = "artifacts/reports/task_14_summary_ru.md";

cfg.notes = [ ...
    "TASK-14 объединяет проверку, подготовку и запуск стенда в единый MATLAB-сценарий."; ...
    "По умолчанию режимы install, start и stop работают в безопасном режиме без параметра Execute."; ...
    "Наземная станция управления не считается подключенной, если отсутствуют пакеты MAVLink или не запущен соответствующий процесс."; ...
    "Состояние процессов, запущенных сценарием TASK-14, сохраняется в отдельном JSON-отчете для безопасной остановки."; ...
    "Стенд не заявляет летную валидацию и не подтверждает устойчивый автоматический полет без фактического приема пакетов ArduPilot."];
end

function repo_root = local_repo_root()
%LOCAL_REPO_ROOT Вернуть корневой каталог репозитория.

repo_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end
