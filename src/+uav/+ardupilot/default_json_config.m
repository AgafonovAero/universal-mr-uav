function cfg = default_json_config()
%DEFAULT_JSON_CONFIG Вернуть типовую конфигурацию средства сопряжения.
% Назначение:
%   Формирует явную структуру конфигурации для средства сопряжения
%   `universal-mr-uav` с `ArduPilot` по `JSON` и `UDP`.
%   Конфигурация охватывает:
%   - параметры приема и передачи по `UDP`;
%   - соглашения по системам координат;
%   - ограничения по длительности импульсов ШИМ;
%   - параметры запуска `ArduPilot SITL`;
%   - параметры потока `MAVLink` к наземной станции управления;
%   - пути к сценариям подготовки внешней среды Windows/WSL.
%
% Входы:
%   none
%
% Выходы:
%   cfg - скалярная структура конфигурации средства сопряжения
%
% Единицы измерения:
%   порты задаются целыми числами;
%   длительности импульсов ШИМ задаются в микросекундах;
%   частоты задаются в герцах;
%   интервалы ожидания задаются в секундах
%
% Допущения:
%   Конфигурация ориентирована на квадрокоптерную схему `quad-X`,
%   земную систему координат `NED`, связанную систему координат `FRD`
%   и текущие этапы подготовки внешней среды `ArduPilot SITL`
%   без подтверждения устойчивого автоматического полета.

cfg = struct();

cfg.udp_local_ip = "127.0.0.1";
cfg.udp_local_port = 9002;
cfg.udp_remote_ip = "127.0.0.1";
cfg.udp_remote_port = 9003;
cfg.mavlink_udp_ip = "127.0.0.1";
cfg.mavlink_udp_port = 14550;
cfg.mavlink_monitor_udp_port = 14552;
cfg.udp_timeout_s = 0.01;
cfg.udp_receive_pause_s = 0.02;
cfg.udp_max_rx_bytes = 4096;

cfg.frame_type = "NED-FRD-QUADX";
cfg.motor_count = 4;
cfg.motor_order = [1; 2; 3; 4];

cfg.pwm_min_us = 1000.0;
cfg.pwm_max_us = 2000.0;
cfg.pwm_hover_us = 1615.0;
cfg.loopback_yaw_step_time_s = 0.5;
cfg.loopback_yaw_delta_us = 60.0;

cfg.update_rate_hz = 100.0;
cfg.use_ardupilot_json = true;

cfg.sitl_magic_16 = 18458;
cfg.sitl_magic_32 = 29569;
cfg.sitl_channel_count_16 = 16;
cfg.sitl_channel_count_32 = 32;
cfg.expected_servo_channels = 16;

cfg.sitl_vehicle = "ArduCopter";
cfg.sitl_frame = "quad";
cfg.ardupilot_vehicle = cfg.sitl_vehicle;
cfg.sitl_enable_console = true;
cfg.sitl_enable_map = true;

cfg.ardupilot_root = "~/src/ardupilot";
cfg.wsl_distro_name = "Ubuntu";

cfg.sitl_start_timeout_s = 20.0;
cfg.udp_handshake_timeout_s = 2.0;
cfg.handshake_wait_timeout_s = cfg.udp_handshake_timeout_s;
cfg.handshake_max_steps = 20;

cfg.tools_windows_dir = "tools/ardupilot/windows";
cfg.tools_wsl_dir = "tools/ardupilot/wsl";
cfg.matlab_wait_for_packet_script = "scripts/run_ardupilot_wait_for_packet.m";
cfg.matlab_handshake_script = "scripts/run_ardupilot_json_udp_handshake.m";

cfg.notes = [ ...
    "TASK-13 автоматизирует подготовку внешней среды ArduPilot SITL для Windows и WSL."; ...
    "Земная система координат задается как NED, связанная система координат - FRD."; ...
    "Длительности импульсов ШИМ преобразуются в команды частоты вращения винтов линейным законом текущего этапа."; ...
    "Сценарии PowerShell и Bash не включают исходные тексты ArduPilot в репозиторий universal-mr-uav."; ...
    "Автоматическое выполнение действий, требующих прав администратора, допускается только при явном параметре Execute."; ...
    "Текущий этап не подтверждает устойчивый автоматический полет и не является летной валидацией."];
end
