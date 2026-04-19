function cfg = default_json_config()
%DEFAULT_JSON_CONFIG Вернуть типовую настройку средства сопряжения.
% Назначение:
%   Формирует явную структуру конфигурации для средства сопряжения
%   `universal-mr-uav` с `ArduPilot` по `JSON` и `UDP`.
%   Конфигурация задает адреса обмена, параметры проверки среды,
%   параметры запуска `ArduPilot SITL`, ограничения по длительности
%   импульсов ШИМ и ожидаемые форматы входного двоичного пакета.
%
% Входы:
%   none
%
% Выходы:
%   cfg - скалярная структура конфигурации средства сопряжения
%
% Единицы измерения:
%   порты задаются целыми числами, длительность импульсов ШИМ - в
%   микросекундах, частоты - в герцах, интервалы ожидания - в секундах
%
% Допущения:
%   Конфигурация ориентирована на квадрокоптерную схему `quad-X`,
%   земную систему координат `NED`, связанную систему координат `FRD`
%   и этап проверки первого двустороннего обмена с `ArduPilot SITL`
%   без подтверждения устойчивого автоматического полета.

cfg = struct();

cfg.udp_local_ip = "127.0.0.1";
cfg.udp_local_port = 9002;
cfg.udp_remote_ip = "127.0.0.1";
cfg.udp_remote_port = 9003;
cfg.udp_timeout_s = 0.01;
cfg.udp_receive_pause_s = 0.02;
cfg.udp_max_rx_bytes = 4096;

cfg.frame_type = "NED-FRD-QUADX";
cfg.motor_count = 4;
cfg.motor_order = [ ...
    1; ...
    2; ...
    3; ...
    4];

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

cfg.ardupilot_root = "";
cfg.wsl_distro_name = "";

cfg.sitl_start_timeout_s = 20.0;
cfg.udp_handshake_timeout_s = 2.0;
cfg.handshake_wait_timeout_s = cfg.udp_handshake_timeout_s;
cfg.handshake_max_steps = 20;

cfg.notes = [ ...
    "TASK-12 подготавливает рабочую среду ArduPilot SITL и проверку первого двустороннего обмена по JSON и UDP."; ...
    "Земная система координат задается как NED, связанная система координат - FRD."; ...
    "Длительности импульсов ШИМ преобразуются в команды частоты вращения винтов линейным законом текущего этапа."; ...
    "Текущий этап не подтверждает устойчивый автоматический полет и не является летной валидацией."; ...
    "При отсутствии входного двоичного пакета допустима только исходящая пробная передача строки JSON."]; 
end
