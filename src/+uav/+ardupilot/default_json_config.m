function cfg = default_json_config()
%DEFAULT_JSON_CONFIG Вернуть типовую настройку средства сопряжения.
% Назначение:
%   Формирует явную структуру конфигурации для средства сопряжения
%   `universal-mr-uav` с `ArduPilot` по `JSON` и `UDP`.
%   В конфигурации задаются адреса обмена, диапазоны длительности
%   импульсов ШИМ, ожидаемые форматы выходного двоичного пакета и
%   служебные параметры транспортного уровня текущего этапа.
%
% Входы:
%   none
%
% Выходы:
%   cfg - скалярная структура конфигурации средства сопряжения
%
% Единицы измерения:
%   порты задаются целыми числами, длительность импульсов ШИМ - в
%   микросекундах, частота обновления - в герцах, время ожидания - в
%   секундах
%
% Допущения:
%   Конфигурация ориентирована на квадрокоптерную схему `quad-X`,
%   земную систему координат `NED`, связанную систему координат `FRD`
%   и первый воспроизводимый этап обмена данными с `ArduPilot`
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

cfg.handshake_wait_timeout_s = 1.0;
cfg.handshake_max_steps = 20;

cfg.ardupilot_vehicle = "ArduCopter";
cfg.ardupilot_root = "";

cfg.notes = [ ...
    "TASK-11 реализует первый воспроизводимый уровень обмена данными по JSON и UDP."; ...
    "Земная система координат задается как NED, связанная система координат - FRD."; ...
    "Длительности импульсов ШИМ преобразуются в команды частоты вращения винтов линейным законом текущего этапа."; ...
    "Текущий этап не подтверждает устойчивый автоматический полет и не является летной валидацией."]; 
end
