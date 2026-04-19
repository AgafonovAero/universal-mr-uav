function cfg = default_json_config()
%DEFAULT_JSON_CONFIG Вернуть типовую настройку средства сопряжения ArduPilot.
% Description:
%   Формирует явную структуру конфигурации для последующего сопряжения с
%   `ArduPilot` через JSON-интерфейс. Конфигурация фиксирует адреса
%   обмена, соглашения по системам координат, семантику ШИМ и внутренний
%   порядок двигателей без запуска реального сетевого обмена.
%
% Inputs:
%   none
%
% Outputs:
%   cfg - scalar configuration struct for the current integration stage
%
% Units:
%   SI only, ports in integer counts, PWM in microseconds
%
% Assumptions:
%   Конфигурация предназначена для квадрокоптерной схемы `quad-X` и
%   проверочных замкнутых прогонов. Полное согласование с реальным
%   `ArduPilot SITL` относится к следующему этапу работ.

cfg = struct();
cfg.udp_local_ip = "127.0.0.1";
cfg.udp_local_port = 9002;
cfg.udp_remote_ip = "127.0.0.1";
cfg.udp_remote_port = 9003;
cfg.frame_type = "NED-FRD-QUADX";
cfg.motor_count = 4;
cfg.pwm_min_us = 1000.0;
cfg.pwm_max_us = 2000.0;
cfg.pwm_hover_us = 1615.0;
cfg.motor_order = (1:4).';
cfg.update_rate_hz = 100.0;
cfg.use_ardupilot_json = true;
cfg.notes = [ ...
    "TASK-10 подготавливает только программную заготовку средства сопряжения без реального UDP-обмена."; ...
    "Земная система координат задается как NED, связанная система координат - FRD/X-forward Y-right Z-down."; ...
    "Полное согласование порядка двигателей, семантики ШИМ и режимов с ArduPilot выполняется на следующем этапе."];

cfg.ardupilot_root = "";
cfg.loopback_yaw_step_time_s = 2.0;
cfg.loopback_yaw_delta_us = 60.0;
end
