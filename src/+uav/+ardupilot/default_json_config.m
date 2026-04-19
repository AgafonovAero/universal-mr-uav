function cfg = default_json_config()
%DEFAULT_JSON_CONFIG Return the default ArduPilot JSON SITL scaffold config.
% Description:
%   Creates one explicit configuration struct for the future ArduPilot
%   JSON SITL adapter boundary. The config fixes UDP endpoint placeholders,
%   frame conventions, PWM semantics, and the internal motor order without
%   starting any real network traffic.
%
% Inputs:
%   none
%
% Outputs:
%   cfg - scalar configuration struct for the JSON SITL scaffold
%
% Units:
%   SI only, ports in integer counts, PWM in microseconds
%
% Assumptions:
%   This scaffold targets quad-X loopback verification only and still
%   requires a later alignment pass against a real ArduPilot SITL setup.

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
    "TASK-10 builds only a code-centric scaffold without real UDP traffic."; ...
    "Earth frame is NED, body frame is FRD/X-forward Y-right Z-down."; ...
    "Real ArduPilot alignment of motor order, PWM semantics, and modes is a separate next step."];

cfg.ardupilot_root = "";
cfg.loopback_yaw_step_time_s = 2.0;
cfg.loopback_yaw_delta_us = 60.0;
end
