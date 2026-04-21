function ctx = json_lockstep_open(cfg)
%JSON_LOCKSTEP_OPEN Открыть официальный lockstep-транспорт ArduPilot JSON.
% Назначение:
%   Подготавливает UDP-приемник MATLAB backend по официальной схеме
%   ArduPilot JSON: MATLAB слушает порт 9002, принимает бинарные кадры SITL
%   и затем отвечает JSON только на адрес и порт последнего валидного
%   пакета.
%
% Входы:
%   cfg - структура конфигурации ArduPilot JSON
%
% Выходы:
%   ctx - структура состояния lockstep-обмена
%
% Единицы измерения:
%   порты - целые числа
%
% Допущения:
%   Низкоуровневое открытие UDP выполняется существующей функцией
%   uav.ardupilot.json_udp_open.

if nargin < 1 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

transport = uav.ardupilot.json_udp_open(cfg);

ctx = struct();
ctx.cfg = cfg;
ctx.transport = transport;
ctx.is_open = logical(transport.is_open);
ctx.received_datagram_count = 0;
ctx.valid_rx_count = 0;
ctx.duplicate_frame_count = 0;
ctx.missed_frame_count = 0;
ctx.invalid_rx_count = 0;
ctx.json_response_tx_count = 0;
ctx.controller_reset_count = 0;
ctx.last_magic = 0;
ctx.last_frame_rate_hz = 0;
ctx.last_frame_count = -1;
ctx.last_sender_address = "";
ctx.last_sender_port = 0;
ctx.last_motor_pwm_us = nan(double(cfg.motor_count), 1);
ctx.last_motor_cmd_radps = nan(double(cfg.motor_count), 1);
ctx.last_status = string(transport.message);
ctx.last_error = "";
ctx.last_valid_frame_seen = false;
ctx.last_reset_required = false;
end
