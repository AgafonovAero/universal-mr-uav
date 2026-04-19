function tests = test_ardupilot_wait_for_sitl_output_packet
%TEST_ARDUPILOT_WAIT_FOR_SITL_OUTPUT_PACKET Проверки ожидания первого пакета.
% Назначение:
%   Подтверждает, что сценарий ожидания первого пакета завершается по
%   тайм-ауту без аварии, если реальный `ArduPilot` не запущен.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testWaitReturnsTimeoutWithoutArduPilot(testCase)
%TESTWAITRETURNSTIMEOUTWITHOUTARDUPILOT Проверить корректное завершение.

cfg = uav.ardupilot.default_json_config();
[cfg.udp_local_port, cfg.udp_remote_port] = local_pick_test_ports();
cfg.udp_handshake_timeout_s = 0.1;
cfg.udp_receive_pause_s = 0.01;

result = uav.ardupilot.wait_for_sitl_output_packet(cfg);

verifyTrue(testCase, isstruct(result));
verifyTrue(testCase, isfield(result, 'received'));
verifyTrue(testCase, isfield(result, 'sitl_output'));
verifyFalse(testCase, result.received);
verifyGreaterThanOrEqual(testCase, result.elapsed_s, 0.0);
verifyGreaterThanOrEqual(testCase, result.timeout_s, 0.1 - 1.0e-12);
verifyTrue(testCase, isstring(string(result.message)));
end

function [local_port, remote_port] = local_pick_test_ports()
%LOCAL_PICK_TEST_PORTS Выбрать пару портов для модульной проверки.

timestamp_ms = posixtime(datetime("now")) * 1000.0;
base_port = 30000 + mod(floor(timestamp_ms), 10000);

local_port = double(base_port);
remote_port = double(base_port + 1);
end
