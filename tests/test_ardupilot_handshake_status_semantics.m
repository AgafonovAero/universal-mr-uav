function tests = test_ardupilot_handshake_status_semantics
%TEST_ARDUPILOT_HANDSHAKE_STATUS_SEMANTICS Проверки семантики статусов обмена.
% Назначение:
%   Подтверждает различие между исходящей пробной передачей и ответной
%   передачей, а также отсутствие искусственного пакета в сценарии
%   реального обмена при отсутствии `ArduPilot`.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testRealHandshakeScenarioDoesNotClaimConfirmedExchangeWithoutPacket(testCase)
%TESTREALHANDSHAKESCENARIODOESNOTCLAIMCONFIRMEDEXCHANGEWITHOUTPACKET Проверить статус.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
[cfg.udp_local_port, cfg.udp_remote_port] = local_pick_test_ports();
cfg.udp_handshake_timeout_s = 0.2;

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = 0.2;
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_udp(case_cfg);
diag_hist = log.exchange_diag;
status_values = arrayfun(@(item) string(item.status), diag_hist);

verifyFalse(testCase, any([diag_hist.handshake_confirmed]));
verifyFalse(testCase, any([log.sitl_output.valid]));
verifyTrue(testCase, any(status_values == "исходящая пробная передача") ...
    || any(status_values == "прием не подтвержден"));
verifyFalse(testCase, any(status_values == "ответная передача"));
end

function testReplyStatusAppearsOnlyAfterReceivedPacket(testCase)
%TESTREPLYSTATUSAPPEARSONLYAFTERRECEIVEDPACKET Проверить ответную передачу.

cfg = uav.ardupilot.default_json_config();
[cfg.udp_local_port, cfg.udp_remote_port] = local_pick_test_ports();
cfg.udp_timeout_s = 0.2;

transport = uav.ardupilot.json_udp_open(cfg);
cleaner = onCleanup(@() uav.ardupilot.json_udp_close(transport)); %#ok<NASGU>
assumeTrue(testCase, transport.is_open);
assumeTrue(testCase, uav.ardupilot.json_udp_is_available().is_available);

aux_port = udpport( ...
    "byte", ...
    "IPV4", ...
    "LocalPort", double(cfg.udp_remote_port), ...
    "Timeout", 0.2);
cleanup_aux = onCleanup(@() clear('aux_port')); %#ok<NASGU>

raw_packet = local_make_packet(cfg.sitl_magic_16, 100, 5, uint16([ ...
    1600 1601 1602 1603 zeros(1, 12)]));

write( ...
    aux_port, ...
    reshape(raw_packet, 1, []), ...
    "uint8", ...
    char(cfg.udp_local_ip), ...
    double(cfg.udp_local_port));
pause(0.05);

json_text = "{""timestamp"":0}" + newline;
[transport, rx_out, diag] = uav.ardupilot.json_udp_step(transport, json_text, cfg);

verifyTrue(testCase, rx_out.valid);
verifyEqual(testCase, string(diag.status), "ответная передача");
verifyTrue(testCase, diag.handshake_confirmed);
verifyEqual(testCase, string(diag.tx_kind), "reply");
end

function [local_port, remote_port] = local_pick_test_ports()
%LOCAL_PICK_TEST_PORTS Выбрать пару портов для модульной проверки.

timestamp_ms = posixtime(datetime("now")) * 1000.0;
base_port = 35000 + mod(floor(timestamp_ms), 10000);

local_port = double(base_port);
remote_port = double(base_port + 1);
end

function raw_bytes = local_make_packet(magic, frame_rate_hz, frame_count, pwm_us)
%LOCAL_MAKE_PACKET Построить искусственный пакет ArduPilot SITL.

raw_bytes = [ ...
    typecast(uint16(magic), 'uint8'), ...
    typecast(uint16(frame_rate_hz), 'uint8'), ...
    typecast(uint32(frame_count), 'uint8'), ...
    typecast(uint16(pwm_us(:).'), 'uint8')];
raw_bytes = uint8(raw_bytes(:));
end
