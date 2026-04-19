function tests = test_ardupilot_decode_sitl_output_packet
%TEST_ARDUPILOT_DECODE_SITL_OUTPUT_PACKET Проверки разбора пакета SITL.
% Назначение:
%   Подтверждает корректный разбор двоичных пакетов `ArduPilot JSON SITL`
%   с 16 и 32 каналами и корректную обработку пустых или коротких пакетов.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testDecodePacketWith16Channels(testCase)
%TESTDECODEPACKETWITH16CHANNELS Проверить пакет с 16 каналами.

cfg = uav.ardupilot.default_json_config();
raw_bytes = local_make_packet(cfg.sitl_magic_16, 100, 17, uint16([ ...
    1600 1601 1602 1603 zeros(1, 12)]));

out = uav.ardupilot.decode_sitl_output_packet(raw_bytes, cfg);

verifyTrue(testCase, out.valid);
verifyEqual(testCase, out.magic, double(cfg.sitl_magic_16));
verifyEqual(testCase, out.frame_rate_hz, 100);
verifyEqual(testCase, out.frame_count, 17);
verifyEqual(testCase, out.motor_pwm_us, [1600; 1601; 1602; 1603], ...
    'AbsTol', 1.0e-12);
end

function testDecodePacketWith32Channels(testCase)
%TESTDECODEPACKETWITH32CHANNELS Проверить пакет с 32 каналами.

cfg = uav.ardupilot.default_json_config();
raw_bytes = local_make_packet(cfg.sitl_magic_32, 400, 33, uint16(1:32));

out = uav.ardupilot.decode_sitl_output_packet(raw_bytes, cfg);

verifyTrue(testCase, out.valid);
verifyEqual(testCase, out.magic, double(cfg.sitl_magic_32));
verifyEqual(testCase, out.frame_rate_hz, 400);
verifyEqual(testCase, out.frame_count, 33);
verifyEqual(testCase, out.motor_pwm_us, [1; 2; 3; 4], ...
    'AbsTol', 1.0e-12);
end

function testDecodeShortPacketReturnsInvalidWithoutThrow(testCase)
%TESTDECODESHORTPACKETRETURNSINVALIDWITHOUTTHROW Проверить короткий пакет.

cfg = uav.ardupilot.default_json_config();
out_empty = uav.ardupilot.decode_sitl_output_packet([], cfg);
out_short = uav.ardupilot.decode_sitl_output_packet(uint8([1 2 3]), cfg);

verifyFalse(testCase, out_empty.valid);
verifyFalse(testCase, out_short.valid);
verifyGreaterThan(testCase, strlength(string(out_empty.message)), 0);
verifyGreaterThan(testCase, strlength(string(out_short.message)), 0);
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
