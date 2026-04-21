function tests = test_ardupilot_lockstep_frame_count
%TEST_ARDUPILOT_LOCKSTEP_FRAME_COUNT Проверки frame_count в lockstep-обмене.
% Назначение:
%   Подтверждает подсчет дублей, пропусков и сбросов frame_count для нового
%   lockstep-транспорта TASK-25.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testLockstepCountsDuplicatesMissesAndReset(testCase)
cfg = uav.ardupilot.default_json_config();
cfg.udp_local_port = 9102;
cfg.udp_remote_ip = "127.0.0.1";
cfg.udp_remote_port = 9103;

ctx = uav.ardupilot.json_lockstep_open(cfg);
sender = udpport("datagram", "IPV4");
cleanupObj = onCleanup(@() local_cleanup_handles(ctx, sender)); %#ok<NASGU>

write(sender, local_make_packet(cfg.sitl_magic_16, 400, 1, uint16(1000:1015)), ...
    "uint8", "127.0.0.1", cfg.udp_local_port);
pause(0.05);
    [ctx, frame1] = uav.ardupilot.json_lockstep_read_frame(ctx);
verifyTrue(testCase, frame1.valid);
verifyEqual(testCase, ctx.valid_rx_count, 1);
verifyEqual(testCase, ctx.duplicate_frame_count, 0);
verifyEqual(testCase, ctx.missed_frame_count, 0);
verifyEqual(testCase, ctx.controller_reset_count, 0);

write(sender, local_make_packet(cfg.sitl_magic_16, 400, 1, uint16(1000:1015)), ...
    "uint8", "127.0.0.1", cfg.udp_local_port);
pause(0.05);
[ctx, frame2] = uav.ardupilot.json_lockstep_read_frame(ctx);
verifyFalse(testCase, frame2.valid);
verifyEqual(testCase, ctx.valid_rx_count, 1);
verifyEqual(testCase, ctx.duplicate_frame_count, 1);

write(sender, local_make_packet(cfg.sitl_magic_16, 400, 4, uint16(1000:1015)), ...
    "uint8", "127.0.0.1", cfg.udp_local_port);
pause(0.05);
[ctx, frame3] = uav.ardupilot.json_lockstep_read_frame(ctx);
verifyTrue(testCase, frame3.valid);
verifyEqual(testCase, frame3.missed_count, 2);
verifyEqual(testCase, ctx.valid_rx_count, 2);
verifyEqual(testCase, ctx.missed_frame_count, 2);

write(sender, local_make_packet(cfg.sitl_magic_16, 400, 1, uint16(1000:1015)), ...
    "uint8", "127.0.0.1", cfg.udp_local_port);
pause(0.05);
[ctx, frame4] = uav.ardupilot.json_lockstep_read_frame(ctx);
verifyTrue(testCase, frame4.valid);
verifyTrue(testCase, frame4.controller_reset);
verifyEqual(testCase, ctx.controller_reset_count, 1);
verifyEqual(testCase, ctx.last_frame_count, 1);
end

function local_cleanup_handles(ctx, sender)
if ~isempty(sender)
    clear sender;
end
if ~isempty(ctx)
    uav.ardupilot.json_lockstep_close(ctx);
end
end

function raw_bytes = local_make_packet(magic, frame_rate_hz, frame_count, pwm_us)
raw_bytes = [ ...
    typecast(uint16(magic), 'uint8'), ...
    typecast(uint16(frame_rate_hz), 'uint8'), ...
    typecast(uint32(frame_count), 'uint8'), ...
    typecast(uint16(pwm_us(:).'), 'uint8')];
raw_bytes = uint8(raw_bytes(:));
end
