function tests = test_ardupilot_default_json_config
%TEST_ARDUPILOT_DEFAULT_JSON_CONFIG Tests for the JSON SITL config scaffold.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testDefaultConfigContainsRequiredFields(testCase)
cfg = uav.ardupilot.default_json_config();

required_fields = { ...
    'udp_local_ip', 'udp_local_port', 'udp_remote_ip', 'udp_remote_port', ...
    'frame_type', 'motor_count', 'pwm_min_us', 'pwm_max_us', ...
    'pwm_hover_us', 'motor_order', 'update_rate_hz', ...
    'use_ardupilot_json', 'notes'};

for k = 1:numel(required_fields)
    verifyTrue(testCase, isfield(cfg, required_fields{k}));
end

verifyEqual(testCase, cfg.motor_count, 4);
verifyEqual(testCase, numel(cfg.motor_order), cfg.motor_count);
verifyGreaterThan(testCase, cfg.pwm_max_us, cfg.pwm_min_us);
verifyTrue(testCase, logical(cfg.use_ardupilot_json));
end
