function tests = test_ardupilot_default_json_config
%TEST_ARDUPILOT_DEFAULT_JSON_CONFIG Проверки типовой конфигурации TASK-10.
% Назначение:
%   Подтверждает, что функция `uav.ardupilot.default_json_config`
%   возвращает структуру с обязательными полями и согласованными
%   ограничениями диапазона ШИМ.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить исходный код проекта.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testDefaultConfigContainsRequiredFields(testCase)
%TESTDEFAULTCONFIGCONTAINSREQUIREDFIELDS Проверить состав полей.

cfg = uav.ardupilot.default_json_config();

required_fields = { ...
    'udp_local_ip', ...
    'udp_local_port', ...
    'udp_remote_ip', ...
    'udp_remote_port', ...
    'frame_type', ...
    'motor_count', ...
    'pwm_min_us', ...
    'pwm_max_us', ...
    'pwm_hover_us', ...
    'motor_order', ...
    'update_rate_hz', ...
    'use_ardupilot_json', ...
    'notes'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    verifyTrue(testCase, isfield(cfg, field_name));
end

verifyEqual(testCase, cfg.motor_count, 4);
verifyEqual(testCase, numel(cfg.motor_order), cfg.motor_count);
verifyGreaterThan(testCase, cfg.pwm_max_us, cfg.pwm_min_us);
verifyTrue(testCase, logical(cfg.use_ardupilot_json));
end
