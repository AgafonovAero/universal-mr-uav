function tests = test_ardupilot_pwm_to_motor_radps
%TEST_ARDUPILOT_PWM_TO_MOTOR_RADPS Проверки преобразования ШИМ.
% Назначение:
%   Подтверждает корректность линейного преобразования длительности
%   импульсов ШИМ в команды по частоте вращения винтов с учетом
%   насыщения на границах диапазона.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить исходный код проекта.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testLinearMappingAndSaturation(testCase)
%TESTLINEARMAPPINGANDSATURATION Проверить линейность и насыщение.

params = uav.sim.default_params_quad_x250();
cfg = uav.ardupilot.default_json_config();

pwm_us = [ ...
    1000.0; ...
    1500.0; ...
    2000.0; ...
    2200.0];

motor_cmd_radps = uav.ardupilot.pwm_to_motor_radps( ...
    pwm_us, ...
    params, ...
    cfg);

verifyEqual(testCase, motor_cmd_radps(1), params.motor.omega_min_radps, ...
    'AbsTol', 1.0e-12);
verifyEqual(testCase, motor_cmd_radps(2), ...
    0.5 * (params.motor.omega_min_radps + params.motor.omega_max_radps), ...
    'AbsTol', 1.0e-12);
verifyEqual(testCase, motor_cmd_radps(3), params.motor.omega_max_radps, ...
    'AbsTol', 1.0e-12);
verifyEqual(testCase, motor_cmd_radps(4), params.motor.omega_max_radps, ...
    'AbsTol', 1.0e-12);
end
