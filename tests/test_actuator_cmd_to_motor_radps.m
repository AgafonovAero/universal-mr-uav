function tests = test_actuator_cmd_to_motor_radps
%TEST_ACTUATOR_CMD_TO_MOTOR_RADPS Tests for SIL actuator conversion.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testLinearNorm01Mapping(testCase)
params = uav.sim.default_params_quad_x250();
act = struct( ...
    'mode', "norm01", ...
    'motor_norm_01', [0.0; 0.25; 0.5; 1.0]);

omega = uav.sil.actuator_cmd_to_motor_radps(act, params);
omega_expected = params.motor.omega_min_radps + ...
    act.motor_norm_01 .* ...
    (params.motor.omega_max_radps - params.motor.omega_min_radps);

verifyEqual(testCase, omega, omega_expected, 'AbsTol', 1.0e-12);
end

function testInputSaturationHonorsMotorLimits(testCase)
params = uav.sim.default_params_quad_x250();
params.motor.omega_min_radps = 50.0;
params.motor.omega_max_radps = 450.0;

act = struct( ...
    'mode', "norm01", ...
    'motor_norm_01', [-0.2; 0.3; 1.2; 2.0]);

omega = uav.sil.actuator_cmd_to_motor_radps(act, params);
omega_expected = [50.0; 170.0; 450.0; 450.0];

verifyEqual(testCase, omega, omega_expected, 'AbsTol', 1.0e-12);
verifyGreaterThanOrEqual(testCase, omega, params.motor.omega_min_radps);
verifyLessThanOrEqual(testCase, omega, params.motor.omega_max_radps);
end
