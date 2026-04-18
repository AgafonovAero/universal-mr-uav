function tests = test_motor_esc_step
%TEST_MOTOR_ESC_STEP Basic tests for the Stage-1.5 motor model.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testMotorEscStepConvergesMonotonically(testCase)
params = uav.sim.default_params_quad_x250();
omega_prev_radps = zeros(4, 1);
omega_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
last_omega_radps = omega_prev_radps;

for k = 1:40
    [omega_next_radps, ~] = uav.vmg.motor_esc_step( ...
        omega_prev_radps, omega_cmd_radps, params.demo.dt_s, params.motor);
    verifyGreaterThanOrEqual(testCase, omega_next_radps, omega_prev_radps);
    verifyLessThanOrEqual(testCase, omega_next_radps, omega_cmd_radps);
    omega_prev_radps = omega_next_radps;
    last_omega_radps = omega_next_radps;
end

verifyEqual(testCase, last_omega_radps, omega_cmd_radps, 'RelTol', 5.0e-4);
end

function testMotorEscStepSaturatesAtMaximum(testCase)
params = uav.sim.default_params_quad_x250();
omega_cmd_radps = repmat(params.motor.omega_max_radps * 2.0, 4, 1);

[omega_next_radps, motor_diag] = uav.vmg.motor_esc_step( ...
    zeros(4, 1), omega_cmd_radps, 10.0 * params.motor.tau_s, params.motor);

verifyEqual(testCase, motor_diag.omega_cmd_sat_radps, ...
    repmat(params.motor.omega_max_radps, 4, 1), 'AbsTol', 1.0e-12);
verifyEqual(testCase, omega_next_radps, ...
    repmat(params.motor.omega_max_radps, 4, 1), 'AbsTol', 1.0e-12);
end
