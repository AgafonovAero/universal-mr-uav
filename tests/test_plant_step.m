function tests = test_plant_step
%TEST_PLANT_STEP Basic tests for the Stage-1.5 plant kernel step.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testTrimLikeHoverAccelerationIsNearZeroAfterSpool(testCase)
params = uav.sim.default_params_quad_x250();
x_state = params.demo.initial_state_plant;
motor_cmd_radps = repmat(params.hover_omega_radps, 4, 1);

for k = 1:200
    [x_state, step_diag] = uav.sim.plant_step(x_state, motor_cmd_radps, params.demo.dt_s, params);
end

verifyEqual(testCase, step_diag.omega_m_radps, motor_cmd_radps, 'RelTol', 5.0e-4);
verifyEqual(testCase, step_diag.dx_rigid(6), 0.0, 'AbsTol', 1.0e-2);
verifyEqual(testCase, step_diag.quat_norm, 1.0, 'AbsTol', 1.0e-12);
end
