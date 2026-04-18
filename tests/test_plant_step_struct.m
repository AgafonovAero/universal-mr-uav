function tests = test_plant_step_struct
%TEST_PLANT_STEP_STRUCT Tests for the struct-based plant-step wrapper.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testPlantStepStructMatchesPackedPlantStep(testCase)
params = uav.sim.default_params_quad_x250();
state_prev = uav.core.state_unpack(params.demo.initial_state_plant);
motor_cmd_radps = repmat(params.hover_omega_radps, 4, 1);

[state_next, diag] = uav.sim.plant_step_struct( ...
    state_prev, motor_cmd_radps, params.demo.dt_s, params);
[x_next_ref, diag_ref] = uav.sim.plant_step( ...
    params.demo.initial_state_plant, motor_cmd_radps, params.demo.dt_s, params);

verifyEqual(testCase, uav.core.state_pack(state_next), x_next_ref, 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.omega_m_radps, diag_ref.omega_m_radps, 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.forces_b_N, diag_ref.forces_b_N, 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.moments_b_Nm, diag_ref.moments_b_Nm, 'AbsTol', 1.0e-12);
verifyEqual(testCase, diag.quat_norm, diag_ref.quat_norm, 'AbsTol', 1.0e-12);
end
