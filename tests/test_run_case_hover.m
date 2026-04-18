function tests = test_run_case_hover
%TEST_RUN_CASE_HOVER Tests for the hover case on the Stage-1.5 runner.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testHoverCaseKeepsQuatNormAndNearZeroVerticalAccel(testCase)
params = uav.sim.default_params_quad_x250();
case_cfg = local_hover_case_cfg(params);

log = uav.sim.run_case(case_cfg);
final_state = log.state(end);
final_az_mps2 = local_vertical_accel( ...
    final_state, log.forces_b_N(end, :).', log.moments_b_Nm(end, :).', params);

verifyEqual(testCase, log.quat_norm(end), 1.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, log.omega_m_radps(end, :).', repmat(params.hover_omega_radps, 4, 1), ...
    'RelTol', 5.0e-4);
verifyEqual(testCase, final_az_mps2, 0.0, 'AbsTol', 1.0e-2);
end

function case_cfg = local_hover_case_cfg(params)
%LOCAL_HOVER_CASE_CFG Build the hover runner configuration for testing.

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = params.demo.case_hover_t_final_s;
case_cfg.command_fun = @local_hover_command;
end

function motor_cmd_radps = local_hover_command(~, ~, params)
%LOCAL_HOVER_COMMAND Return the constant hover command.

motor_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
end

function az_mps2 = local_vertical_accel(state, forces_b_N, moments_b_Nm, params)
%LOCAL_VERTICAL_ACCEL Estimate body-axis vertical acceleration from a state.

x_packed = uav.core.state_pack(state);
inputs = struct('forces_b_N', forces_b_N, 'moments_b_Nm', moments_b_Nm);
dx_rigid = uav.core.eom6dof_quat(0.0, x_packed(1:13), params, inputs);
az_mps2 = dx_rigid(6);
end
