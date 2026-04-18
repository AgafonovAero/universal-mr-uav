function tests = test_run_case_yaw_step
%TEST_RUN_CASE_YAW_STEP Tests for the yaw-step case on the Stage-1.5 runner.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testYawStepProducesPositiveYawRate(testCase)
params = uav.sim.default_params_quad_x250();
case_cfg = local_yaw_case_cfg(params);

log = uav.sim.run_case(case_cfg);
final_state = log.state(end);

verifyEqual(testCase, log.quat_norm(end), 1.0, 'AbsTol', 1.0e-12);
verifyGreaterThan(testCase, final_state.w_b_radps(3), 0.0);
end

function case_cfg = local_yaw_case_cfg(params)
%LOCAL_YAW_CASE_CFG Build the yaw-step runner configuration for testing.

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = params.demo.case_yaw_step_t_final_s;
case_cfg.command_fun = @local_yaw_step_command;
end

function motor_cmd_radps = local_yaw_step_command(t_s, ~, params)
%LOCAL_YAW_STEP_COMMAND Return the hover-level yaw-step motor command.

total_thrust_N = params.mass_kg * params.gravity_mps2;
body_moments_Nm = [0.0; 0.0; local_yaw_moment(t_s, params)];
[motor_cmd_radps, ~] = uav.vmg.mixer_quad_x(total_thrust_N, body_moments_Nm, params);
end

function yaw_moment_Nm = local_yaw_moment(t_s, params)
%LOCAL_YAW_MOMENT Return the positive yaw-step command.

yaw_moment_Nm = 0.0;
if t_s >= params.demo.yaw_step_time_s
    yaw_moment_Nm = params.demo.yaw_step_moment_Nm;
end
end
