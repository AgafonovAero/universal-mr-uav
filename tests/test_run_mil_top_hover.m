function tests = test_run_mil_top_hover
%TEST_RUN_MIL_TOP_HOVER Smoke and consistency tests for the MIL hover shell.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function teardown(~)
if bdIsLoaded('mil_top')
    close_system('mil_top', 0);
end
end

function testMilHoverRunMatchesCodeCentricKernel(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));

run(fullfile(repoRoot, 'scripts', 'run_mil_top_hover.m'));
demo = evalin('base', 'mil_top_hover_demo');

verifyGreaterThan(testCase, numel(demo.state_log), 0);
verifyEqual(testCase, numel(demo.state_log), numel(demo.time_s));
verifyTrue(testCase, isfield(demo.state_log(end), 'p_ned_m'));
verifyTrue(testCase, isfield(demo.sensors_log(end), 'imu'));
verifyTrue(testCase, isfield(demo.estimator_log(end), 'q_nb'));
verifyTrue(testCase, isfield(demo.diag_log(end), 'plant'));
verifyLessThan(testCase, abs(demo.final_true_quat_norm - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(demo.final_estimated_quat_norm - 1.0), 1.0e-9);

case_cfg = struct();
case_cfg.params = demo.params;
case_cfg.state0 = demo.state0;
case_cfg.dt_s = demo.dt_s;
case_cfg.t_final_s = demo.t_final_s;
case_cfg.command_fun = @local_hover_command;

ref_log = uav.sim.run_case_with_estimator(case_cfg);
ref_final_state = ref_log.state(end);
ref_final_est = ref_log.estimator(end);

verifyEqual(testCase, demo.state_log(end).p_ned_m, ref_final_state.p_ned_m, ...
    'AbsTol', 1.0e-10);
verifyEqual(testCase, demo.state_log(end).v_b_mps, ref_final_state.v_b_mps, ...
    'AbsTol', 1.0e-10);
verifyEqual(testCase, demo.state_log(end).q_nb, ref_final_state.q_nb, ...
    'AbsTol', 1.0e-10);
verifyEqual(testCase, demo.estimator_log(end).q_nb, ref_final_est.q_nb, ...
    'AbsTol', 1.0e-10);
verifyEqual(testCase, demo.estimator_log(end).alt_m, ref_final_est.alt_m, ...
    'AbsTol', 1.0e-10);
end

function motor_cmd_radps = local_hover_command(~, ~, params)
%LOCAL_HOVER_COMMAND Return the constant hover motor command.

motor_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
end
