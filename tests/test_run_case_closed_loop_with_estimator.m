function tests = test_run_case_closed_loop_with_estimator
%TEST_RUN_CASE_CLOSED_LOOP_WITH_ESTIMATOR Tests for estimator-driven closed loop.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function testClosedLoopRunnerReturnsDiagnosticsAndBoundedQuatNorms(testCase)
params = uav.sim.make_deterministic_demo_params();
case_cfg = local_takeoff_case_cfg(params);

log = uav.sim.run_case_closed_loop_with_estimator(case_cfg);
accel_weight_hist = arrayfun(@(d) d.attitude.accel_correction_weight, ...
    log.estimator_diag);

verifyGreaterThan(testCase, numel(log.time_s), 0);
verifyTrue(testCase, isfield(log, 'reference'));
verifyTrue(testCase, isfield(log, 'controller_diag'));
verifyTrue(testCase, isfield(log, 'controller_state'));
verifyEqual(testCase, numel(log.reference), numel(log.time_s));
verifyEqual(testCase, numel(log.controller_diag), numel(log.time_s));
verifyEqual(testCase, numel(log.controller_state), numel(log.time_s));
verifyLessThan(testCase, max(abs(log.quat_norm_true - 1.0)), 1.0e-10);
verifyLessThan(testCase, max(abs(log.quat_norm_est - 1.0)), 1.0e-10);
verifyGreaterThan(testCase, log.estimator(end).alt_m, 3.0);
verifyGreaterThan(testCase, max(accel_weight_hist), 0.9);
end

function testDemoControllersUseEstimatorSensorInputsOnly(testCase)
params = uav.sim.make_deterministic_demo_params();
sens = local_level_sensor_sample(params);
est = uav.est.estimator_init(params, sens);
ctrl_input = struct( ...
    'time_s', 0.0, ...
    'sensors', sens, ...
    'estimator', est, ...
    'reference', struct( ...
        'altitude_ref_m', 2.0, ...
        'vertical_speed_ref_mps', 0.0, ...
        'pitch_ref_rad', deg2rad(-5.0)));

[motor_cmd_takeoff, ctrl_state_takeoff] = uav.ctrl.demo_takeoff_hold_controller( ...
    ctrl_input, struct(), params.demo.dt_s, params, local_controller_cfg(params));
[motor_cmd_pitch, ctrl_state_pitch] = uav.ctrl.demo_pitch_hold_controller( ...
    ctrl_input, struct(), params.demo.dt_s, params, local_controller_cfg(params));

verifySize(testCase, motor_cmd_takeoff, [4, 1]);
verifySize(testCase, motor_cmd_pitch, [4, 1]);
verifyTrue(testCase, isstruct(ctrl_state_takeoff));
verifyTrue(testCase, isstruct(ctrl_state_pitch));
end

function case_cfg = local_takeoff_case_cfg(params)
%LOCAL_TAKEOFF_CASE_CFG Build a compact closed-loop takeoff case.

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = 8.0;
case_cfg.reference_fun = @(t_s, ~, ~, ~) local_reference_at_time(t_s);
case_cfg.controller_fun = @(ctrl_input, ctrl_state, dt_s, params_local) ...
    uav.ctrl.demo_takeoff_hold_controller( ...
        ctrl_input, ctrl_state, dt_s, params_local, local_controller_cfg(params_local));
end

function ref = local_reference_at_time(t_s)
%LOCAL_REFERENCE_AT_TIME Return one compact takeoff reference.

target_altitude_m = 5.0;
climb_rate_mps = 1.5;
ramp_duration_s = target_altitude_m / climb_rate_mps;

if t_s < ramp_duration_s
    altitude_ref_m = climb_rate_mps * t_s;
    vertical_speed_ref_mps = climb_rate_mps;
else
    altitude_ref_m = target_altitude_m;
    vertical_speed_ref_mps = 0.0;
end

ref = struct();
ref.altitude_ref_m = altitude_ref_m;
ref.vertical_speed_ref_mps = vertical_speed_ref_mps;
ref.pitch_ref_rad = 0.0;
end

function cfg = local_controller_cfg(params)
%LOCAL_CONTROLLER_CFG Return one deterministic demo-controller config.

cfg = struct( ...
    'altitude_kp_per_s2', 0.55, ...
    'vertical_speed_kp_per_s', 1.35, ...
    'up_accel_limit_mps2', 3.0, ...
    'angle_kp_radps_per_rad', [3.0; 3.2; 1.2], ...
    'body_rate_cmd_limit_radps', [0.7; 0.7; 0.5], ...
    'rate_pid_gains', struct( ...
        'Kp', [0.12; 0.12; 0.08], ...
        'Ki', [0.04; 0.04; 0.02], ...
        'Kd', [0.003; 0.003; 0.001]), ...
    'body_moment_limit_Nm', [0.05; 0.05; 0.03], ...
    'total_thrust_min_N', 0.40 * params.mass_kg * params.gravity_mps2, ...
    'total_thrust_max_N', 1.80 * params.mass_kg * params.gravity_mps2, ...
    'min_body_z_up_gain', 0.35);
end

function sens = local_level_sensor_sample(params)
%LOCAL_LEVEL_SENSOR_SAMPLE Build one deterministic level sensor sample.

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', [0.0; 0.0; -params.gravity_mps2], ...
    'gyro_b_radps', zeros(3, 1));
sens.baro = struct( ...
    'alt_m', 0.0, ...
    'pressure_pa', uav.env.isa_pressure_pa(0.0));
sens.mag = struct( ...
    'field_b_uT', params.env.mag_ned_uT(:));
sens.gnss = struct( ...
    'pos_ned_m', zeros(3, 1), ...
    'vel_ned_mps', zeros(3, 1));
end
