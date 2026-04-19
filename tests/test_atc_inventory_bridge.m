function tests = test_atc_inventory_bridge
%TEST_ATC_INVENTORY_BRIDGE Tests for ATC inventory and bridge adapters.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function testAtcInventoryReportExistsAndMentionsDirectEntryPoint(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
inventoryFile = fullfile(repoRoot, 'artifacts', 'reports', ...
    'task_08a_atc_inventory_ru.md');

verifyEqual(testCase, exist(inventoryFile, 'file'), 2);

contents = fileread(inventoryFile);
verifyNotEmpty(testCase, strfind(contents, 'FSW_Simulink_wrapper_step'));
verifyNotEmpty(testCase, strfind(contents, 'FSW_make_default_in'));
verifyNotEmpty(testCase, strfind(contents, 'motor_cmd'));
verifyNotEmpty(testCase, strfind(contents, 'Прямой MATLAB-вызов'));
end

function testBridgePackAndUnpackAgreeWithDirectControllerCall(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
externalRepoRoot = fullfile(fileparts(repoRoot), 'atc_controller');

params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
cfg = uav.atc.default_atc_bridge_config(externalRepoRoot, params);
ctx = uav.atc.make_controller_context(cfg, params);

state = uav.core.state_unpack(params.demo.initial_state_plant);
plant_diag = local_snapshot_diag(state, params);
sensors = uav.sensors.sensors_step(state, plant_diag, params);
estimator0 = uav.est.estimator_init(params, sensors);
[estimator, ~] = uav.est.estimator_step(estimator0, sensors, 0.0, params);

controller_in = uav.atc.pack_sensor_packet_for_atc( ...
    sensors, estimator, params.demo.dt_s, 0.0, cfg, ctx.input_template);

verifyEqual(testCase, controller_in.pos_ned(3), single(-estimator.alt_m), ...
    'AbsTol', single(1.0e-6));
verifyEqual(testCase, controller_in.q_bn, single(reshape(estimator.q_nb, 1, 4)), ...
    'AbsTol', single(1.0e-6));

controller_out = [];
for k = 1:60
    controller_in.reset = (k == 1);
    controller_out = FSW_Simulink_wrapper_step(controller_in, ctx.P);
end

[actuator_cmd, meta] = uav.atc.unpack_atc_actuation(controller_out, ctx);
motor_cmd_radps = uav.sil.actuator_cmd_to_motor_radps(actuator_cmd, params);

verifyEqual(testCase, actuator_cmd.mode, "norm01");
verifySize(testCase, actuator_cmd.motor_norm_01, [4 1]);
verifyGreaterThanOrEqual(testCase, actuator_cmd.motor_norm_01, zeros(4, 1));
verifyLessThanOrEqual(testCase, actuator_cmd.motor_norm_01, ones(4, 1));
verifyGreaterThan(testCase, max(actuator_cmd.motor_norm_01), 0.0);
verifyGreaterThan(testCase, max(meta.motor_thrust_norm), 0.0);
verifySize(testCase, motor_cmd_radps, [4 1]);
verifyGreaterThan(testCase, max(motor_cmd_radps), 0.0);
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build one plant diagnostic snapshot from state.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.imu.accel_bias_b_mps2 = zeros(3, 1);
params.sensors.imu.accel_noise_std_b_mps2 = zeros(3, 1);
params.sensors.imu.gyro_bias_b_radps = zeros(3, 1);
params.sensors.imu.gyro_noise_std_b_radps = zeros(3, 1);
params.sensors.baro.alt_bias_m = 0.0;
params.sensors.baro.alt_noise_std_m = 0.0;
params.sensors.gnss.pos_bias_ned_m = zeros(3, 1);
params.sensors.gnss.pos_noise_std_ned_m = zeros(3, 1);
params.sensors.gnss.vel_bias_ned_mps = zeros(3, 1);
params.sensors.gnss.vel_noise_std_ned_mps = zeros(3, 1);
params.sensors.mag.field_bias_b_uT = zeros(3, 1);
params.sensors.mag.field_noise_std_b_uT = zeros(3, 1);
end
