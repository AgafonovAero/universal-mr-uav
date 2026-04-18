function tests = test_stage15_sil_bridge
%TEST_STAGE15_SIL_BRIDGE Tests for the thin SIL-prep bridge wrapper.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testBridgeMatchesCodeCentricReferenceLoop(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
dt_s = params.demo.dt_s;
state0 = uav.core.state_unpack(params.demo.initial_state_plant);
rates_cfg = params.sil.rates;
actuator_cmd = struct( ...
    'mode', "norm01", ...
    'motor_norm_01', uav.sil.hover_trim_from_params(params));

bridge = uav.sl.Stage15SILBridgeSystem( ...
    'dt_s', dt_s, ...
    'params', params, ...
    'state0', state0, ...
    'rates_cfg', rates_cfg, ...
    'time_s', 0.0, ...
    'actuation_mode', "norm01");
cleanup = onCleanup(@() release(bridge)); %#ok<NASGU>

state_ref = state0;
est_ref = local_empty_estimator_sample();
packet_ref = [];
scheduler_ref = [];
time_ref = 0.0;
is_initialized_ref = false;

baro_valid_hist = false(12, 1);
gnss_valid_hist = false(12, 1);

for k = 1:12
    [sensor_packet, truth_out, diag_out] = step(bridge, actuator_cmd);
    [sensor_packet_ref, truth_ref, diag_ref, state_ref, est_ref, ...
        packet_ref, scheduler_ref, time_ref, is_initialized_ref] = ...
        local_reference_step( ...
            state_ref, est_ref, packet_ref, scheduler_ref, time_ref, ...
            is_initialized_ref, actuator_cmd, dt_s, params, rates_cfg);

    baro_valid_hist(k) = sensor_packet.baro_valid;
    gnss_valid_hist(k) = sensor_packet.gnss_valid;

    verifyEqual(testCase, sensor_packet.time_s, sensor_packet_ref.time_s, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, sensor_packet.imu.accel_b_mps2, ...
        sensor_packet_ref.imu.accel_b_mps2, 'AbsTol', 1.0e-12);
    verifyEqual(testCase, sensor_packet.baro.alt_m, ...
        sensor_packet_ref.baro.alt_m, 'AbsTol', 1.0e-12);
    verifyEqual(testCase, truth_out.state.p_ned_m, truth_ref.state.p_ned_m, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, truth_out.state.q_nb, truth_ref.state.q_nb, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, truth_out.estimator.q_nb, truth_ref.estimator.q_nb, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, truth_out.estimator.alt_m, truth_ref.estimator.alt_m, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, diag_out.omega_m_radps, diag_ref.omega_m_radps, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, diag_out.quat_norm_true, diag_ref.quat_norm_true, ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, diag_out.quat_norm_est, diag_ref.quat_norm_est, ...
        'AbsTol', 1.0e-12);
end

verifyEqual(testCase, baro_valid_hist(:).', ...
    logical([1 0 1 0 1 0 1 0 1 0 1 0]));
verifyEqual(testCase, gnss_valid_hist(:).', ...
    logical([1 0 0 0 0 0 0 0 0 0 1 0]));
verifyLessThan(testCase, abs(diag_out.quat_norm_true - 1.0), 1.0e-9);
verifyLessThan(testCase, abs(diag_out.quat_norm_est - 1.0), 1.0e-9);
end

function [packet_ref, truth_ref, diag_ref, state_next, est_next, ...
        packet_hold, scheduler_next, time_next, is_initialized_next] = ...
        local_reference_step(state_ref, est_ref, packet_hold, ...
        scheduler_ref, time_ref, is_initialized_ref, actuator_cmd, ...
        dt_s, params, rates_cfg)
%LOCAL_REFERENCE_STEP Reference implementation of one bridge sample.

snapshot = local_snapshot_diag(state_ref, params);
sensors_now = uav.sensors.sensors_step(state_ref, snapshot, params);
[scheduler_next, valid_flags] = uav.sil.update_rate_scheduler( ...
    scheduler_ref, dt_s, rates_cfg);
packet_ref = uav.sil.make_sensor_packet( ...
    time_ref, sensors_now, valid_flags, packet_hold);

estimator_sens = struct( ...
    'imu', packet_ref.imu, ...
    'baro', packet_ref.baro, ...
    'mag', packet_ref.mag, ...
    'gnss', packet_ref.gnss);

if ~is_initialized_ref
    est_ref = uav.est.estimator_init(params, estimator_sens);
    dt_est_s = 0.0;
    is_initialized_next = true;
else
    dt_est_s = dt_s;
    is_initialized_next = true;
end

[est_next, ~] = uav.est.estimator_step(est_ref, estimator_sens, dt_est_s, params);

truth_ref = struct();
truth_ref.state = state_ref;
truth_ref.estimator = est_next;

diag_ref = struct();
diag_ref.quat_norm_true = snapshot.quat_norm;
diag_ref.quat_norm_est = norm(est_next.q_nb);
diag_ref.omega_m_radps = state_ref.omega_m_radps;

motor_cmd_radps = uav.sil.actuator_cmd_to_motor_radps(actuator_cmd, params);
state_next = uav.sim.plant_step_struct(state_ref, motor_cmd_radps, dt_s, params);
packet_hold = packet_ref;
time_next = time_ref + dt_s;
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build one plant diagnostic snapshot from state.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function est = local_empty_estimator_sample()
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Build one empty estimator sample.

est = struct();
est.attitude = struct( ...
    'q_nb', [1.0; 0.0; 0.0; 0.0], ...
    'euler_rpy_rad', zeros(3, 1));
est.altitude = struct( ...
    'alt_m', 0.0, ...
    'vz_mps', 0.0);
est.q_nb = est.attitude.q_nb;
est.euler_rpy_rad = est.attitude.euler_rpy_rad;
est.alt_m = est.altitude.alt_m;
est.vz_mps = est.altitude.vz_mps;
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
