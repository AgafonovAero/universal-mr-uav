function tests = test_ardupilot_pack_json_fdm_packet
%TEST_ARDUPILOT_PACK_JSON_FDM_PACKET Проверки пакета данных TASK-10.
% Назначение:
%   Подтверждает наличие обязательных полей в каноническом пакете данных
%   и корректный перенос основных величин из состояния и измерений.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог `src`.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testPacketContainsRequiredTruthAndSensorFields(testCase)
%TESTPACKETCONTAINSREQUIREDTRUTHANDSENSORFIELDS Проверить состав пакета.

params = uav.sim.make_deterministic_demo_params();
state = uav.core.state_unpack(params.demo.initial_state_plant);
snapshot = local_snapshot_diag(state, params);
sensors = uav.sensors.sensors_step(state, snapshot, params);
estimator = uav.est.estimator_init(params, sensors);
cfg = uav.ardupilot.default_json_config();

packet = uav.ardupilot.pack_json_fdm_packet( ...
    state, ...
    sensors, ...
    estimator, ...
    0.0, ...
    params, ...
    cfg);

verifyTrue(testCase, isfield(packet, 'time_s'));
verifyTrue(testCase, isfield(packet, 'position_ned_m'));
verifyTrue(testCase, isfield(packet, 'velocity_ned_mps'));
verifyTrue(testCase, isfield(packet, 'q_nb'));
verifyTrue(testCase, isfield(packet, 'imu'));
verifyTrue(testCase, isfield(packet, 'baro'));
verifyTrue(testCase, isfield(packet, 'mag'));
verifyTrue(testCase, isfield(packet, 'gnss'));

verifyEqual(testCase, packet.position_ned_m, state.p_ned_m, 'AbsTol', 1.0e-12);
verifyEqual(testCase, packet.velocity_ned_mps, zeros(3, 1), 'AbsTol', 1.0e-12);
verifyEqual(testCase, packet.imu.gyro_b_radps, state.w_b_radps, 'AbsTol', 1.0e-12);
verifyEqual(testCase, packet.gnss.pos_ned_m, state.p_ned_m, 'AbsTol', 1.0e-12);
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Построить детерминированный диагностический снимок.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end
