function tests = test_ardupilot_official_json_frame
%TEST_ARDUPILOT_OFFICIAL_JSON_FRAME Проверки официального JSON-кадра ArduPilot.
% Назначение:
%   Подтверждает, что новый TASK-25 кадр MATLAB backend формирует
%   обязательные поля официального JSON интерфейса ArduPilot.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testOfficialJsonFrameContainsMandatoryFields(testCase)
params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
state = uav.core.state_unpack(params.demo.initial_state_plant);

snapshot = local_snapshot_diag(state, params);
sensors = uav.sensors.sensors_step(state, snapshot, params);

[sample, diag] = uav.ardupilot.convert_state_to_official_json( ...
    state, sensors, 1.25, params, cfg, 'SnapshotDiag', snapshot);
frame = uav.ardupilot.build_official_json_frame(sample, cfg);

verifyTrue(testCase, isfield(frame, 'timestamp'));
verifyTrue(testCase, isfield(frame, 'imu'));
verifyTrue(testCase, isfield(frame.imu, 'gyro'));
verifyTrue(testCase, isfield(frame.imu, 'accel_body'));
verifyTrue(testCase, isfield(frame, 'position'));
verifyTrue(testCase, isfield(frame, 'velocity'));
verifyTrue(testCase, isfield(frame, 'attitude'));
verifyFalse(testCase, isfield(frame, 'quaternion'));

verifySize(testCase, frame.imu.gyro, [1 3]);
verifySize(testCase, frame.imu.accel_body, [1 3]);
verifySize(testCase, frame.position, [1 3]);
verifySize(testCase, frame.velocity, [1 3]);
verifySize(testCase, frame.attitude, [1 3]);
verifyEqual(testCase, frame.timestamp, 1.25, 'AbsTol', 1.0e-12);
verifyTrue(testCase, isfinite(norm(diag.accel_diag.accel_ned_mps2)));
end

function snapshot = local_snapshot_diag(state, params)
fm = uav.core.forces_moments_sum(state.omega_m_radps, params);
snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end
