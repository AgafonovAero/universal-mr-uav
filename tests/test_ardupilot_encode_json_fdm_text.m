function tests = test_ardupilot_encode_json_fdm_text
%TEST_ARDUPILOT_ENCODE_JSON_FDM_TEXT Проверки кодирования JSON TASK-11.
% Назначение:
%   Подтверждает наличие обязательных полей в строке JSON, формируемой
%   из канонического пакета данных средства сопряжения.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testJsonContainsRequiredFieldsAndNewline(testCase)
%TESTJSONCONTAINSREQUIREDFIELDSANDNEWLINE Проверить состав JSON.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
state = uav.core.state_unpack(params.demo.initial_state_plant);
snapshot = local_snapshot_diag(state, params);
sensors = uav.sensors.sensors_step(state, snapshot, params);
estimator = uav.est.estimator_init(params, sensors);

packet = uav.ardupilot.pack_json_fdm_packet( ...
    state, sensors, estimator, 0.0, params, cfg);
json_text = uav.ardupilot.encode_json_fdm_text(packet);

verifyTrue(testCase, endsWith(json_text, newline));
json_data = jsondecode(char(strip(json_text)));

verifyTrue(testCase, isfield(json_data, 'timestamp'));
verifyTrue(testCase, isfield(json_data, 'imu'));
verifyTrue(testCase, isfield(json_data.imu, 'gyro'));
verifyTrue(testCase, isfield(json_data.imu, 'accel_body'));
verifyTrue(testCase, isfield(json_data, 'position'));
verifyTrue(testCase, isfield(json_data, 'velocity'));
verifyTrue(testCase, isfield(json_data, 'quaternion'));
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Построить детерминированный диагностический снимок.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end
