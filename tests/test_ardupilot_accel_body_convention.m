function tests = test_ardupilot_accel_body_convention
%TEST_ARDUPILOT_ACCEL_BODY_CONVENTION Проверки accel_body по официальной формуле.
% Назначение:
%   Подтверждает, что вычисление accel_body в TASK-25 следует логике
%   официального примера ArduPilot MATLAB backend.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testGroundStateProducesMinusGravityInBodyFrame(testCase)
params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();

state = uav.core.state_unpack(params.demo.initial_state_plant);
state.p_ned_m = [0.0; 0.0; 0.0];
state.v_b_mps = [0.0; 0.0; 0.0];
state.w_b_radps = [0.0; 0.0; 0.0];

sensors = struct();
sensors.imu = struct( ...
    'accel_b_mps2', [0.0; 0.0; 0.0], ...
    'gyro_b_radps', [0.0; 0.0; 0.0]);

[sample, ~] = uav.ardupilot.convert_state_to_official_json( ...
    state, sensors, 0.0, params, cfg, ...
    'SnapshotDiag', struct('forces_b_N', [0.0; 0.0; 0.0]));

verifyEqual(testCase, sample.imu.accel_body, [0.0, 0.0, -params.gravity_mps2], ...
    'AbsTol', 1.0e-9);
end

function testHoverSpecificForceMatchesOfficialFormula(testCase)
params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();

state = uav.core.state_unpack(params.demo.initial_state_plant);
state.p_ned_m = [0.0; 0.0; -2.0];
state.v_b_mps = [0.0; 0.0; 0.0];
state.w_b_radps = [0.0; 0.0; 0.0];

hover_force_b_N = [0.0; 0.0; -params.mass_kg * params.gravity_mps2];
sensors = struct();
sensors.imu = struct( ...
    'accel_b_mps2', hover_force_b_N ./ params.mass_kg, ...
    'gyro_b_radps', [0.0; 0.0; 0.0]);

[sample, ~] = uav.ardupilot.convert_state_to_official_json( ...
    state, sensors, 0.2, params, cfg, ...
    'SnapshotDiag', struct('forces_b_N', hover_force_b_N));

verifyEqual(testCase, sample.imu.accel_body, [0.0, 0.0, -params.gravity_mps2], ...
    'AbsTol', 1.0e-9);
end
