function tests = test_mag_measure
%TEST_MAG_MEASURE Tests for the body-frame magnetometer model.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testIdentityQuaternionKeepsNedField(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);

mag = uav.sensors.mag_measure(state, params);

verifySize(testCase, mag.field_b_uT, [3, 1]);
verifyEqual(testCase, mag.field_b_uT, params.env.mag_ned_uT, 'AbsTol', 1.0e-12);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.mag.field_bias_b_uT = zeros(3, 1);
params.sensors.mag.field_noise_std_b_uT = zeros(3, 1);
end
