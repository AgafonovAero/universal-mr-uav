function tests = test_baro_measure
%TEST_BARO_MEASURE Tests for the barometric measurement model.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testBaroAltitudeMatchesNegativeNedZ(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);
state.p_ned_m = [5.0; -2.0; -123.4];

baro = uav.sensors.baro_measure(state, params);

verifyEqual(testCase, baro.alt_m, 123.4, 'AbsTol', 1.0e-12);
end

function testPressureDecreasesWithAltitude(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state_low = uav.core.state_unpack(params.demo.initial_state_plant);
state_high = state_low;

state_low.p_ned_m = [0.0; 0.0; 0.0];
state_high.p_ned_m = [0.0; 0.0; -200.0];

baro_low = uav.sensors.baro_measure(state_low, params);
baro_high = uav.sensors.baro_measure(state_high, params);

verifyGreaterThan(testCase, baro_low.pressure_pa, baro_high.pressure_pa);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.baro.alt_bias_m = 0.0;
params.sensors.baro.alt_noise_std_m = 0.0;
end
