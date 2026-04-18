function tests = test_gnss_measure
%TEST_GNSS_MEASURE Tests for the GNSS position and velocity model.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testGnssReturnsPositionAndRotatedVelocityInNed(testCase)
params = local_zero_sensor_noise(uav.sim.default_params_quad_x250());
state = uav.core.state_unpack(params.demo.initial_state_plant);

state.p_ned_m = [10.0; -20.0; 30.0];
state.v_b_mps = [1.0; 2.0; 3.0];
state.q_nb = [cos(pi / 4.0); 0.0; 0.0; sin(pi / 4.0)];

gnss = uav.sensors.gnss_measure(state, params);

verifySize(testCase, gnss.pos_ned_m, [3, 1]);
verifySize(testCase, gnss.vel_ned_mps, [3, 1]);
verifyEqual(testCase, gnss.pos_ned_m, state.p_ned_m, 'AbsTol', 1.0e-12);
verifyEqual(testCase, gnss.vel_ned_mps, [-2.0; 1.0; 3.0], 'AbsTol', 1.0e-12);
end

function params = local_zero_sensor_noise(params)
%LOCAL_ZERO_SENSOR_NOISE Force deterministic sensor settings.

params.sensors.gnss.pos_bias_ned_m = zeros(3, 1);
params.sensors.gnss.pos_noise_std_ned_m = zeros(3, 1);
params.sensors.gnss.vel_bias_ned_mps = zeros(3, 1);
params.sensors.gnss.vel_noise_std_ned_mps = zeros(3, 1);
end
