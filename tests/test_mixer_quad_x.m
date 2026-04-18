function tests = test_mixer_quad_x
%TEST_MIXER_QUAD_X Basic tests for the quad-X mixer geometry and symmetry.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testHoverMixDistributesThrustEqually(testCase)
params = uav.sim.default_params_quad_x250();
totalThrust_N = params.mass_kg * params.gravity_mps2;

[omega_cmd_radps, thrust_cmd_N] = uav.vmg.mixer_quad_x(totalThrust_N, [0.0; 0.0; 0.0], params);

verifyGreaterThanOrEqual(testCase, omega_cmd_radps, zeros(4, 1));
verifyGreaterThanOrEqual(testCase, thrust_cmd_N, zeros(4, 1));
verifyEqual(testCase, thrust_cmd_N, repmat(totalThrust_N / 4.0, 4, 1), 'AbsTol', 1.0e-12);
end

function testMixerGeometryMatchesPresetParameters(testCase)
params = uav.sim.default_params_quad_x250();
motor_xy_m = params.motor_xy_m;

verifySize(testCase, motor_xy_m, [4, 2]);
verifyEqual(testCase, params.motor_radius_m, params.wheelbase_m / 2.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, vecnorm(motor_xy_m, 2, 2), repmat(params.motor_radius_m, 4, 1), 'AbsTol', 1.0e-12);
verifyEqual( ...
    testCase, ...
    norm(motor_xy_m(1, :) - motor_xy_m(3, :)), ...
    params.wheelbase_m, ...
    'AbsTol', 1.0e-12);
verifyEqual( ...
    testCase, ...
    norm(motor_xy_m(2, :) - motor_xy_m(4, :)), ...
    params.wheelbase_m, ...
    'AbsTol', 1.0e-12);
end

function testMixerReproducesRequestedMoments(testCase)
params = uav.sim.default_params_quad_x250();
totalThrust_N = params.mass_kg * params.gravity_mps2;
bodyMoments_Nm = [0.08; -0.05; 0.02];

[~, thrust_cmd_N] = uav.vmg.mixer_quad_x(totalThrust_N, bodyMoments_Nm, params);
reconstructed = local_reconstruct_wrench(thrust_cmd_N, params);

verifyEqual(testCase, reconstructed(1), totalThrust_N, 'AbsTol', 1.0e-12);
verifyEqual(testCase, reconstructed(2:4), bodyMoments_Nm, 'AbsTol', 1.0e-12);
end

function testPureRollCommandIsSymmetricAcrossFrontAndRear(testCase)
params = uav.sim.default_params_quad_x250();
totalThrust_N = params.mass_kg * params.gravity_mps2;

[~, thrust_cmd_N] = uav.vmg.mixer_quad_x(totalThrust_N, [0.05; 0.0; 0.0], params);

verifyEqual(testCase, thrust_cmd_N(1), thrust_cmd_N(4), 'AbsTol', 1.0e-12);
verifyEqual(testCase, thrust_cmd_N(2), thrust_cmd_N(3), 'AbsTol', 1.0e-12);
verifyGreaterThan(testCase, thrust_cmd_N(1), thrust_cmd_N(2));
end

function wrench = local_reconstruct_wrench(thrust_cmd_N, params)
%LOCAL_RECONSTRUCT_WRENCH Rebuild thrust and moments from rotor thrusts.

x_i_m = params.motor_xy_m(:, 1);
y_i_m = params.motor_xy_m(:, 2);
yaw_gain_m = params.kQ_Nm_per_radps2 / params.kT_N_per_radps2;

wrench = [ ...
    sum(thrust_cmd_N); ...
    sum(-y_i_m .* thrust_cmd_N); ...
    sum(x_i_m .* thrust_cmd_N); ...
    sum(yaw_gain_m .* params.spin_dir .* thrust_cmd_N)];
end
