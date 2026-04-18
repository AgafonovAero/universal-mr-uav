function tests = test_hover_balance
%TEST_HOVER_BALANCE Basic test for baseline hover force balance.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testBaselineHoverThrustBalancesWeight(testCase)
params = uav.sim.default_params_quad_x250();
rotor = uav.vmg.rotor_coeffs(params);

[omega_cmd_radps, thrust_cmd_N] = uav.vmg.mixer_quad_x( ...
    params.mass_kg * params.gravity_mps2, [0.0; 0.0; 0.0], params);
[rotorThrust_N, ~] = uav.vmg.rotor_simple( ...
    omega_cmd_radps, rotor.kT_N_per_radps2, rotor.kQ_Nm_per_radps2);

verifyGreaterThanOrEqual(testCase, omega_cmd_radps, zeros(4, 1));
verifyEqual(testCase, thrust_cmd_N, rotorThrust_N, 'AbsTol', 1.0e-12);
verifyEqual( ...
    testCase, ...
    sum(rotorThrust_N), ...
    params.mass_kg * params.gravity_mps2, ...
    'AbsTol', 1.0e-12);
end
