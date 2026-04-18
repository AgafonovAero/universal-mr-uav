function tests = test_forces_moments_sum
%TEST_FORCES_MOMENTS_SUM Basic tests for summed rotor wrench calculations.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testEqualRotorSpeedsProducePureNegativeZForce(testCase)
params = uav.sim.default_params_quad_x250();
omega_radps = repmat(params.hover_omega_radps, 4, 1);

fm = uav.core.forces_moments_sum(omega_radps, params);

verifyEqual(testCase, fm.moments_b_Nm(1), 0.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, fm.moments_b_Nm(2), 0.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, fm.moments_b_Nm(3), 0.0, 'AbsTol', 1.0e-12);
verifyEqual(testCase, fm.forces_b_N(1:2), [0.0; 0.0], 'AbsTol', 1.0e-12);
verifyLessThan(testCase, fm.forces_b_N(3), 0.0);
end
