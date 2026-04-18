function tests = test_quat_tools
%TEST_QUAT_TOOLS Basic tests for quaternion helper functions.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testQuatNormalizeReturnsUnitNorm(testCase)
q_raw = [2.0; -2.0; 1.0; 0.5];
q_unit = uav.core.quat_normalize(q_raw);

verifyEqual(testCase, norm(q_unit), 1.0, 'AbsTol', 1.0e-12);
end

function testQuatToDcmProducesProperRotation(testCase)
q_nb = uav.core.quat_normalize([0.9; 0.1; -0.2; 0.3]);
c_nb = uav.core.quat_to_dcm(q_nb);

verifySize(testCase, c_nb, [3, 3]);
verifyEqual(testCase, c_nb * c_nb.', eye(3), 'AbsTol', 1.0e-12);
verifyEqual(testCase, det(c_nb), 1.0, 'AbsTol', 1.0e-12);
end
