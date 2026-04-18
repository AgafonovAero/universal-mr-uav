function tests = test_state_pack_unpack
%TEST_STATE_PACK_UNPACK Tests for the canonical plant state API helpers.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testPackUnpackRoundTripIsConsistent(testCase)
params = uav.sim.default_params_quad_x250();
state_ref = uav.core.state_unpack(params.demo.initial_state_plant);

x_packed = uav.core.state_pack(state_ref);
state_roundtrip = uav.core.state_unpack(x_packed);

verifyEqual(testCase, x_packed, params.demo.initial_state_plant, 'AbsTol', 1.0e-12);
verifyEqual(testCase, state_roundtrip, state_ref);
end

function testStateValidateRejectsWrongQuaternionSize(testCase)
params = uav.sim.default_params_quad_x250();
state_bad = uav.core.state_unpack(params.demo.initial_state_plant);
state_bad.q_nb = zeros(5, 1);

verifyError(testCase, @() uav.core.state_validate(state_bad), ...
    'uav:core:state_validate:FieldSize');
end
