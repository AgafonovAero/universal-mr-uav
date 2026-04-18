function tests = test_make_bus_defs
%TEST_MAKE_BUS_DEFS Tests for Simulink bus-definition generation.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testMakeBusDefsCreatesTopLevelBuses(testCase)
bus_defs = uav.sl.make_bus_defs();

verifyEqual(testCase, bus_defs.state, 'uavStage15StateBus');
verifyEqual(testCase, bus_defs.sensors, 'uavStage15SensorsBus');
verifyEqual(testCase, bus_defs.estimator, 'uavStage15EstimatorBus');
verifyEqual(testCase, bus_defs.diag, 'uavStage15DiagBus');

state_bus = evalin('base', bus_defs.state);
sensors_bus = evalin('base', bus_defs.sensors);
estimator_bus = evalin('base', bus_defs.estimator);
diag_bus = evalin('base', bus_defs.diag);

verifyClass(testCase, state_bus, 'Simulink.Bus');
verifyClass(testCase, sensors_bus, 'Simulink.Bus');
verifyClass(testCase, estimator_bus, 'Simulink.Bus');
verifyClass(testCase, diag_bus, 'Simulink.Bus');
verifyEqual(testCase, {state_bus.Elements.Name}, ...
    {'p_ned_m', 'v_b_mps', 'q_nb', 'w_b_radps', 'omega_m_radps'});
end
