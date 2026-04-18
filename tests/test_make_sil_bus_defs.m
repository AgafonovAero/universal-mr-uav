function tests = test_make_sil_bus_defs
%TEST_MAKE_SIL_BUS_DEFS Tests for SIL bus-definition generation.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testMakeSilBusDefsCreatesTopLevelBuses(testCase)
bus_defs = uav.sl.make_sil_bus_defs();

verifyEqual(testCase, bus_defs.external_actuator, 'uavStage15ExternalActuatorBus');
verifyEqual(testCase, bus_defs.sensor_packet, 'uavStage15SensorPacketBus');
verifyEqual(testCase, bus_defs.truth, 'uavStage15SILTruthBus');
verifyEqual(testCase, bus_defs.diag, 'uavStage15SILDiagBus');

actuator_bus = evalin('base', bus_defs.external_actuator);
sensor_packet_bus = evalin('base', bus_defs.sensor_packet);
truth_bus = evalin('base', bus_defs.truth);
diag_bus = evalin('base', bus_defs.diag);

verifyClass(testCase, actuator_bus, 'Simulink.Bus');
verifyClass(testCase, sensor_packet_bus, 'Simulink.Bus');
verifyClass(testCase, truth_bus, 'Simulink.Bus');
verifyClass(testCase, diag_bus, 'Simulink.Bus');

verifyEqual(testCase, {actuator_bus.Elements.Name}, ...
    {'mode', 'motor_norm_01'});
verifyEqual(testCase, actuator_bus.Elements(1).DataType, 'string');
verifyEqual(testCase, {sensor_packet_bus.Elements.Name}, ...
    {'time_s', 'imu_valid', 'imu', 'baro_valid', 'baro', ...
    'mag_valid', 'mag', 'gnss_valid', 'gnss'});
end
