function bus_defs = make_sil_bus_defs()
%MAKE_SIL_BUS_DEFS Create Simulink.Bus definitions for the SIL-prep shell.
% Description:
%   Generates the minimal bus objects required by the Stage-1.5+ SIL-prep
%   wrapper and assigns them into the base workspace without using `.sldd`.
%
% Inputs:
%   none
%
% Outputs:
%   bus_defs - struct with the top-level SIL bus names and the full list
%
% Units:
%   not applicable
%
% Assumptions:
%   Simulink is available in the local MATLAB installation.

local_require_simulink();

mil_bus_defs = uav.sl.make_bus_defs();
bus_map = struct();

bus_map.uavStage15ExternalActuatorBus = local_make_bus([ ...
    local_bus_element('mode', 1, 'string'); ...
    local_bus_element('motor_norm_01', [4, 1], 'double')]);

bus_map.uavStage15SensorPacketBus = local_make_bus([ ...
    local_bus_element('time_s', 1, 'double'); ...
    local_bus_element('imu_valid', 1, 'boolean'); ...
    local_bus_element('imu', 1, 'Bus: uavStage15ImuBus'); ...
    local_bus_element('baro_valid', 1, 'boolean'); ...
    local_bus_element('baro', 1, 'Bus: uavStage15BaroBus'); ...
    local_bus_element('mag_valid', 1, 'boolean'); ...
    local_bus_element('mag', 1, 'Bus: uavStage15MagBus'); ...
    local_bus_element('gnss_valid', 1, 'boolean'); ...
    local_bus_element('gnss', 1, 'Bus: uavStage15GnssBus')]);

bus_map.uavStage15SILTruthBus = local_make_bus([ ...
    local_bus_element('state', 1, 'Bus: uavStage15StateBus'); ...
    local_bus_element('estimator', 1, 'Bus: uavStage15EstimatorBus')]);

bus_map.uavStage15SILDiagBus = local_make_bus([ ...
    local_bus_element('quat_norm_true', 1, 'double'); ...
    local_bus_element('quat_norm_est', 1, 'double'); ...
    local_bus_element('omega_m_radps', [4, 1], 'double')]);

bus_names = fieldnames(bus_map);
for k = 1:numel(bus_names)
    assignin('base', bus_names{k}, bus_map.(bus_names{k}));
end

bus_defs = struct();
bus_defs.external_actuator = 'uavStage15ExternalActuatorBus';
bus_defs.sensor_packet = 'uavStage15SensorPacketBus';
bus_defs.truth = 'uavStage15SILTruthBus';
bus_defs.diag = 'uavStage15SILDiagBus';
bus_defs.mil = mil_bus_defs;
bus_defs.all = [mil_bus_defs.all; bus_names];
end

function bus = local_make_bus(elements)
%LOCAL_MAKE_BUS Build one Simulink.Bus object from its elements.
% Description:
%   Creates a Simulink.Bus object and attaches the provided element array.
%
% Inputs:
%   elements - Simulink.BusElement array
%
% Outputs:
%   bus - Simulink.Bus object
%
% Units:
%   not applicable
%
% Assumptions:
%   Each element already carries the correct dimensions and data type.

bus = Simulink.Bus;
bus.Elements = elements;
end

function element = local_bus_element(name, dimensions, data_type)
%LOCAL_BUS_ELEMENT Create one Simulink.BusElement.
% Description:
%   Convenience helper for concise and explicit SIL bus definitions.
%
% Inputs:
%   name       - bus element name
%   dimensions - scalar or dimension vector
%   data_type  - Simulink data type string
%
% Outputs:
%   element - configured Simulink.BusElement
%
% Units:
%   not applicable
%
% Assumptions:
%   Dimensions correspond to the canonical external interface.

element = Simulink.BusElement;
element.Name = name;
element.Dimensions = dimensions;
element.DataType = data_type;
element.Complexity = 'real';
element.SamplingMode = 'Sample based';
end

function local_require_simulink()
%LOCAL_REQUIRE_SIMULINK Validate that Simulink is available locally.
% Description:
%   Fails early with a clear message if Simulink classes are unavailable.
%
% Inputs:
%   none
%
% Outputs:
%   none
%
% Units:
%   not applicable
%
% Assumptions:
%   None.

if exist('Simulink.Bus', 'class') ~= 8
    error('uav:sl:make_sil_bus_defs:SimulinkUnavailable', ...
        'Simulink.Bus is unavailable. Thin SIL shell requires Simulink.');
end
end
