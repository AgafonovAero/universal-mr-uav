function bus_defs = make_bus_defs()
%MAKE_BUS_DEFS Create Simulink.Bus definitions for the thin MIL shell.
% Description:
%   Generates the minimal bus objects required by uav.sl.Stage15MILSystem
%   and assigns them into the base workspace without using `.sldd`.
%
% Inputs:
%   none
%
% Outputs:
%   bus_defs - struct with the top-level bus names and the full list
%
% Units:
%   not applicable
%
% Assumptions:
%   Simulink is available in the local MATLAB installation.

local_require_simulink();

bus_map = struct();

bus_map.uavStage15StateBus = local_make_bus([ ...
    local_bus_element('p_ned_m', [3, 1], 'double'); ...
    local_bus_element('v_b_mps', [3, 1], 'double'); ...
    local_bus_element('q_nb', [4, 1], 'double'); ...
    local_bus_element('w_b_radps', [3, 1], 'double'); ...
    local_bus_element('omega_m_radps', [4, 1], 'double')]);

bus_map.uavStage15ImuBus = local_make_bus([ ...
    local_bus_element('accel_b_mps2', [3, 1], 'double'); ...
    local_bus_element('gyro_b_radps', [3, 1], 'double')]);

bus_map.uavStage15BaroBus = local_make_bus([ ...
    local_bus_element('alt_m', 1, 'double'); ...
    local_bus_element('pressure_pa', 1, 'double')]);

bus_map.uavStage15MagBus = local_make_bus( ...
    local_bus_element('field_b_uT', [3, 1], 'double'));

bus_map.uavStage15GnssBus = local_make_bus([ ...
    local_bus_element('pos_ned_m', [3, 1], 'double'); ...
    local_bus_element('vel_ned_mps', [3, 1], 'double')]);

bus_map.uavStage15SensorsBus = local_make_bus([ ...
    local_bus_element('imu', 1, 'Bus: uavStage15ImuBus'); ...
    local_bus_element('baro', 1, 'Bus: uavStage15BaroBus'); ...
    local_bus_element('mag', 1, 'Bus: uavStage15MagBus'); ...
    local_bus_element('gnss', 1, 'Bus: uavStage15GnssBus')]);

bus_map.uavStage15AttitudeBus = local_make_bus([ ...
    local_bus_element('q_nb', [4, 1], 'double'); ...
    local_bus_element('euler_rpy_rad', [3, 1], 'double')]);

bus_map.uavStage15AltitudeBus = local_make_bus([ ...
    local_bus_element('alt_m', 1, 'double'); ...
    local_bus_element('vz_mps', 1, 'double')]);

bus_map.uavStage15EstimatorBus = local_make_bus([ ...
    local_bus_element('attitude', 1, 'Bus: uavStage15AttitudeBus'); ...
    local_bus_element('altitude', 1, 'Bus: uavStage15AltitudeBus'); ...
    local_bus_element('q_nb', [4, 1], 'double'); ...
    local_bus_element('euler_rpy_rad', [3, 1], 'double'); ...
    local_bus_element('alt_m', 1, 'double'); ...
    local_bus_element('vz_mps', 1, 'double')]);

bus_map.uavStage15PlantDiagBus = local_make_bus([ ...
    local_bus_element('forces_b_N', [3, 1], 'double'); ...
    local_bus_element('moments_b_Nm', [3, 1], 'double'); ...
    local_bus_element('quat_norm', 1, 'double')]);

bus_map.uavStage15AttitudeDiagBus = local_make_bus([ ...
    local_bus_element('quat_norm', 1, 'double'); ...
    local_bus_element('accel_correction_weight', 1, 'double'); ...
    local_bus_element('accel_consistency_metric', 1, 'double'); ...
    local_bus_element('acc_correction_norm', 1, 'double'); ...
    local_bus_element('mag_correction_norm', 1, 'double')]);

bus_map.uavStage15AltitudeDiagBus = local_make_bus([ ...
    local_bus_element('az_ned_mps2', 1, 'double'); ...
    local_bus_element('baro_residual_m', 1, 'double')]);

bus_map.uavStage15EstimatorDiagBus = local_make_bus([ ...
    local_bus_element('attitude', 1, 'Bus: uavStage15AttitudeDiagBus'); ...
    local_bus_element('altitude', 1, 'Bus: uavStage15AltitudeDiagBus')]);

bus_map.uavStage15DiagBus = local_make_bus([ ...
    local_bus_element('plant', 1, 'Bus: uavStage15PlantDiagBus'); ...
    local_bus_element('estimator', 1, 'Bus: uavStage15EstimatorDiagBus')]);

bus_names = fieldnames(bus_map);
for k = 1:numel(bus_names)
    assignin('base', bus_names{k}, bus_map.(bus_names{k}));
end

bus_defs = struct();
bus_defs.state = 'uavStage15StateBus';
bus_defs.sensors = 'uavStage15SensorsBus';
bus_defs.estimator = 'uavStage15EstimatorBus';
bus_defs.diag = 'uavStage15DiagBus';
bus_defs.all = bus_names;
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
%   Convenience helper for concise and explicit bus definitions.
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
%   Dimensions correspond to the existing Stage-1.5+ struct API.

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
    error('uav:sl:make_bus_defs:SimulinkUnavailable', ...
        'Simulink.Bus is unavailable. Thin MIL shell requires Simulink.');
end
end
