function bus_defs = make_atc_bus_defs()
%MAKE_ATC_BUS_DEFS Create Simulink.Bus definitions for the ATC MIL shell.
% Description:
%   Generates the minimal bus objects required by
%   `uav.sl.Stage15ATCMILSystem` and assigns them into the base workspace
%   without using `.sldd`.
%
% Inputs:
%   none
%
% Outputs:
%   bus_defs - struct with the top-level ATC MIL bus names and the full list
%
% Units:
%   not applicable
%
% Assumptions:
%   Simulink is available in the local MATLAB installation.

local_require_simulink();

sil_bus_defs = uav.sl.make_sil_bus_defs();
bus_map = struct();

bus_map.uavStage15ATCDiagBus = local_make_bus([ ...
    local_bus_element('time_s', 1, 'double'); ...
    local_bus_element('controller_ready', 1, 'boolean'); ...
    local_bus_element('controller_reset_applied', 1, 'boolean'); ...
    local_bus_element('mode_cmd', 1, 'uint8'); ...
    local_bus_element('mode_used', 1, 'uint8'); ...
    local_bus_element('spool_state', 1, 'uint8'); ...
    local_bus_element('spool_desired', 1, 'uint8'); ...
    local_bus_element('fs_active', 1, 'boolean'); ...
    local_bus_element('fs_reason', 1, 'uint8'); ...
    local_bus_element('ground_contact', 1, 'boolean'); ...
    local_bus_element('quat_norm_true', 1, 'double'); ...
    local_bus_element('quat_norm_est', 1, 'double'); ...
    local_bus_element('motor_actuator_01', [4, 1], 'double'); ...
    local_bus_element('motor_thrust_norm', [4, 1], 'double'); ...
    local_bus_element('motor_cmd_radps', [4, 1], 'double'); ...
    local_bus_element('atc_dbg', [12, 1], 'double')]);

bus_names = fieldnames(bus_map);
for k = 1:numel(bus_names)
    assignin('base', bus_names{k}, bus_map.(bus_names{k}));
end

bus_defs = struct();
bus_defs.truth = 'uavStage15StateBus';
bus_defs.sensors = 'uavStage15SensorsBus';
bus_defs.estimator = 'uavStage15EstimatorBus';
bus_defs.atc_cmd = 'uavStage15ExternalActuatorBus';
bus_defs.diag = 'uavStage15ATCDiagBus';
bus_defs.sil = sil_bus_defs;
bus_defs.all = [sil_bus_defs.all; bus_names];
end

function bus = local_make_bus(elements)
%LOCAL_MAKE_BUS Build one Simulink.Bus object from its elements.

bus = Simulink.Bus;
bus.Elements = elements;
end

function element = local_bus_element(name, dimensions, data_type)
%LOCAL_BUS_ELEMENT Create one Simulink.BusElement.

element = Simulink.BusElement;
element.Name = name;
element.Dimensions = dimensions;
element.DataType = data_type;
element.Complexity = 'real';
element.SamplingMode = 'Sample based';
end

function local_require_simulink()
%LOCAL_REQUIRE_SIMULINK Validate that Simulink is available locally.

if exist('Simulink.Bus', 'class') ~= 8
    error('uav:sl:make_atc_bus_defs:SimulinkUnavailable', ...
        'Simulink.Bus is unavailable. Thin ATC MIL shell requires Simulink.');
end
end
