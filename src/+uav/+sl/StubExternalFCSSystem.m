classdef StubExternalFCSSystem < matlab.System
%STUBEXTERNALFCSSYSTEM Minimal external FCS placeholder for SIL smoke tests.
% Description:
%   Produces a canonical external actuator packet from the SIL sensor
%   packet using only a hover trim, a small barometric correction, and an
%   optional yaw bias. It is intentionally simple and exists only as a
%   placeholder before real ArduPilot/PX4 adapters.
%
% Inputs:
%   sensor_packet_in - canonical external sensor packet struct
%
% Outputs:
%   actuator_cmd_out - canonical external actuator command struct
%
% Units:
%   SI only, normalized actuator command in [0, 1]
%
% Assumptions:
%   Stub behavior is configured through params.sil.stub_fcs.

    properties (Nontunable)
        mode_name = "hover";
        params = struct();
    end

    properties (Constant, Access = private)
        ActuatorBusName = 'uavStage15ExternalActuatorBus';
    end

    methods
        function obj = StubExternalFCSSystem(varargin)
        %STUBEXTERNALFCSSYSTEM Construct the temporary external FCS stub.

            obj.params = uav.sim.default_params_quad_x250();
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function validatePropertiesImpl(obj)
        %VALIDATEPROPERTIESIMPL Validate the stub controller properties.

            local_validate_mode_name(obj.mode_name);
            local_validate_params(obj.params);
        end

        function validateInputsImpl(~, sensor_packet_in)
        %VALIDATEINPUTSIMPL Validate the incoming sensor packet.

            local_validate_sensor_packet(sensor_packet_in);
        end

        function actuator_cmd_out = stepImpl(obj, sensor_packet_in)
        %STEPIMPL Produce one canonical external actuator command.

            sensor_packet_in = local_validate_sensor_packet(sensor_packet_in);
            cfg = obj.params.sil.stub_fcs;

            base_cmd = uav.sil.hover_trim_from_params(obj.params);
            alt_error_m = cfg.baro_alt_ref_m - sensor_packet_in.baro.alt_m;
            collective_corr = cfg.baro_k_p .* alt_error_m;
            collective_corr = min(max(collective_corr, ...
                -cfg.baro_corr_limit_norm), cfg.baro_corr_limit_norm);

            motor_norm_01 = base_cmd + collective_corr;
            if local_validate_mode_name(obj.mode_name) == "yaw_step" && ...
                    sensor_packet_in.time_s >= obj.params.demo.yaw_step_time_s
                motor_norm_01 = motor_norm_01 + ...
                    cfg.yaw_bias_norm .* obj.params.spin_dir(:);
            end

            motor_norm_01 = min(max(motor_norm_01, 0.0), 1.0);

            actuator_cmd_out = struct();
            actuator_cmd_out.mode = "norm01";
            actuator_cmd_out.motor_norm_01 = motor_norm_01(:);
        end

        function [out1] = getOutputSizeImpl(~)
        %GETOUTPUTSIZEIMPL Return the scalar bus size of the actuator output.

            out1 = [1, 1];
        end

        function [out1] = getOutputDataTypeImpl(~)
        %GETOUTPUTDATATYPEIMPL Bind the output to the actuator bus object.

            out1 = ['Bus: ' uav.sl.StubExternalFCSSystem.ActuatorBusName];
        end

        function [out1] = isOutputComplexImpl(~)
        %ISOUTPUTCOMPLEXIMPL Declare a real-valued actuator output bus.

            out1 = false;
        end

        function [out1] = isOutputFixedSizeImpl(~)
        %ISOUTPUTFIXEDSIZEIMPL Declare a fixed-size actuator output bus.

            out1 = true;
        end

        function [name1] = getOutputNamesImpl(~)
        %GETOUTPUTNAMESIMPL Name the actuator output port.

            name1 = 'actuator_cmd_out';
        end

        function num = getNumOutputsImpl(~)
        %GETNUMOUTPUTSIMPL Return the number of outputs.

            num = 1;
        end

        function icon = getIconImpl(obj)
        %GETICONIMPL Return a compact block icon label.

            icon = sprintf('Stub\\n%s', upper(char(local_validate_mode_name(obj.mode_name))));
        end
    end
end

function mode_name = local_validate_mode_name(mode_value)
%LOCAL_VALIDATE_MODE_NAME Normalize and validate one stub mode string.
% Description:
%   Converts a char or string scalar mode identifier into one normalized
%   lower-case string and validates it against the supported stub modes.
%
% Inputs:
%   mode_value - stub mode representation
%
% Outputs:
%   mode_name - normalized stub mode string
%
% Units:
%   not applicable
%
% Assumptions:
%   Only "hover" and "yaw_step" are supported.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
elseif ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
else
    error('uav:sl:StubExternalFCSSystem:ModeType', ...
        'Expected mode_name to be a char vector or string scalar.');
end

if ~any(mode_name == ["hover", "yaw_step"])
    error('uav:sl:StubExternalFCSSystem:UnsupportedMode', ...
        'Unsupported mode_name "%s".', mode_name);
end
end

function local_validate_params(params)
%LOCAL_VALIDATE_PARAMS Validate the minimal stub-controller config.
% Description:
%   Checks that the parameter struct contains the fields needed by the
%   temporary external FCS placeholder.
%
% Inputs:
%   params - parameter struct
%
% Outputs:
%   none
%
% Units:
%   not applicable
%
% Assumptions:
%   params.sil.stub_fcs is used for stub-only gains and limits.

if ~isstruct(params) || ~isscalar(params)
    error('uav:sl:StubExternalFCSSystem:ParamsType', ...
        'Expected params to be a scalar struct.');
end

required_fields = {'motor', 'rotor', 'mass_kg', 'gravity_mps2', 'sil', ...
    'demo', 'spin_dir'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(params, field_name)
        error('uav:sl:StubExternalFCSSystem:MissingParamsField', ...
            'Expected params.%s to be present.', field_name);
    end
end

if ~isstruct(params.sil) || ~isfield(params.sil, 'stub_fcs') || ...
        ~isstruct(params.sil.stub_fcs)
    error('uav:sl:StubExternalFCSSystem:MissingStubConfig', ...
        'Expected params.sil.stub_fcs to be present.');
end

cfg = params.sil.stub_fcs;
cfg_fields = {'baro_alt_ref_m', 'baro_k_p', 'baro_corr_limit_norm', ...
    'yaw_bias_norm'};
for k = 1:numel(cfg_fields)
    field_name = cfg_fields{k};
    if ~isfield(cfg, field_name)
        error('uav:sl:StubExternalFCSSystem:MissingStubField', ...
            'Expected params.sil.stub_fcs.%s to be present.', field_name);
    end
    validateattributes(cfg.(field_name), {'numeric'}, ...
        {'real', 'scalar', 'finite'}, ...
        mfilename, ['params.sil.stub_fcs.' field_name]);
end

validateattributes(cfg.baro_corr_limit_norm, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'params.sil.stub_fcs.baro_corr_limit_norm');
validateattributes(cfg.yaw_bias_norm, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'params.sil.stub_fcs.yaw_bias_norm');
validateattributes(params.spin_dir, {'numeric'}, ...
    {'real', 'finite', 'numel', 4}, mfilename, 'params.spin_dir');
validateattributes(params.demo.yaw_step_time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'params.demo.yaw_step_time_s');
end

function packet = local_validate_sensor_packet(packet)
%LOCAL_VALIDATE_SENSOR_PACKET Validate one external sensor packet.
% Description:
%   Checks that the incoming packet carries the fields required by the
%   temporary stub controller.
%
% Inputs:
%   packet - canonical external sensor packet
%
% Outputs:
%   packet - validated packet with normalized vector shapes
%
% Units:
%   SI only
%
% Assumptions:
%   The packet follows the uav.sil.make_sensor_packet format.

if ~isstruct(packet) || ~isscalar(packet)
    error('uav:sl:StubExternalFCSSystem:SensorPacketType', ...
        'Expected sensor_packet_in to be a scalar struct.');
end

required_fields = {'time_s', 'baro', 'imu', 'gnss', 'mag'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(packet, field_name)
        error('uav:sl:StubExternalFCSSystem:MissingPacketField', ...
            'Expected sensor_packet_in.%s to be present.', field_name);
    end
end

validateattributes(packet.time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'sensor_packet_in.time_s');
validateattributes(packet.baro.alt_m, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, ...
    mfilename, 'sensor_packet_in.baro.alt_m');
validateattributes(packet.imu.gyro_b_radps, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, ...
    mfilename, 'sensor_packet_in.imu.gyro_b_radps');

packet.imu.gyro_b_radps = packet.imu.gyro_b_radps(:);
end
