classdef Stage15SILBridgeSystem < matlab.System
%STAGE15SILBRIDGESYSTEM Thin SIL-prep bridge over the Stage-1.5+ kernel.
% Description:
%   Wraps the existing plant, sensor, and estimator MATLAB APIs into a
%   discrete matlab.System boundary that is ready for future external
%   flight-stack adapters without moving physics or estimation logic into
%   the `.slx` model.
%
% Inputs:
%   actuator_cmd_in - canonical external actuator command struct
%
% Outputs:
%   sensor_packet_out - canonical external sensor packet struct
%   truth_out         - struct with state and estimator snapshots
%   diag_out          - compact diagnostic struct
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   Simulink.Bus definitions are created by uav.sl.make_sil_bus_defs
%   before the model is updated or simulated.

    properties (Nontunable)
        dt_s = 0.01;
        params = struct();
        state0 = struct( ...
            'p_ned_m', zeros(3, 1), ...
            'v_b_mps', zeros(3, 1), ...
            'q_nb', [1.0; 0.0; 0.0; 0.0], ...
            'w_b_radps', zeros(3, 1), ...
            'omega_m_radps', zeros(4, 1));
        rates_cfg = struct();
        time_s = 0.0;
        actuation_mode = "norm01";
    end

    properties (Access = private)
        state_internal
        estimator_internal
        estimator_initialized
        scheduler_internal
        sensor_packet_internal
        time_internal
    end

    properties (Constant, Access = private)
        SensorPacketBusName = 'uavStage15SensorPacketBus';
        TruthBusName = 'uavStage15SILTruthBus';
        DiagBusName = 'uavStage15SILDiagBus';
    end

    methods
        function obj = Stage15SILBridgeSystem(varargin)
        %STAGE15SILBRIDGESYSTEM Construct the thin SIL-prep bridge.

            defaults = uav.sim.default_params_quad_x250();

            obj.params = defaults;
            obj.dt_s = defaults.demo.dt_s;
            obj.state0 = uav.core.state_unpack(defaults.demo.initial_state_plant);
            obj.rates_cfg = local_default_rates_cfg(obj.dt_s, defaults);
            obj.time_s = 0.0;
            obj.actuation_mode = "norm01";

            has_params = local_has_property(varargin, 'params');
            has_dt = local_has_property(varargin, 'dt_s');
            has_state0 = local_has_property(varargin, 'state0');
            has_rates = local_has_property(varargin, 'rates_cfg');

            setProperties(obj, nargin, varargin{:});

            if has_params && ~has_dt && isfield(obj.params, 'demo') && ...
                    isfield(obj.params.demo, 'dt_s')
                obj.dt_s = obj.params.demo.dt_s;
            end

            if has_params && ~has_state0 && isfield(obj.params, 'demo') && ...
                    isfield(obj.params.demo, 'initial_state_plant')
                obj.state0 = uav.core.state_unpack( ...
                    obj.params.demo.initial_state_plant);
            end

            if ~has_rates
                obj.rates_cfg = local_default_rates_cfg(obj.dt_s, obj.params);
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj, ~)
        %SETUPIMPL Initialize the internal plant, estimator, and scheduler.

            obj.state_internal = uav.core.state_validate(obj.state0);
            obj.estimator_internal = local_empty_estimator_sample();
            obj.estimator_initialized = false;
            obj.scheduler_internal = [];
            obj.sensor_packet_internal = local_empty_sensor_packet();
            obj.time_internal = obj.time_s;
        end

        function resetImpl(obj)
        %RESETIMPL Reset the internal plant, estimator, and scheduler.

            obj.state_internal = uav.core.state_validate(obj.state0);
            obj.estimator_internal = local_empty_estimator_sample();
            obj.estimator_initialized = false;
            obj.scheduler_internal = [];
            obj.sensor_packet_internal = local_empty_sensor_packet();
            obj.time_internal = obj.time_s;
        end

        function validatePropertiesImpl(obj)
        %VALIDATEPROPERTIESIMPL Validate bridge configuration properties.

            validateattributes(obj.dt_s, {'numeric'}, ...
                {'real', 'scalar', 'finite', 'positive'}, ...
                mfilename, 'dt_s');
            validateattributes(obj.time_s, {'numeric'}, ...
                {'real', 'scalar', 'finite', 'nonnegative'}, ...
                mfilename, 'time_s');

            local_validate_mode_name(obj.actuation_mode, 'actuation_mode');
            uav.core.state_validate(obj.state0);
            local_validate_params(obj.params);
            uav.sil.update_rate_scheduler([], obj.dt_s, obj.rates_cfg);
        end

        function validateInputsImpl(obj, actuator_cmd_in)
        %VALIDATEINPUTSIMPL Validate the external actuator packet input.

            local_validate_actuator_packet_shape(actuator_cmd_in);
            if local_has_nonempty_mode(actuator_cmd_in.mode)
                local_validate_mode_match(actuator_cmd_in, obj.actuation_mode);
                uav.sil.actuator_cmd_to_motor_radps(actuator_cmd_in, obj.params);
            end
        end

        function [sensor_packet_out, truth_out, diag_out] = ...
                stepImpl(obj, actuator_cmd_in)
        %STEPIMPL Produce one packet/truth sample and advance the kernel.

            state_now = uav.core.state_validate(obj.state_internal);
            plant_diag = local_snapshot_diag(state_now, obj.params);
            sensors_now = uav.sensors.sensors_step(state_now, plant_diag, obj.params);

            [obj.scheduler_internal, valid_flags] = ...
                uav.sil.update_rate_scheduler( ...
                    obj.scheduler_internal, obj.dt_s, obj.rates_cfg);

            sensor_packet_out = uav.sil.make_sensor_packet( ...
                obj.time_internal, sensors_now, valid_flags, ...
                obj.sensor_packet_internal);
            obj.sensor_packet_internal = sensor_packet_out;

            estimator_sens = local_sensor_sample_from_packet(sensor_packet_out);
            if ~logical(obj.estimator_initialized)
                obj.estimator_internal = uav.est.estimator_init( ...
                    obj.params, estimator_sens);
                dt_est_s = 0.0;
                obj.estimator_initialized = true;
            else
                dt_est_s = obj.dt_s;
            end

            [estimator_out, ~] = uav.est.estimator_step( ...
                obj.estimator_internal, estimator_sens, dt_est_s, obj.params);
            obj.estimator_internal = estimator_out;

            truth_out = struct();
            truth_out.state = state_now;
            truth_out.estimator = estimator_out;

            diag_out = struct();
            diag_out.quat_norm_true = plant_diag.quat_norm;
            diag_out.quat_norm_est = norm(estimator_out.q_nb);
            diag_out.omega_m_radps = state_now.omega_m_radps;

            local_validate_mode_match(actuator_cmd_in, obj.actuation_mode);
            motor_cmd_radps = uav.sil.actuator_cmd_to_motor_radps( ...
                actuator_cmd_in, obj.params);
            obj.state_internal = uav.sim.plant_step_struct( ...
                state_now, motor_cmd_radps, obj.dt_s, obj.params);
            obj.time_internal = obj.time_internal + obj.dt_s;
        end

        function sts = getSampleTimeImpl(obj)
        %GETSAMPLETIMEIMPL Declare the bridge as discrete fixed-step.

            sts = createSampleTime(obj, ...
                'Type', 'Discrete', ...
                'SampleTime', obj.dt_s);
        end

        function [out1, out2, out3] = getOutputSizeImpl(~)
        %GETOUTPUTSIZEIMPL Return scalar bus sizes for the outputs.

            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(~)
        %GETOUTPUTDATATYPEIMPL Bind outputs to generated bus objects.

            out1 = ['Bus: ' uav.sl.Stage15SILBridgeSystem.SensorPacketBusName];
            out2 = ['Bus: ' uav.sl.Stage15SILBridgeSystem.TruthBusName];
            out3 = ['Bus: ' uav.sl.Stage15SILBridgeSystem.DiagBusName];
        end

        function [out1, out2, out3] = isOutputComplexImpl(~)
        %ISOUTPUTCOMPLEXIMPL Declare real-valued bus outputs.

            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
        %ISOUTPUTFIXEDSIZEIMPL Declare fixed-size bus outputs.

            out1 = true;
            out2 = true;
            out3 = true;
        end

        function [name1, name2, name3] = getOutputNamesImpl(~)
        %GETOUTPUTNAMESIMPL Name the bridge outputs.

            name1 = 'sensor_packet_out';
            name2 = 'truth_out';
            name3 = 'diag_out';
        end

        function direct_feedthrough = isInputDirectFeedthroughImpl(~, ~)
        %ISINPUTDIRECTFEEDTHROUGHIMPL Input affects only the next sample.

            direct_feedthrough = false;
        end

        function num = getNumOutputsImpl(~)
        %GETNUMOUTPUTSIMPL Return the number of bridge outputs.

            num = 3;
        end

        function icon = getIconImpl(~)
        %GETICONIMPL Return a compact block icon label.

            icon = sprintf('Stage15\\nSIL');
        end
    end
end

function plant_diag = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build one plant diagnostic snapshot from state.
% Description:
%   Recomputes the summed forces and moments from the rotor speeds stored
%   in the canonical state struct so the sensor and estimator layers can be
%   sampled without moving their logic into Simulink.
%
% Inputs:
%   state  - canonical plant state struct
%   params - parameter struct
%
% Outputs:
%   plant_diag - diagnostic struct with forces, moments, and quat norm
%
% Units:
%   SI only
%
% Assumptions:
%   The state quaternion follows the scalar-first q_nb convention.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

plant_diag = struct();
plant_diag.forces_b_N = fm.forces_b_N;
plant_diag.moments_b_Nm = fm.moments_b_Nm;
plant_diag.quat_norm = norm(state.q_nb);
end

function est = local_empty_estimator_sample()
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Build one empty estimator sample.
% Description:
%   Creates a deterministic placeholder estimator state so the bridge can
%   reset cleanly before the first real sensor packet arrives.
%
% Inputs:
%   none
%
% Outputs:
%   est - placeholder estimator struct
%
% Units:
%   SI only
%
% Assumptions:
%   Identity quaternion is the neutral initial attitude.

est = struct();
est.attitude = struct( ...
    'q_nb', [1.0; 0.0; 0.0; 0.0], ...
    'euler_rpy_rad', zeros(3, 1));
est.altitude = struct( ...
    'alt_m', 0.0, ...
    'vz_mps', 0.0);
est.q_nb = est.attitude.q_nb;
est.euler_rpy_rad = est.attitude.euler_rpy_rad;
est.alt_m = est.altitude.alt_m;
est.vz_mps = est.altitude.vz_mps;
end

function packet = local_empty_sensor_packet()
%LOCAL_EMPTY_SENSOR_PACKET Build one deterministic zero sensor packet.
% Description:
%   Seeds the bridge with a zero packet before the first scheduler update.
%
% Inputs:
%   none
%
% Outputs:
%   packet - canonical sensor packet struct
%
% Units:
%   SI only
%
% Assumptions:
%   Zero values are used only before the first valid sample.

packet = struct();
packet.time_s = 0.0;
packet.imu_valid = false;
packet.imu = struct( ...
    'accel_b_mps2', zeros(3, 1), ...
    'gyro_b_radps', zeros(3, 1));
packet.baro_valid = false;
packet.baro = struct( ...
    'alt_m', 0.0, ...
    'pressure_pa', 0.0);
packet.mag_valid = false;
packet.mag = struct( ...
    'field_b_uT', zeros(3, 1));
packet.gnss_valid = false;
packet.gnss = struct( ...
    'pos_ned_m', zeros(3, 1), ...
    'vel_ned_mps', zeros(3, 1));
end

function sens = local_sensor_sample_from_packet(packet)
%LOCAL_SENSOR_SAMPLE_FROM_PACKET Convert one packet back to sensor struct.
% Description:
%   Extracts the held sensor values from the canonical external packet so
%   the existing estimator layer can remain the source of truth.
%
% Inputs:
%   packet - canonical external sensor packet struct
%
% Outputs:
%   sens - internal sensor snapshot struct
%
% Units:
%   SI only
%
% Assumptions:
%   The packet already stores held values for slower channels.

sens = struct();
sens.imu = packet.imu;
sens.baro = packet.baro;
sens.mag = packet.mag;
sens.gnss = packet.gnss;
end

function local_validate_params(params)
%LOCAL_VALIDATE_PARAMS Validate the minimal required parameter fields.
% Description:
%   Performs a compact structural validation so configuration issues are
%   reported at model-update time instead of only during simulation.
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
%   Detailed field validation remains implemented in the downstream APIs.

if ~isstruct(params) || ~isscalar(params)
    error('uav:sl:Stage15SILBridgeSystem:ParamsType', ...
        'Expected params to be a scalar struct.');
end

required_fields = {'mass_kg', 'gravity_mps2', 'env', 'rotor', 'motor', ...
    'sensors', 'estimator'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(params, field_name)
        error('uav:sl:Stage15SILBridgeSystem:MissingParamsField', ...
            'Expected params.%s to be present.', field_name);
    end
end

validateattributes(params.mass_kg, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.mass_kg');
validateattributes(params.gravity_mps2, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.gravity_mps2');
end

function rates_cfg = local_default_rates_cfg(dt_s, params)
%LOCAL_DEFAULT_RATES_CFG Build one default SIL scheduler config.
% Description:
%   Uses params.sil.rates when available and otherwise falls back to the
%   minimal deterministic Stage-1.5+ scheduler defaults.
%
% Inputs:
%   dt_s   - base discrete step [s]
%   params - parameter struct
%
% Outputs:
%   rates_cfg - scheduler-rate config struct
%
% Units:
%   seconds
%
% Assumptions:
%   The fallback rates match TASK-08 requirements.

if isstruct(params) && isfield(params, 'sil') && isstruct(params.sil) && ...
        isfield(params.sil, 'rates') && isstruct(params.sil.rates)
    rates_cfg = params.sil.rates;
    rates_cfg.base_dt_s = dt_s;
    return;
end

rates_cfg = struct( ...
    'base_dt_s', dt_s, ...
    'baro_period_s', 0.02, ...
    'mag_period_s', 0.02, ...
    'gnss_period_s', 0.1);
end

function mode_name = local_validate_mode_name(mode_value, arg_name)
%LOCAL_VALIDATE_MODE_NAME Normalize and validate one mode string.
% Description:
%   Converts a char or string scalar mode identifier into a normalized
%   lower-case string and ensures it matches the only supported mode.
%
% Inputs:
%   mode_value - actuator mode representation
%   arg_name   - argument name for diagnostics
%
% Outputs:
%   mode_name - normalized mode string
%
% Units:
%   not applicable
%
% Assumptions:
%   Only "norm01" is supported at this stage.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
elseif ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
else
    error('uav:sl:Stage15SILBridgeSystem:ModeType', ...
        'Expected %s to be a char vector or string scalar.', arg_name);
end

if mode_name ~= "norm01"
    error('uav:sl:Stage15SILBridgeSystem:UnsupportedMode', ...
        'Unsupported %s "%s". Only "norm01" is supported.', ...
        arg_name, mode_name);
end
end

function local_validate_mode_match(actuator_cmd_in, expected_mode)
%LOCAL_VALIDATE_MODE_MATCH Ensure the incoming mode matches the property.
% Description:
%   Validates that the external actuator packet uses the configured bridge
%   actuation mode.
%
% Inputs:
%   actuator_cmd_in - actuator command struct
%   expected_mode   - configured bridge mode
%
% Outputs:
%   none
%
% Units:
%   not applicable
%
% Assumptions:
%   actuator_cmd_in.mode exists.

expected_mode = local_validate_mode_name(expected_mode, 'actuation_mode');
incoming_mode = local_validate_mode_name(actuator_cmd_in.mode, ...
    'actuator_cmd_in.mode');

if incoming_mode ~= expected_mode
    error('uav:sl:Stage15SILBridgeSystem:ActuationModeMismatch', ...
        'Expected actuator_cmd_in.mode to match actuation_mode.');
end
end

function local_validate_actuator_packet_shape(actuator_cmd_in)
%LOCAL_VALIDATE_ACTUATOR_PACKET_SHAPE Validate actuator-packet structure.
% Description:
%   Performs a compile-time-safe validation of the actuator packet fields
%   without requiring the mode string to be populated yet.
%
% Inputs:
%   actuator_cmd_in - actuator command struct
%
% Outputs:
%   none
%
% Units:
%   not applicable
%
% Assumptions:
%   The mode field can be an empty placeholder during model compilation.

if ~isstruct(actuator_cmd_in) || ~isscalar(actuator_cmd_in)
    error('uav:sl:Stage15SILBridgeSystem:ActuatorPacketType', ...
        'Expected actuator_cmd_in to be a scalar struct.');
end

required_fields = {'mode', 'motor_norm_01'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(actuator_cmd_in, field_name)
        error('uav:sl:Stage15SILBridgeSystem:MissingActuatorField', ...
            'Expected actuator_cmd_in.%s to be present.', field_name);
    end
end

validateattributes(actuator_cmd_in.motor_norm_01, {'numeric'}, ...
    {'real', 'finite', 'numel', 4}, ...
    mfilename, 'actuator_cmd_in.motor_norm_01');
end

function tf = local_has_nonempty_mode(mode_value)
%LOCAL_HAS_NONEMPTY_MODE Return true when one mode value is populated.
% Description:
%   Distinguishes a real runtime mode string from the empty placeholder
%   values Simulink may use while compiling the model.
%
% Inputs:
%   mode_value - mode field from the actuator packet
%
% Outputs:
%   tf - true when the mode contains non-whitespace text
%
% Units:
%   not applicable
%
% Assumptions:
%   Empty or whitespace-only values should be treated as placeholders.

if isstring(mode_value) && isscalar(mode_value)
    tf = strlength(strtrim(mode_value)) > 0;
    return;
end

if ischar(mode_value)
    tf = ~isempty(strtrim(mode_value));
    return;
end

tf = false;
end

function tf = local_has_property(args, property_name)
%LOCAL_HAS_PROPERTY Check whether varargin contains one property name.
% Description:
%   Performs a case-insensitive search across name/value constructor inputs.
%
% Inputs:
%   args          - constructor varargin cell array
%   property_name - property name to search
%
% Outputs:
%   tf - true when the property name is present
%
% Units:
%   not applicable
%
% Assumptions:
%   Name/value inputs are well-formed pairs.

tf = false;
for k = 1:2:numel(args)
    if ischar(args{k}) || (isstring(args{k}) && isscalar(args{k}))
        if strcmpi(string(args{k}), string(property_name))
            tf = true;
            return;
        end
    end
end
end
