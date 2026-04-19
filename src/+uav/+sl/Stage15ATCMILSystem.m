classdef Stage15ATCMILSystem < matlab.System
%STAGE15ATCMILSYSTEM Thin ATC MIL wrapper over the Stage-1.5+ kernel.
% Description:
%   Wraps the existing plant, sensor, and estimator MATLAB APIs together
%   with a direct MATLAB call into the external `atc_controller`
%   repository. The Simulink block remains a thin orchestration shell over
%   the existing code-centric kernel.
%
% Inputs:
%   none
%
% Outputs:
%   truth_out     - canonical plant truth struct
%   sensors_out   - aggregated sensor sample struct
%   estimator_out - aggregated estimator state/output struct
%   atc_cmd_out   - canonical actuator command struct
%   diag_out      - compact bridge and controller diagnostic struct
%
% Units:
%   SI only in the kernel, with explicit conversion at the ATC boundary
%
% Assumptions:
%   Simulink.Bus definitions are created by `uav.sl.make_atc_bus_defs`
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
        bridge_cfg = struct();
        time_s = 0.0;
    end

    properties (Access = private)
        state_internal
        estimator_internal
        estimator_initialized
        controller_ctx
        controller_reset_pending
        time_internal
    end

    properties (Constant, Access = private)
        TruthBusName = 'uavStage15StateBus';
        SensorsBusName = 'uavStage15SensorsBus';
        EstimatorBusName = 'uavStage15EstimatorBus';
        ActuatorBusName = 'uavStage15ExternalActuatorBus';
        DiagBusName = 'uavStage15ATCDiagBus';
    end

    methods
        function obj = Stage15ATCMILSystem(varargin)
        %STAGE15ATCMILSYSTEM Construct the thin ATC MIL wrapper.

            defaults = uav.sim.default_params_quad_x250();

            obj.params = defaults;
            obj.dt_s = defaults.demo.dt_s;
            obj.state0 = uav.core.state_unpack(defaults.demo.initial_state_plant);
            obj.bridge_cfg = uav.atc.default_atc_bridge_config([], defaults);
            obj.time_s = 0.0;

            has_params = local_has_property(varargin, 'params');
            has_dt = local_has_property(varargin, 'dt_s');
            has_state0 = local_has_property(varargin, 'state0');
            has_bridge_cfg = local_has_property(varargin, 'bridge_cfg');

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

            if ~has_bridge_cfg
                obj.bridge_cfg = uav.atc.default_atc_bridge_config([], obj.params);
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
        %SETUPIMPL Initialize plant, estimator, and external controller.

            obj.state_internal = uav.core.state_validate(obj.state0);
            obj.estimator_internal = local_empty_estimator_sample();
            obj.estimator_initialized = false;
            obj.controller_ctx = uav.atc.make_controller_context( ...
                obj.bridge_cfg, obj.params);
            obj.controller_reset_pending = true;
            obj.time_internal = obj.time_s;
        end

        function resetImpl(obj)
        %RESETIMPL Reset plant, estimator, and controller state request.

            obj.state_internal = uav.core.state_validate(obj.state0);
            obj.estimator_internal = local_empty_estimator_sample();
            obj.estimator_initialized = false;
            obj.controller_ctx = uav.atc.make_controller_context( ...
                obj.bridge_cfg, obj.params);
            obj.controller_reset_pending = true;
            obj.time_internal = obj.time_s;
        end

        function validatePropertiesImpl(obj)
        %VALIDATEPROPERTIESIMPL Validate wrapper configuration.

            validateattributes(obj.dt_s, {'numeric'}, ...
                {'real', 'scalar', 'finite', 'positive'}, ...
                mfilename, 'dt_s');
            validateattributes(obj.time_s, {'numeric'}, ...
                {'real', 'scalar', 'finite', 'nonnegative'}, ...
                mfilename, 'time_s');
            uav.core.state_validate(obj.state0);
            local_validate_params(obj.params);
            local_validate_bridge_cfg(obj.bridge_cfg);
        end

        function [truth_out, sensors_out, estimator_out, atc_cmd_out, ...
                diag_out] = stepImpl(obj)
        %STEPIMPL Produce one MIL sample and advance internal states.

            truth_out = uav.core.state_validate(obj.state_internal);

            plant_diag = local_snapshot_diag(truth_out, obj.params);
            sensors_out = uav.sensors.sensors_step(truth_out, plant_diag, obj.params);

            if ~logical(obj.estimator_initialized)
                obj.estimator_internal = uav.est.estimator_init( ...
                    obj.params, sensors_out);
                dt_est_s = 0.0;
                obj.estimator_initialized = true;
            else
                dt_est_s = obj.dt_s;
            end

            [estimator_out, ~] = uav.est.estimator_step( ...
                obj.estimator_internal, sensors_out, dt_est_s, obj.params);
            obj.estimator_internal = estimator_out;

            controller_in = uav.atc.pack_sensor_packet_for_atc( ...
                sensors_out, estimator_out, obj.dt_s, obj.time_internal, ...
                obj.bridge_cfg, obj.controller_ctx.input_template);
            controller_in.reset = logical(obj.controller_reset_pending);

            controller_out = local_step_external_controller( ...
                obj.controller_ctx, controller_in);
            obj.controller_reset_pending = false;

            [atc_cmd_out, act_meta] = uav.atc.unpack_atc_actuation( ...
                controller_out, obj.controller_ctx);
            motor_cmd_radps = uav.sil.actuator_cmd_to_motor_radps( ...
                atc_cmd_out, obj.params);

            diag_out = struct();
            diag_out.time_s = obj.time_internal;
            diag_out.controller_ready = true;
            diag_out.controller_reset_applied = controller_in.reset;
            diag_out.mode_cmd = uint8(controller_in.mode_cmd);
            diag_out.mode_used = uint8(act_meta.mode_used);
            diag_out.spool_state = uint8(act_meta.spool_state);
            diag_out.spool_desired = uint8(act_meta.spool_desired);
            diag_out.fs_active = logical(act_meta.fs_active);
            diag_out.fs_reason = uint8(act_meta.fs_reason);
            diag_out.ground_contact = logical(controller_in.ground_contact);
            diag_out.quat_norm_true = plant_diag.quat_norm;
            diag_out.quat_norm_est = norm(estimator_out.q_nb);
            diag_out.motor_actuator_01 = act_meta.motor_actuator_01(:);
            diag_out.motor_thrust_norm = act_meta.motor_thrust_norm(:);
            diag_out.motor_cmd_radps = motor_cmd_radps(:);
            diag_out.atc_dbg = act_meta.atc_dbg(:);

            [state_next, ~] = uav.sim.plant_step_struct( ...
                truth_out, motor_cmd_radps, obj.dt_s, obj.params);
            obj.state_internal = state_next;
            obj.time_internal = obj.time_internal + obj.dt_s;
        end

        function sts = getSampleTimeImpl(obj)
        %GETSAMPLETIMEIMPL Declare the wrapper as discrete fixed-step.

            sts = createSampleTime(obj, ...
                'Type', 'Discrete', ...
                'SampleTime', obj.dt_s);
        end

        function [out1, out2, out3, out4, out5] = getOutputSizeImpl(~)
        %GETOUTPUTSIZEIMPL Return scalar bus sizes for the five outputs.

            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = [1, 1];
            out5 = [1, 1];
        end

        function [out1, out2, out3, out4, out5] = getOutputDataTypeImpl(~)
        %GETOUTPUTDATATYPEIMPL Bind outputs to generated bus objects.

            out1 = ['Bus: ' uav.sl.Stage15ATCMILSystem.TruthBusName];
            out2 = ['Bus: ' uav.sl.Stage15ATCMILSystem.SensorsBusName];
            out3 = ['Bus: ' uav.sl.Stage15ATCMILSystem.EstimatorBusName];
            out4 = ['Bus: ' uav.sl.Stage15ATCMILSystem.ActuatorBusName];
            out5 = ['Bus: ' uav.sl.Stage15ATCMILSystem.DiagBusName];
        end

        function [out1, out2, out3, out4, out5] = isOutputComplexImpl(~)
        %ISOUTPUTCOMPLEXIMPL Declare real-valued bus outputs.

            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
        end

        function [out1, out2, out3, out4, out5] = isOutputFixedSizeImpl(~)
        %ISOUTPUTFIXEDSIZEIMPL Declare fixed-size bus outputs.

            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
        end

        function [name1, name2, name3, name4, name5] = getOutputNamesImpl(~)
        %GETOUTPUTNAMESIMPL Name the wrapper outputs.

            name1 = 'truth_out';
            name2 = 'sensors_out';
            name3 = 'estimator_out';
            name4 = 'atc_cmd_out';
            name5 = 'diag_out';
        end

        function num = getNumOutputsImpl(~)
        %GETNUMOUTPUTSIMPL Return the number of outputs.

            num = 5;
        end

        function icon = getIconImpl(~)
        %GETICONIMPL Return a compact block icon label.

            icon = sprintf('ATC\\nMIL');
        end
    end
end

function controller_out = local_step_external_controller(controller_ctx, controller_in)
%LOCAL_STEP_EXTERNAL_CONTROLLER Run one direct MATLAB controller iteration.
% Description:
%   Calls the external `FSW_Simulink_wrapper_step` entrypoint with the
%   prepared fixed-field input struct and explicit parameter struct.
%
% Inputs:
%   controller_ctx - context struct from `uav.atc.make_controller_context`
%   controller_in  - input struct packed for the external controller
%
% Outputs:
%   controller_out - direct output struct from the external controller
%
% Units:
%   external controller native interface units
%
% Assumptions:
%   The required external path bootstrap already succeeded.

controller_out = feval( ...
    char(controller_ctx.repo.controller_step_function), ...
    controller_in, controller_ctx.P);
end

function plant_diag = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build one plant diagnostic snapshot from state.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

plant_diag = struct();
plant_diag.forces_b_N = fm.forces_b_N;
plant_diag.moments_b_Nm = fm.moments_b_Nm;
plant_diag.quat_norm = norm(state.q_nb);
end

function est = local_empty_estimator_sample()
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Build one empty estimator sample.

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

function local_validate_params(params)
%LOCAL_VALIDATE_PARAMS Validate the minimal wrapper parameter set.

if ~isstruct(params) || ~isscalar(params)
    error('uav:sl:Stage15ATCMILSystem:ParamsType', ...
        'Expected params to be a scalar struct.');
end

required_fields = {'motor', 'rotor', 'sensors', 'estimator', 'demo'};
for k = 1:numel(required_fields)
    if ~isfield(params, required_fields{k})
        error('uav:sl:Stage15ATCMILSystem:MissingParamsField', ...
            'Expected params.%s to be present.', required_fields{k});
    end
end
end

function local_validate_bridge_cfg(cfg)
%LOCAL_VALIDATE_BRIDGE_CFG Validate the wrapper bridge config.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:sl:Stage15ATCMILSystem:BridgeCfgType', ...
        'Expected bridge_cfg to be a scalar struct.');
end

required_fields = {'external_repo_root', 'mode_name', 'mode_cmd', ...
    'manual_throttle_norm', 'sim_motor_order_map'};
for k = 1:numel(required_fields)
    if ~isfield(cfg, required_fields{k})
        error('uav:sl:Stage15ATCMILSystem:MissingBridgeCfgField', ...
            'Expected bridge_cfg.%s to be present.', required_fields{k});
    end
end
end

function tf = local_has_property(args, property_name)
%LOCAL_HAS_PROPERTY Check whether one name-value property is supplied.

tf = false;
for idx = 1:2:numel(args)
    if idx + 1 > numel(args)
        break;
    end

    key = args{idx};
    if ~(ischar(key) || (isstring(key) && isscalar(key)))
        continue;
    end

    if strcmpi(char(string(key)), property_name)
        tf = true;
        return;
    end
end
end
