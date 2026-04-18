classdef Stage15MILSystem < matlab.System
%STAGE15MILSYSTEM Thin MIL wrapper over the Stage-1.5 code-centric kernel.
% Description:
%   Wraps the existing plant, sensor, and estimator MATLAB APIs into a
%   discrete matlab.System so Simulink can orchestrate the Stage-1.5+
%   model without moving physics or estimation logic into the block
%   diagram.
%
% Inputs:
%   motor_cmd_radps - motor speed command with 4 elements [rad/s]
%
% Outputs:
%   state_out     - canonical plant state struct
%   sensors_out   - aggregated sensor sample struct
%   estimator_out - aggregated estimator state/output struct
%   diag_out      - plant and estimator diagnostic struct
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   Simulink.Bus definitions are created by uav.sl.make_bus_defs before the
%   model is updated or simulated.

    properties (Nontunable)
        dt_s = 0.01;
        state0 = struct( ...
            'p_ned_m', zeros(3, 1), ...
            'v_b_mps', zeros(3, 1), ...
            'q_nb', [1.0; 0.0; 0.0; 0.0], ...
            'w_b_radps', zeros(3, 1), ...
            'omega_m_radps', zeros(4, 1));
        params = struct();
    end

    properties (Access = private)
        state_internal
        estimator_internal
        estimator_initialized
    end

    properties (Constant, Access = private)
        StateBusName = 'uavStage15StateBus';
        SensorsBusName = 'uavStage15SensorsBus';
        EstimatorBusName = 'uavStage15EstimatorBus';
        DiagBusName = 'uavStage15DiagBus';
    end

    methods
        function obj = Stage15MILSystem(varargin)
        %STAGE15MILSYSTEM Construct the thin MIL wrapper with defaults.

            defaults = uav.sim.default_params_quad_x250();

            obj.params = defaults;
            obj.dt_s = defaults.demo.dt_s;
            obj.state0 = uav.core.state_unpack(defaults.demo.initial_state_plant);

            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, ~)
        %SETUPIMPL Initialize the internal plant and estimator states.

            obj.state_internal = uav.core.state_validate(obj.state0);
            obj.estimator_internal = local_empty_estimator_sample();
            obj.estimator_initialized = false;
        end

        function resetImpl(obj)
        %RESETIMPL Reset the internal plant and estimator states.

            obj.state_internal = uav.core.state_validate(obj.state0);
            obj.estimator_internal = local_empty_estimator_sample();
            obj.estimator_initialized = false;
        end

        function validatePropertiesImpl(obj)
        %VALIDATEPROPERTIESIMPL Validate tunable configuration properties.

            validateattributes(obj.dt_s, {'numeric'}, ...
                {'real', 'scalar', 'finite', 'positive'}, ...
                mfilename, 'dt_s');
            uav.core.state_validate(obj.state0);
            local_validate_params(obj.params);
        end

        function validateInputsImpl(~, motor_cmd_radps)
        %VALIDATEINPUTSIMPL Validate the motor-command input shape.

            validateattributes(motor_cmd_radps, {'numeric'}, ...
                {'real', 'finite', 'numel', 4}, ...
                mfilename, 'motor_cmd_radps');
        end

        function [state_out, sensors_out, estimator_out, diag_out] = ...
                stepImpl(obj, motor_cmd_radps)
        %STEPIMPL Produce one sampled output and advance internal states.

            motor_cmd_radps = motor_cmd_radps(:);
            state_out = uav.core.state_validate(obj.state_internal);

            plant_diag = local_snapshot_diag(state_out, obj.params);
            sensors_out = uav.sensors.sensors_step(state_out, plant_diag, obj.params);

            if ~logical(obj.estimator_initialized)
                obj.estimator_internal = uav.est.estimator_init(obj.params, sensors_out);
                dt_est_s = 0.0;
                obj.estimator_initialized = true;
            else
                dt_est_s = obj.dt_s;
            end

            [estimator_out, estimator_diag] = uav.est.estimator_step( ...
                obj.estimator_internal, sensors_out, dt_est_s, obj.params);

            obj.estimator_internal = estimator_out;

            diag_out = struct();
            diag_out.plant = plant_diag;
            diag_out.estimator = estimator_diag;

            [state_next, ~] = uav.sim.plant_step_struct( ...
                state_out, motor_cmd_radps, obj.dt_s, obj.params);
            obj.state_internal = state_next;
        end

        function sts = getSampleTimeImpl(obj)
        %GETSAMPLETIMEIMPL Declare the block as discrete fixed-step.

            sts = createSampleTime(obj, ...
                'Type', 'Discrete', ...
                'SampleTime', obj.dt_s);
        end

        function [out1, out2, out3, out4] = getOutputSizeImpl(~)
        %GETOUTPUTSIZEIMPL Return scalar bus sizes for the four outputs.

            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = [1, 1];
        end

        function [out1, out2, out3, out4] = getOutputDataTypeImpl(~)
        %GETOUTPUTDATATYPEIMPL Bind outputs to generated bus objects.

            out1 = ['Bus: ' uav.sl.Stage15MILSystem.StateBusName];
            out2 = ['Bus: ' uav.sl.Stage15MILSystem.SensorsBusName];
            out3 = ['Bus: ' uav.sl.Stage15MILSystem.EstimatorBusName];
            out4 = ['Bus: ' uav.sl.Stage15MILSystem.DiagBusName];
        end

        function [out1, out2, out3, out4] = isOutputComplexImpl(~)
        %ISOUTPUTCOMPLEXIMPL Declare real-valued bus outputs.

            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
        end

        function [out1, out2, out3, out4] = isOutputFixedSizeImpl(~)
        %ISOUTPUTFIXEDSIZEIMPL Declare fixed-size bus outputs.

            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
        end

        function [name1, name2, name3, name4] = getOutputNamesImpl(~)
        %GETOUTPUTNAMESIMPL Name the block outputs.

            name1 = 'state_out';
            name2 = 'sensors_out';
            name3 = 'estimator_out';
            name4 = 'diag_out';
        end

        function direct_feedthrough = isInputDirectFeedthroughImpl(~, ~)
        %ISINPUTDIRECTFEEDTHROUGHIMPL Input affects only the next sample.

            direct_feedthrough = false;
        end

        function num = getNumOutputsImpl(~)
        %GETNUMOUTPUTSIMPL Return the number of outputs.

            num = 4;
        end

        function icon = getIconImpl(~)
        %GETICONIMPL Return a compact block icon label.

            icon = sprintf('Stage15\\nMIL');
        end
    end
end

function plant_diag = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Build one plant diagnostic snapshot from state.
% Description:
%   Recomputes the current summed forces and moments from the rotor speeds
%   stored in the canonical state struct so the sensor and estimator layers
%   can be sampled without moving their logic into Simulink.
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
%   The state quaternion already follows the scalar-first q_nb convention.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

plant_diag = struct();
plant_diag.forces_b_N = fm.forces_b_N;
plant_diag.moments_b_Nm = fm.moments_b_Nm;
plant_diag.quat_norm = norm(state.q_nb);
end

function est = local_empty_estimator_sample()
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Build one empty estimator sample.
% Description:
%   Creates a deterministic placeholder estimator state so the System
%   object can reset cleanly before the first real sensor sample arrives.
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

function local_validate_params(params)
%LOCAL_VALIDATE_PARAMS Validate the minimal required parameter fields.
% Description:
%   Performs a compact structural validation so configuration issues are
%   reported at model update time instead of only during simulation.
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
    error('uav:sl:Stage15MILSystem:ParamsType', ...
        'Expected params to be a scalar struct.');
end

required_fields = {'mass_kg', 'gravity_mps2', 'env', 'motor', ...
    'sensors', 'estimator'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(params, field_name)
        error('uav:sl:Stage15MILSystem:MissingParamsField', ...
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
