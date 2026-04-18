function [motor_cmd_radps, ctrl_state, diag] = demo_takeoff_hold_controller( ...
        ctrl_input, ctrl_state, dt_s, params, cfg)
%DEMO_TAKEOFF_HOLD_CONTROLLER Demo-level estimator-driven takeoff/hold law.
% Description:
%   Wraps the generic estimator-driven pitch/attitude demo controller with
%   zero roll, pitch, and yaw references so takeoff/hold scenarios stay
%   level while using only estimator outputs and measured gyro rates.
%
% Inputs:
%   ctrl_input - struct with estimator, sensors, and reference substructs
%   ctrl_state - controller state struct
%   dt_s       - controller sample time [s]
%   params     - vehicle parameter struct
%   cfg        - controller configuration struct
%
% Outputs:
%   motor_cmd_radps - 4x1 motor speed command [rad/s]
%   ctrl_state      - updated controller state
%   diag            - compact controller diagnostics
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   ctrl_input.reference contains altitude and vertical-speed references.

ctrl_input = local_force_level_reference(ctrl_input);
[motor_cmd_radps, ctrl_state, diag] = ...
    uav.ctrl.demo_pitch_hold_controller( ...
        ctrl_input, ctrl_state, dt_s, params, cfg);
end

function ctrl_input = local_force_level_reference(ctrl_input)
%LOCAL_FORCE_LEVEL_REFERENCE Set missing attitude references to zero.

if ~isstruct(ctrl_input) || ~isscalar(ctrl_input)
    error('uav:ctrl:demo_takeoff_hold_controller:InputType', ...
        'Expected ctrl_input to be a scalar struct.');
end

if ~isfield(ctrl_input, 'reference') || ~isstruct(ctrl_input.reference) || ...
        ~isscalar(ctrl_input.reference)
    error('uav:ctrl:demo_takeoff_hold_controller:ReferenceType', ...
        'Expected ctrl_input.reference to be a scalar struct.');
end

ctrl_input.reference.roll_ref_rad = 0.0;
ctrl_input.reference.pitch_ref_rad = 0.0;
ctrl_input.reference.yaw_ref_rad = 0.0;
end
