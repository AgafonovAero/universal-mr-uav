function [actuator_cmd, meta] = unpack_atc_actuation(controller_out, controller_ctx)
%UNPACK_ATC_ACTUATION Convert external ATC output into canonical actuation.
% Description:
%   Translates `atc_controller` motor outputs into the canonical
%   universal-mr-uav actuator packet by explicitly converting
%   `actuator -> normalized thrust -> normalized omega`.
%
% Inputs:
%   controller_out - output struct from `FSW_Simulink_wrapper_step`
%   controller_ctx - context struct from `uav.atc.make_controller_context`
%
% Outputs:
%   actuator_cmd - canonical actuator packet with mode `norm01`
%   meta         - compact struct with raw and converted ATC actuation data
%
% Units:
%   normalized actuator output in [0, 1]
%
% Assumptions:
%   The external controller is configured for quad-X and four motors.

controller_out = local_validate_controller_out(controller_out);
controller_ctx = local_validate_controller_ctx(controller_ctx);

actuator_raw = single(controller_out.motor_cmd(:));
thrust_norm = feval( ...
    char(controller_ctx.repo.required_functions(end)), ...
    actuator_raw, controller_ctx.mix_params);
thrust_norm = double(reshape(thrust_norm, 4, 1));
thrust_norm = min(max(thrust_norm, 0.0), 1.0);

motor_norm_01 = sqrt(thrust_norm);
motor_norm_01 = min(max(motor_norm_01, 0.0), 1.0);

actuator_cmd = struct();
actuator_cmd.mode = "norm01";
actuator_cmd.motor_norm_01 = motor_norm_01(:);

meta = struct();
meta.motor_actuator_01 = double(actuator_raw(:));
meta.motor_thrust_norm = thrust_norm(:);
meta.mode_used = uint8(controller_out.mode_used);
meta.spool_state = uint8(controller_out.spool_state);
meta.spool_desired = uint8(controller_out.spool_desired);
meta.fs_active = logical(controller_out.fs_active);
meta.fs_reason = uint8(controller_out.fs_reason);
meta.atc_dbg = double(reshape(controller_out.atc_dbg, 12, 1));
end

function controller_out = local_validate_controller_out(controller_out)
%LOCAL_VALIDATE_CONTROLLER_OUT Validate the required ATC output fields.

if ~isstruct(controller_out) || ~isscalar(controller_out)
    error('uav:atc:unpack_atc_actuation:ControllerOutType', ...
        'Expected controller_out to be a scalar struct.');
end

required_fields = {'motor_cmd', 'mode_used', 'spool_state', ...
    'spool_desired', 'fs_active', 'fs_reason', 'atc_dbg'};
for k = 1:numel(required_fields)
    if ~isfield(controller_out, required_fields{k})
        error('uav:atc:unpack_atc_actuation:MissingControllerField', ...
            'Expected controller_out.%s to be present.', required_fields{k});
    end
end

validateattributes(controller_out.motor_cmd, {'numeric'}, ...
    {'real', 'finite', 'numel', 4}, mfilename, 'controller_out.motor_cmd');
validateattributes(controller_out.atc_dbg, {'numeric'}, ...
    {'real', 'finite', 'numel', 12}, mfilename, 'controller_out.atc_dbg');
end

function controller_ctx = local_validate_controller_ctx(controller_ctx)
%LOCAL_VALIDATE_CONTROLLER_CTX Validate the direct-call controller context.

if ~isstruct(controller_ctx) || ~isscalar(controller_ctx)
    error('uav:atc:unpack_atc_actuation:ControllerCtxType', ...
        'Expected controller_ctx to be a scalar struct.');
end

required_fields = {'repo', 'mix_params'};
for k = 1:numel(required_fields)
    if ~isfield(controller_ctx, required_fields{k})
        error('uav:atc:unpack_atc_actuation:MissingControllerCtxField', ...
            'Expected controller_ctx.%s to be present.', required_fields{k});
    end
end
end
