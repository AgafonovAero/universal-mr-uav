function ctx = make_controller_context(cfg, params)
%MAKE_CONTROLLER_CONTEXT Prepare the direct-call ATC controller context.
% Description:
%   Builds the external controller parameter struct, input template, and
%   mixer metadata required by the bridge while keeping all external-stack
%   assumptions explicit in MATLAB code.
%
% Inputs:
%   cfg    - scalar bridge configuration struct
%   params - universal-mr-uav plant parameter struct
%
% Outputs:
%   ctx - scalar context struct for `uav.sl.Stage15ATCMILSystem`
%
% Units:
%   SI only at the bridge boundary, with explicit conversion to the
%   external controller's ArduPilot-like units where required
%
% Assumptions:
%   The external controller is used in direct MATLAB-call mode.

if nargin < 2 || isempty(params)
    params = uav.sim.default_params_quad_x250();
end

info = uav.atc.ensure_atc_controller_on_path(cfg);

P = feval(cfg.controller_param_factory);
P = local_apply_param_overrides(P, cfg, params);

input_template = feval(cfg.controller_input_factory);
mix_params = feval(cfg.controller_mixer_factory, P);

if isstruct(mix_params) && isfield(mix_params, 'order_map')
    mix_params.order_map = uint8(cfg.sim_motor_order_map(:));
end

ctx = struct();
ctx.repo = info;
ctx.P = P;
ctx.input_template = input_template;
ctx.mix_params = mix_params;
ctx.hover_thrust_norm = double(cfg.hover_thrust_norm);
end

function P = local_apply_param_overrides(P, cfg, params)
%LOCAL_APPLY_PARAM_OVERRIDES Align ATC params with universal-mr-uav.
% Description:
%   Applies only the minimal overrides needed to keep the external
%   controller consistent with the quad-X ordering and hover thrust level
%   of the universal-mr-uav plant.
%
% Inputs:
%   P      - external ATC parameter struct
%   cfg    - ATC bridge configuration struct
%   params - universal-mr-uav plant parameter struct
%
% Outputs:
%   P - overridden ATC parameter struct
%
% Units:
%   Mixed: ArduPilot-style units inside `P`, SI in `params`
%
% Assumptions:
%   Only quad-X and four motors are supported at this stage.

if ~isstruct(P) || ~isscalar(P)
    error('uav:atc:make_controller_context:ParamType', ...
        'Expected external ATC parameters to be a scalar struct.');
end

P.MOT_THST_HOVER = single(cfg.hover_thrust_norm);
P.sim_motor_order_map = uint8(cfg.sim_motor_order_map(:));

if isfield(P, 'MOT_BAT_VOLT_MAX') && P.MOT_BAT_VOLT_MAX == 0.0
    P.MOT_BAT_VOLT_MAX = single(max(cfg.battery_voltage_v, 0.0));
end

if isfield(P, 'MOT_BAT_VOLT_MIN') && P.MOT_BAT_VOLT_MIN == 0.0
    P.MOT_BAT_VOLT_MIN = single(max(0.0, 0.8 * cfg.battery_voltage_v));
end

if isfield(P, 'MOT_THST_MIN')
    P.MOT_THST_MIN = single(0.0);
end

if isfield(P, 'MOT_THST_MAX')
    P.MOT_THST_MAX = single(1.0);
end

if ~isfield(params, 'motor') || ~isfield(params.motor, 'omega_max_radps')
    error('uav:atc:make_controller_context:MissingPlantMotorLimit', ...
        'Expected params.motor.omega_max_radps to be present.');
end
end
