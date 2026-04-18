function [state_next, diag] = plant_step_struct(state_prev, motor_cmd_radps, dt_s, params)
%PLANT_STEP_STRUCT Advance the plant state using the canonical state struct.
% Description:
%   Wraps uav.sim.plant_step so the external Stage-1.5 API can operate on
%   the canonical state struct instead of a raw packed vector.
%
% Inputs:
%   state_prev      - canonical plant state struct
%   motor_cmd_radps - 4x1 motor speed command [rad/s]
%   dt_s            - discrete time step [s]
%   params          - plant and propulsion parameter struct
%
% Outputs:
%   state_next - canonical state struct after one step
%   diag       - diagnostic struct forwarded from uav.sim.plant_step
%
% Units:
%   SI only
%
% Assumptions:
%   The wrapped plant_step remains the source of truth for propagation.

state_prev = uav.core.state_validate(state_prev);
motor_cmd_radps = motor_cmd_radps(:);

if numel(motor_cmd_radps) ~= 4
    error('uav:sim:plant_step_struct:CommandSize', ...
        'Expected motor_cmd_radps to have 4 elements.');
end

[x_next, diag] = uav.sim.plant_step( ...
    uav.core.state_pack(state_prev), motor_cmd_radps, dt_s, params);
state_next = uav.core.state_unpack(x_next);
end
