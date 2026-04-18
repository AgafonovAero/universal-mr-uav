function [omega_next_radps, diag] = motor_esc_step(omega_prev_radps, omega_cmd_radps, dt_s, motor_params)
%MOTOR_ESC_STEP Minimal first-order ESC and motor speed update.
% Description:
%   Updates rotor angular speeds with a discrete first-order lag, command
%   saturation, and output saturation.
%
% Inputs:
%   omega_prev_radps - previous rotor angular speeds [rad/s]
%   omega_cmd_radps  - commanded rotor angular speeds [rad/s]
%   dt_s             - time step [s]
%   motor_params     - struct with tau_s, omega_min_radps, omega_max_radps
%
% Outputs:
%   omega_next_radps - updated rotor angular speeds [rad/s]
%   diag             - struct with saturated command and derivative terms
%
% Units:
%   SI only
%
% Assumptions:
%   Rotor commands are omega references and the same lag law is used for
%   all four motors.

omega_prev_radps = omega_prev_radps(:);
omega_cmd_radps = omega_cmd_radps(:);

if numel(omega_prev_radps) ~= 4 || numel(omega_cmd_radps) ~= 4
    error('uav:vmg:motor_esc_step:SizeMismatch', ...
        'Expected 4x1 motor speed vectors.');
end

tau_s = motor_params.tau_s;
omega_min_radps = motor_params.omega_min_radps;
omega_max_radps = motor_params.omega_max_radps;

if tau_s <= 0.0
    error('uav:vmg:motor_esc_step:InvalidTau', ...
        'motor_params.tau_s must be positive.');
end

omega_cmd_sat_radps = min(max(omega_cmd_radps, omega_min_radps), omega_max_radps);
domega_radps2 = (omega_cmd_sat_radps - omega_prev_radps) ./ tau_s;

if dt_s > 0.0
    omega_next_radps = omega_prev_radps + dt_s .* domega_radps2;
else
    omega_next_radps = omega_prev_radps;
end

omega_next_radps = min(max(omega_next_radps, omega_min_radps), omega_max_radps);

diag = struct();
diag.omega_cmd_sat_radps = omega_cmd_sat_radps;
diag.domega_radps2 = domega_radps2;
diag.omega_error_radps = omega_cmd_sat_radps - omega_prev_radps;
end
