function [thrust_N, torque_Nm] = rotor_simple(omega_radps, kT_N_per_radps2, kQ_Nm_per_radps2)
%ROTOR_SIMPLE Simple rotor thrust and drag-torque model.
% Description:
%   Computes scalar thrust and reaction torque magnitudes from rotor angular
%   speed using quadratic coefficients.
%
% Inputs:
%   omega_radps        - rotor angular speed [rad/s]
%   kT_N_per_radps2    - thrust coefficient [N/(rad/s)^2]
%   kQ_Nm_per_radps2   - drag torque coefficient [N*m/(rad/s)^2]
%
% Outputs:
%   thrust_N   - scalar rotor thrust magnitude [N]
%   torque_Nm  - scalar rotor drag torque magnitude [N*m]
%
% Units:
%   SI only
%
% Assumptions:
%   The same quadratic law is used for positive and negative speed inputs.

omega_sq = omega_radps .^ 2;

thrust_N = kT_N_per_radps2 .* omega_sq;
torque_Nm = kQ_Nm_per_radps2 .* omega_sq;
end
