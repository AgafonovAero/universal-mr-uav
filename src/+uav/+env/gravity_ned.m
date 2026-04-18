function g_n_mps2 = gravity_ned(g0_mps2)
%GRAVITY_NED Return gravity vector in the NED frame.
% Description:
%   Returns the gravity acceleration vector using the NED convention where
%   positive Z points down.
%
% Inputs:
%   g0_mps2 - optional gravity magnitude [m/s^2]
%
% Outputs:
%   g_n_mps2 - 3x1 gravity vector in NED [m/s^2]
%
% Units:
%   m/s^2
%
% Assumptions:
%   Gravity is uniform and constant over the operating region.

if nargin < 1 || isempty(g0_mps2)
    g0_mps2 = 9.81;
end

g_n_mps2 = [0.0; 0.0; g0_mps2];
end
