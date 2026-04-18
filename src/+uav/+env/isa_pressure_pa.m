function pressure_pa = isa_pressure_pa(alt_m)
%ISA_PRESSURE_PA Return ISA static pressure for low-altitude sensor use.
% Description:
%   Computes static pressure using a minimal International Standard
%   Atmosphere troposphere model. The implementation is text-first and
%   intended for the Stage-1.5 barometric sensor layer.
%
% Inputs:
%   alt_m - geometric altitude above the NED origin [m]
%
% Outputs:
%   pressure_pa - static pressure [Pa]
%
% Units:
%   SI only
%
% Assumptions:
%   Valid for low-altitude troposphere use in the current demos and tests.

validateattributes(alt_m, {'numeric'}, {'real', 'finite'}, mfilename, 'alt_m');

p0_pa = 101325.0;
t0_k = 288.15;
lapse_kpm = 0.0065;
g0_mps2 = 9.80665;
r_air_jpkgk = 287.05287;

temp_ratio = 1.0 - (lapse_kpm .* alt_m) ./ t0_k;
if any(temp_ratio(:) <= 0.0)
    error('uav:env:isa_pressure_pa:AltitudeRange', ...
        'Expected alt_m to remain within the troposphere model range.');
end

pressure_pa = p0_pa .* temp_ratio .^ (g0_mps2 / (r_air_jpkgk * lapse_kpm));
end
