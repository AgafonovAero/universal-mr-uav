function baro = baro_measure(state, params)
%BARO_MEASURE Build one barometric measurement sample.
% Description:
%   Returns barometric altitude and static pressure using the NED position
%   state and a minimal ISA pressure model.
%
% Inputs:
%   state  - canonical plant state struct
%   params - parameter struct with barometer bias/noise settings
%
% Outputs:
%   baro - struct with alt_m and pressure_pa
%
% Units:
%   alt_m [m], pressure_pa [Pa]
%
% Assumptions:
%   Pressure is derived from the measured barometric altitude and the ISA
%   troposphere model.

state = uav.core.state_validate(state);

alt_noise_m = local_scalar_noise(params.sensors.baro.alt_noise_std_m);

baro = struct();
baro.alt_m = -state.p_ned_m(3) + params.sensors.baro.alt_bias_m + alt_noise_m;
baro.pressure_pa = uav.env.isa_pressure_pa(baro.alt_m);
end

function noise = local_scalar_noise(std_value)
%LOCAL_SCALAR_NOISE Draw a zero-mean scalar white-noise sample.

validateattributes(std_value, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, 'std_value');

if std_value == 0.0
    noise = 0.0;
else
    noise = std_value * randn();
end
end
