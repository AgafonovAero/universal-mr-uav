function mag = mag_measure(state, params)
%MAG_MEASURE Build one magnetometer measurement sample in body axes.
% Description:
%   Rotates the configured Earth magnetic field from NED into the body
%   frame using q_nb and adds the configured bias and white noise.
%
% Inputs:
%   state  - canonical plant state struct
%   params - parameter struct with magnetic field and sensor settings
%
% Outputs:
%   mag - struct with field_b_uT
%
% Units:
%   field_b_uT [uT]
%
% Assumptions:
%   q_nb rotates body-frame vectors into the NED frame.

state = uav.core.state_validate(state);

c_nb = uav.core.quat_to_dcm(state.q_nb);
field_ned_uT = params.env.mag_ned_uT(:);

mag = struct();
mag.field_b_uT = c_nb.' * field_ned_uT + ...
    params.sensors.mag.field_bias_b_uT(:) + ...
    local_white_noise(params.sensors.mag.field_noise_std_b_uT(:));
end

function noise = local_white_noise(std_vec)
%LOCAL_WHITE_NOISE Draw a zero-mean white-noise sample.

validateattributes(std_vec, {'numeric'}, {'real', 'finite', 'numel', 3}, ...
    mfilename, 'std_vec');
std_vec = std_vec(:);

if all(std_vec == 0.0)
    noise = zeros(3, 1);
else
    noise = std_vec .* randn(3, 1);
end
end
