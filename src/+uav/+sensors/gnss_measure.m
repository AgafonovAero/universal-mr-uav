function gnss = gnss_measure(state, params)
%GNSS_MEASURE Build one GNSS position and velocity sample in NED.
% Description:
%   Returns GNSS-like position and velocity measurements in the Earth NED
%   frame. Body-frame linear velocity is rotated into NED through q_nb.
%
% Inputs:
%   state  - canonical plant state struct
%   params - parameter struct with GNSS bias/noise settings
%
% Outputs:
%   gnss - struct with pos_ned_m and vel_ned_mps
%
% Units:
%   pos_ned_m [m], vel_ned_mps [m/s]
%
% Assumptions:
%   q_nb rotates body-frame vectors into the NED frame.

state = uav.core.state_validate(state);

c_nb = uav.core.quat_to_dcm(state.q_nb);
vel_ned_mps = c_nb * state.v_b_mps;

gnss = struct();
gnss.pos_ned_m = state.p_ned_m + ...
    params.sensors.gnss.pos_bias_ned_m(:) + ...
    local_white_noise(params.sensors.gnss.pos_noise_std_ned_m(:));
gnss.vel_ned_mps = vel_ned_mps + ...
    params.sensors.gnss.vel_bias_ned_mps(:) + ...
    local_white_noise(params.sensors.gnss.vel_noise_std_ned_mps(:));
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
