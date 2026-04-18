function imu = imu_measure(state, diag, params)
%IMU_MEASURE Build one IMU measurement sample from plant state and diag.
% Description:
%   Returns a minimal IMU sample with body-frame specific force and body
%   angular rate. The accelerometer output uses the plant force balance and
%   does not add gravity a second time.
%
% Inputs:
%   state  - canonical plant state struct
%   diag   - diagnostic struct with forces_b_N
%   params - parameter struct with mass and IMU bias/noise settings
%
% Outputs:
%   imu - struct with accel_b_mps2 and gyro_b_radps
%
% Units:
%   accel_b_mps2 [m/s^2], gyro_b_radps [rad/s]
%
% Assumptions:
%   Accelerometer output is specific force in the body frame.

state = uav.core.state_validate(state);
forces_b_N = local_validate_vec3(diag, 'forces_b_N');

imu = struct();
imu.accel_b_mps2 = (forces_b_N ./ params.mass_kg) + ...
    params.sensors.imu.accel_bias_b_mps2(:) + ...
    local_white_noise(params.sensors.imu.accel_noise_std_b_mps2(:));
imu.gyro_b_radps = state.w_b_radps + ...
    params.sensors.imu.gyro_bias_b_radps(:) + ...
    local_white_noise(params.sensors.imu.gyro_noise_std_b_radps(:));
end

function vec = local_validate_vec3(data, field_name)
%LOCAL_VALIDATE_VEC3 Validate one 3x1 diagnostic vector.

if ~isstruct(data) || ~isfield(data, field_name)
    error('uav:sensors:imu_measure:MissingField', ...
        'Expected diag.%s to be present.', field_name);
end

vec = data.(field_name);
validateattributes(vec, {'numeric'}, {'real', 'finite', 'numel', 3}, ...
    mfilename, ['diag.' field_name]);
vec = vec(:);
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
