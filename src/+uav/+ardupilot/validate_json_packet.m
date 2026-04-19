function packet = validate_json_packet(packet)
%VALIDATE_JSON_PACKET Validate the canonical ArduPilot JSON FDM packet.
% Description:
%   Checks that a packet prepared for future ArduPilot JSON transport
%   contains the mandatory fields, expected vector sizes, and finite
%   numeric values.
%
% Inputs:
%   packet - scalar FDM packet struct
%
% Outputs:
%   packet - validated packet with normalized column-vector fields
%
% Units:
%   SI only, frames are NED and FRD as documented in packet.frames
%
% Assumptions:
%   The packet uses the TASK-10 canonical field names and scalar-first
%   quaternion convention.

if ~isstruct(packet) || ~isscalar(packet)
    error('uav:ardupilot:validate_json_packet:PacketType', ...
        'Expected packet to be a scalar struct.');
end

required_fields = { ...
    'time_s', 'frames', 'position_ned_m', 'velocity_ned_mps', 'q_nb', ...
    'w_b_radps', 'imu', 'baro', 'mag', 'gnss'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(packet, field_name)
        error('uav:ardupilot:validate_json_packet:MissingField', ...
            'Expected packet.%s to be present.', field_name);
    end
end

validateattributes(packet.time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'packet.time_s');
packet.position_ned_m = local_validate_vec( ...
    packet, 'position_ned_m', 3, 'packet.position_ned_m');
packet.velocity_ned_mps = local_validate_vec( ...
    packet, 'velocity_ned_mps', 3, 'packet.velocity_ned_mps');
packet.q_nb = local_validate_vec(packet, 'q_nb', 4, 'packet.q_nb');
packet.w_b_radps = local_validate_vec( ...
    packet, 'w_b_radps', 3, 'packet.w_b_radps');

packet.imu.accel_b_mps2 = local_validate_vec( ...
    packet.imu, 'accel_b_mps2', 3, 'packet.imu.accel_b_mps2');
packet.imu.gyro_b_radps = local_validate_vec( ...
    packet.imu, 'gyro_b_radps', 3, 'packet.imu.gyro_b_radps');
packet.baro.alt_m = local_validate_scalar( ...
    packet.baro, 'alt_m', 'packet.baro.alt_m');
packet.baro.pressure_pa = local_validate_scalar( ...
    packet.baro, 'pressure_pa', 'packet.baro.pressure_pa');
packet.mag.field_b_uT = local_validate_vec( ...
    packet.mag, 'field_b_uT', 3, 'packet.mag.field_b_uT');
packet.gnss.pos_ned_m = local_validate_vec( ...
    packet.gnss, 'pos_ned_m', 3, 'packet.gnss.pos_ned_m');
packet.gnss.vel_ned_mps = local_validate_vec( ...
    packet.gnss, 'vel_ned_mps', 3, 'packet.gnss.vel_ned_mps');

if ~isstruct(packet.frames) || ~isscalar(packet.frames)
    error('uav:ardupilot:validate_json_packet:FramesType', ...
        'Expected packet.frames to be a scalar struct.');
end

required_frame_fields = {'earth', 'body', 'quaternion'};
for k = 1:numel(required_frame_fields)
    field_name = required_frame_fields{k};
    if ~isfield(packet.frames, field_name)
        error('uav:ardupilot:validate_json_packet:MissingFrameField', ...
            'Expected packet.frames.%s to be present.', field_name);
    end
end

if isfield(packet, 'estimator') && ~isempty(packet.estimator)
    packet.estimator.q_nb = local_validate_vec( ...
        packet.estimator, 'q_nb', 4, 'packet.estimator.q_nb');
    packet.estimator.euler_rpy_rad = local_validate_vec( ...
        packet.estimator, 'euler_rpy_rad', 3, ...
        'packet.estimator.euler_rpy_rad');
    packet.estimator.alt_m = local_validate_scalar( ...
        packet.estimator, 'alt_m', 'packet.estimator.alt_m');
    packet.estimator.vz_mps = local_validate_scalar( ...
        packet.estimator, 'vz_mps', 'packet.estimator.vz_mps');
end
end

function value = local_validate_scalar(data, field_name, label_name)
%LOCAL_VALIDATE_SCALAR Validate one scalar numeric field.

if ~isstruct(data) || ~isfield(data, field_name)
    error('uav:ardupilot:validate_json_packet:MissingScalarField', ...
        'Expected %s to be present.', label_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, label_name);
end

function vec = local_validate_vec(data, field_name, expected_len, label_name)
%LOCAL_VALIDATE_VEC Validate one fixed-length numeric vector.

if ~isstruct(data) || ~isfield(data, field_name)
    error('uav:ardupilot:validate_json_packet:MissingVectorField', ...
        'Expected %s to be present.', label_name);
end

vec = data.(field_name);
validateattributes(vec, {'numeric'}, ...
    {'real', 'finite', 'numel', expected_len}, ...
    mfilename, label_name);
vec = vec(:);
end
