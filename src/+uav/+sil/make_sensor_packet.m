function packet = make_sensor_packet(time_s, sens, valid_flags, packet_prev)
%MAKE_SENSOR_PACKET Pack one canonical external SIL sensor packet.
% Description:
%   Converts the internal sensor-layer snapshot into a canonical external
%   packet with deterministic valid flags and held samples for slower
%   channels.
%
% Inputs:
%   time_s      - current simulation time [s]
%   sens        - scalar sensor snapshot struct from uav.sensors.sensors_step
%   valid_flags - scalar struct with imu_valid, baro_valid, mag_valid,
%                 and gnss_valid flags
%   packet_prev - previous sensor packet for hold behavior (optional)
%
% Outputs:
%   packet - canonical external sensor packet struct
%
% Units:
%   SI only, magnetic field in uT
%
% Assumptions:
%   Slower channels use zero-order hold between scheduler updates.

validateattributes(time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, 'time_s');
sens = local_validate_sensor_snapshot(sens);
valid_flags = local_validate_valid_flags(valid_flags);

if nargin < 4 || isempty(packet_prev)
    packet_prev = local_empty_sensor_packet();
else
    packet_prev = local_validate_sensor_packet(packet_prev);
end

packet = packet_prev;
packet.time_s = time_s;
packet.imu_valid = valid_flags.imu_valid;
packet.baro_valid = valid_flags.baro_valid;
packet.mag_valid = valid_flags.mag_valid;
packet.gnss_valid = valid_flags.gnss_valid;

if packet.imu_valid
    packet.imu = sens.imu;
end

if packet.baro_valid
    packet.baro = sens.baro;
end

if packet.mag_valid
    packet.mag = sens.mag;
end

if packet.gnss_valid
    packet.gnss = sens.gnss;
end
end

function sens = local_validate_sensor_snapshot(sens)
%LOCAL_VALIDATE_SENSOR_SNAPSHOT Validate one internal sensor snapshot.
% Description:
%   Checks that the provided sensor snapshot matches the canonical
%   Stage-1.5 sensor layer format.
%
% Inputs:
%   sens - scalar sensor struct
%
% Outputs:
%   sens - validated sensor struct with column vectors
%
% Units:
%   SI only
%
% Assumptions:
%   The snapshot comes from uav.sensors.sensors_step.

if ~isstruct(sens) || ~isscalar(sens)
    error('uav:sil:make_sensor_packet:SensorType', ...
        'Expected sens to be a scalar struct.');
end

required_fields = {'imu', 'baro', 'mag', 'gnss'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(sens, field_name)
        error('uav:sil:make_sensor_packet:MissingField', ...
            'Expected sens.%s to be present.', field_name);
    end
end

sens.imu.accel_b_mps2 = local_validate_vec3(sens.imu, 'accel_b_mps2');
sens.imu.gyro_b_radps = local_validate_vec3(sens.imu, 'gyro_b_radps');
sens.baro.alt_m = local_validate_scalar(sens.baro, 'alt_m');
sens.baro.pressure_pa = local_validate_scalar(sens.baro, 'pressure_pa');
sens.mag.field_b_uT = local_validate_vec3(sens.mag, 'field_b_uT');
sens.gnss.pos_ned_m = local_validate_vec3(sens.gnss, 'pos_ned_m');
sens.gnss.vel_ned_mps = local_validate_vec3(sens.gnss, 'vel_ned_mps');
end

function valid_flags = local_validate_valid_flags(valid_flags)
%LOCAL_VALIDATE_VALID_FLAGS Validate scheduler valid flags.
% Description:
%   Checks that the scheduler valid-flag struct carries the expected
%   logical fields.
%
% Inputs:
%   valid_flags - scheduler flag struct
%
% Outputs:
%   valid_flags - normalized struct with logical scalar fields
%
% Units:
%   not applicable
%
% Assumptions:
%   The scheduler follows the uav.sil.update_rate_scheduler API.

if ~isstruct(valid_flags) || ~isscalar(valid_flags)
    error('uav:sil:make_sensor_packet:ValidFlagsType', ...
        'Expected valid_flags to be a scalar struct.');
end

flag_fields = {'imu_valid', 'baro_valid', 'mag_valid', 'gnss_valid'};
for k = 1:numel(flag_fields)
    field_name = flag_fields{k};
    if ~isfield(valid_flags, field_name)
        error('uav:sil:make_sensor_packet:MissingValidFlag', ...
            'Expected valid_flags.%s to be present.', field_name);
    end

    validateattributes(valid_flags.(field_name), {'logical', 'numeric'}, ...
        {'real', 'scalar', 'finite'}, ...
        mfilename, ['valid_flags.' field_name]);
    valid_flags.(field_name) = logical(valid_flags.(field_name));
end
end

function packet = local_validate_sensor_packet(packet)
%LOCAL_VALIDATE_SENSOR_PACKET Validate one external sensor packet.
% Description:
%   Reuses the top-level packet schema to ensure held samples remain
%   compatible with the canonical external interface.
%
% Inputs:
%   packet - external sensor packet struct
%
% Outputs:
%   packet - validated packet with normalized vector shapes
%
% Units:
%   SI only
%
% Assumptions:
%   The packet uses the canonical field names.

if ~isstruct(packet) || ~isscalar(packet)
    error('uav:sil:make_sensor_packet:PacketType', ...
        'Expected packet_prev to be a scalar struct.');
end

required_fields = {'time_s', 'imu_valid', 'imu', 'baro_valid', 'baro', ...
    'mag_valid', 'mag', 'gnss_valid', 'gnss'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(packet, field_name)
        error('uav:sil:make_sensor_packet:MissingPacketField', ...
            'Expected packet_prev.%s to be present.', field_name);
    end
end

validateattributes(packet.time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'packet_prev.time_s');

packet = local_validate_sensor_snapshot(packet);
packet.imu_valid = logical(packet.imu_valid);
packet.baro_valid = logical(packet.baro_valid);
packet.mag_valid = logical(packet.mag_valid);
packet.gnss_valid = logical(packet.gnss_valid);
end

function packet = local_empty_sensor_packet()
%LOCAL_EMPTY_SENSOR_PACKET Build one deterministic zero packet.
% Description:
%   Creates an all-zero packet for initialization before the first valid
%   samples are emitted by the scheduler.
%
% Inputs:
%   none
%
% Outputs:
%   packet - canonical sensor packet struct
%
% Units:
%   SI only
%
% Assumptions:
%   Zero values are acceptable only before the first packet update.

packet = struct();
packet.time_s = 0.0;
packet.imu_valid = false;
packet.imu = struct( ...
    'accel_b_mps2', zeros(3, 1), ...
    'gyro_b_radps', zeros(3, 1));
packet.baro_valid = false;
packet.baro = struct( ...
    'alt_m', 0.0, ...
    'pressure_pa', 0.0);
packet.mag_valid = false;
packet.mag = struct( ...
    'field_b_uT', zeros(3, 1));
packet.gnss_valid = false;
packet.gnss = struct( ...
    'pos_ned_m', zeros(3, 1), ...
    'vel_ned_mps', zeros(3, 1));
end

function value = local_validate_scalar(data, field_name)
%LOCAL_VALIDATE_SCALAR Validate one finite scalar field.
% Description:
%   Reads and validates one scalar numeric field from a struct.
%
% Inputs:
%   data       - source struct
%   field_name - scalar field name
%
% Outputs:
%   value - validated scalar value
%
% Units:
%   inherited from the field
%
% Assumptions:
%   The field exists in the provided struct.

if ~isfield(data, field_name)
    error('uav:sil:make_sensor_packet:MissingScalarField', ...
        'Expected field %s to be present.', field_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, field_name);
end

function vec = local_validate_vec3(data, field_name)
%LOCAL_VALIDATE_VEC3 Validate one 3x1 vector field.
% Description:
%   Reads one vector field from a struct and normalizes it to a column.
%
% Inputs:
%   data       - source struct
%   field_name - vector field name
%
% Outputs:
%   vec - validated 3x1 column vector
%
% Units:
%   inherited from the field
%
% Assumptions:
%   The field exists in the provided struct.

if ~isfield(data, field_name)
    error('uav:sil:make_sensor_packet:MissingVectorField', ...
        'Expected field %s to be present.', field_name);
end

vec = data.(field_name);
validateattributes(vec, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, mfilename, field_name);
vec = vec(:);
end
