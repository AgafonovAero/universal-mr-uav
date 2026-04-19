function atc_in = pack_sensor_packet_for_atc(sensors, estimator, dt_s, ...
        time_s, cfg, input_template)
%PACK_SENSOR_PACKET_FOR_ATC Map Stage-1.5 data into ATC controller input.
% Description:
%   Packs the existing code-centric sensor and estimator outputs into the
%   fixed-field input struct expected by `FSW_Simulink_wrapper_step`.
%
% Inputs:
%   sensors        - output struct from `uav.sensors.sensors_step`
%   estimator      - output struct from `uav.est.estimator_step`
%   dt_s           - current discrete step [s]
%   time_s         - current controller sample time [s]
%   cfg            - scalar bridge configuration struct
%   input_template - optional external-controller input template struct
%
% Outputs:
%   atc_in - scalar fixed-field input struct for the external controller
%
% Units:
%   SI only at this boundary; angular commands are explicitly converted to
%   centi-degrees or centi-degrees per second for the external controller
%
% Assumptions:
%   XY position and velocity come from GNSS, while attitude and altitude
%   use the existing estimator layer.

if nargin < 6 || isempty(input_template)
    atc_in = local_default_input_template();
else
    atc_in = input_template;
end

cfg = local_validate_cfg(cfg);
sensors = local_validate_sensors(sensors);
estimator = local_validate_estimator(estimator);

validateattributes(dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'dt_s');
validateattributes(time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'time_s');

[att_cmd_cd, rate_cmd_cds, thr_in] = local_command_profile(cfg, time_s);
ground_contact = estimator.alt_m <= cfg.ground_contact_alt_m && ...
    abs(estimator.vz_mps) <= cfg.ground_contact_vz_mps;

pos_ned = single(reshape(sensors.gnss.pos_ned_m, 3, 1));
vel_ned = single(reshape(sensors.gnss.vel_ned_mps, 3, 1));

if logical(cfg.use_estimator_altitude)
    pos_ned(3) = single(-estimator.alt_m);
    vel_ned(3) = single(-estimator.vz_mps);
end

atc_in.dt = single(dt_s);
atc_in.reset = false;
atc_in.armed = logical(time_s >= cfg.arm_time_s);
atc_in.mode_cmd = uint8(cfg.mode_cmd);
atc_in.stick_bf_xy = single([0.0; 0.0]);
atc_in.wp_xy = single([0.0; 0.0]);
atc_in.takeoff_alt = single(cfg.takeoff_alt_m);
atc_in.att_cmd_cd = single(att_cmd_cd(:));
atc_in.rate_cmd_cds = single(rate_cmd_cds(:));
atc_in.thr_in = single(thr_in);
atc_in.slew_yaw = logical(cfg.slew_yaw);
atc_in.pos_ned = pos_ned;
atc_in.vel_ned = vel_ned;
atc_in.q_bn = single(reshape(estimator.q_nb, 1, 4));
atc_in.gyro_rads = single(reshape(sensors.imu.gyro_b_radps, 3, 1));
atc_in.ground_contact = logical(ground_contact);
atc_in.rc_ok = logical(cfg.rc_ok);
atc_in.gps_ok = logical(cfg.gps_ok);
atc_in.of_ok = logical(cfg.of_ok);
atc_in.batt_v = single(cfg.battery_voltage_v);
end

function [att_cmd_cd, rate_cmd_cds, thr_in] = local_command_profile(cfg, time_s)
%LOCAL_COMMAND_PROFILE Build one minimal manual-command profile.
% Description:
%   Generates the direct controller command for the supported hover and
%   yaw-step smoke scenarios.
%
% Inputs:
%   cfg    - bridge configuration struct
%   time_s - current sample time [s]
%
% Outputs:
%   att_cmd_cd   - roll/pitch/yaw angle command [centi-deg]
%   rate_cmd_cds - body-rate command [centi-deg/s]
%   thr_in       - normalized manual throttle [0..1]
%
% Units:
%   centi-deg, centi-deg/s, dimensionless
%
% Assumptions:
%   Stage-08a demos use STABILIZE mode with zero roll/pitch command.

mode_name = local_normalize_mode_name(cfg.mode_name);

att_cmd_cd = 18000.0 / pi .* ...
    [cfg.roll_cmd_rad; cfg.pitch_cmd_rad; 0.0];
rate_cmd_cds = zeros(3, 1);
thr_in = cfg.manual_throttle_norm;

if mode_name == "yaw_step" && time_s >= cfg.yaw_step_time_s
    rate_cmd_cds(3) = 18000.0 / pi * cfg.yaw_rate_step_radps;
end
end

function mode_name = local_normalize_mode_name(mode_value)
%LOCAL_NORMALIZE_MODE_NAME Normalize one supported scenario name.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
elseif ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
else
    error('uav:atc:pack_sensor_packet_for_atc:ModeType', ...
        'Expected cfg.mode_name to be a text scalar.');
end

if ~any(mode_name == ["hover", "yaw_step"])
    error('uav:atc:pack_sensor_packet_for_atc:UnsupportedMode', ...
        'Unsupported cfg.mode_name "%s".', mode_name);
end
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Validate ATC bridge config fields used in packing.

required_fields = {'mode_name', 'mode_cmd', 'arm_time_s', ...
    'yaw_step_time_s', 'yaw_rate_step_radps', 'roll_cmd_rad', ...
    'pitch_cmd_rad', 'takeoff_alt_m', 'slew_yaw', ...
    'manual_throttle_norm', 'use_estimator_altitude', 'rc_ok', ...
    'gps_ok', 'of_ok', 'battery_voltage_v', 'ground_contact_alt_m', ...
    'ground_contact_vz_mps'};

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:atc:pack_sensor_packet_for_atc:CfgType', ...
        'Expected cfg to be a scalar struct.');
end

for k = 1:numel(required_fields)
    if ~isfield(cfg, required_fields{k})
        error('uav:atc:pack_sensor_packet_for_atc:MissingCfgField', ...
            'Expected cfg.%s to be present.', required_fields{k});
    end
end
end

function sensors = local_validate_sensors(sensors)
%LOCAL_VALIDATE_SENSORS Validate the Stage-1.5 sensor sample struct.

if ~isstruct(sensors) || ~isscalar(sensors)
    error('uav:atc:pack_sensor_packet_for_atc:SensorsType', ...
        'Expected sensors to be a scalar struct.');
end

required_fields = {'imu', 'gnss'};
for k = 1:numel(required_fields)
    if ~isfield(sensors, required_fields{k})
        error('uav:atc:pack_sensor_packet_for_atc:MissingSensorsField', ...
            'Expected sensors.%s to be present.', required_fields{k});
    end
end

validateattributes(sensors.imu.gyro_b_radps, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, mfilename, 'sensors.imu.gyro_b_radps');
validateattributes(sensors.gnss.pos_ned_m, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, mfilename, 'sensors.gnss.pos_ned_m');
validateattributes(sensors.gnss.vel_ned_mps, {'numeric'}, ...
    {'real', 'finite', 'numel', 3}, mfilename, 'sensors.gnss.vel_ned_mps');
end

function estimator = local_validate_estimator(estimator)
%LOCAL_VALIDATE_ESTIMATOR Validate the Stage-1.5 estimator output struct.

if ~isstruct(estimator) || ~isscalar(estimator)
    error('uav:atc:pack_sensor_packet_for_atc:EstimatorType', ...
        'Expected estimator to be a scalar struct.');
end

required_fields = {'q_nb', 'alt_m', 'vz_mps'};
for k = 1:numel(required_fields)
    if ~isfield(estimator, required_fields{k})
        error('uav:atc:pack_sensor_packet_for_atc:MissingEstimatorField', ...
            'Expected estimator.%s to be present.', required_fields{k});
    end
end

validateattributes(estimator.q_nb, {'numeric'}, ...
    {'real', 'finite', 'numel', 4}, mfilename, 'estimator.q_nb');
validateattributes(estimator.alt_m, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, 'estimator.alt_m');
validateattributes(estimator.vz_mps, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, 'estimator.vz_mps');
end

function template = local_default_input_template()
%LOCAL_DEFAULT_INPUT_TEMPLATE Create one ATC input template.
% Description:
%   Provides a local fixed-field fallback template so the bridge can build
%   the expected struct shape even before the external helper is loaded.
%
% Inputs:
%   none
%
% Outputs:
%   template - scalar input struct compatible with `FSW_make_default_in`
%
% Units:
%   SI only, except manual angle/rate commands in centi-deg
%
% Assumptions:
%   Field names follow the external repository's published interface.

template = struct();
template.dt = single(0.01);
template.reset = false;
template.armed = false;
template.mode_cmd = uint8(5);
template.stick_bf_xy = single([0; 0]);
template.wp_xy = single([0; 0]);
template.takeoff_alt = single(1.0);
template.att_cmd_cd = single([0; 0; 0]);
template.rate_cmd_cds = single([0; 0; 0]);
template.thr_in = single(0.0);
template.slew_yaw = false;
template.pos_ned = single([0; 0; 0]);
template.vel_ned = single([0; 0; 0]);
template.q_bn = single([1 0 0 0]);
template.gyro_rads = single([0; 0; 0]);
template.ground_contact = true;
template.rc_ok = true;
template.gps_ok = true;
template.of_ok = true;
template.batt_v = single(16.0);
end
