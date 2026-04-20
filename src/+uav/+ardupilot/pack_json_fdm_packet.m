function packet = pack_json_fdm_packet(state, sensors, estimator, time_s, params, cfg)
%PACK_JSON_FDM_PACKET Сформировать канонический пакет данных для ArduPilot.
% Назначение:
%   Преобразует снимки состояния объекта управления, подсистемы датчиков и
%   алгоритма оценивания состояния в явную структуру MATLAB, фиксирующую
%   будущую границу JSON-обмена. Функция не выполняет сериализацию JSON и
%   не осуществляет отправку по UDP.
%
% Входы:
%   state     - каноническая структура состояния объекта управления
%   sensors   - текущий снимок измерений подсистемы датчиков
%   estimator - текущий снимок алгоритма оценивания состояния
%   time_s    - время моделирования [s]
%   params    - структура параметров летательного аппарата
%   cfg       - структура конфигурации JSON-сопряжения с ArduPilot
%
% Выходы:
%   packet - скалярная структура с каналами состояния и измерений
%
% Единицы измерения:
%   используются единицы СИ; земная система координат - `NED`, связанная
%   система координат - `FRD`
%
% Допущения:
%   `q_nb` задается в скалярно-первой форме и переводит векторы из
%   связанной системы координат в земную систему координат `NED`.
%   Скорость в `NED` и скорость в связанной системе координат сохраняются
%   раздельно.

if nargin < 6 || isempty(cfg)
    cfg = uav.ardupilot.default_json_config();
end

state = uav.core.state_validate(state);
sensors = local_validate_sensor_snapshot(sensors);
estimator = local_validate_estimator_snapshot(estimator);
cfg = local_validate_cfg(cfg);

validateattributes(time_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, ...
    mfilename, 'time_s');

validateattributes(params.gravity_mps2, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'params.gravity_mps2');

velocity_ned_mps = uav.core.quat_to_dcm(state.q_nb) * state.v_b_mps;
[accel_json_mps2, accel_diag] = uav.ardupilot.convert_imu_accel_for_json( ...
    state, ...
    sensors, ...
    time_s, ...
    params, ...
    cfg);

packet = struct();
packet.time_s = time_s;
packet.frames = struct( ...
    'earth', "NED", ...
    'body', "FRD", ...
    'quaternion', "q_nb body_to_ned scalar_first");
packet.transport = struct( ...
    'udp_local_ip', string(cfg.udp_local_ip), ...
    'udp_local_port', cfg.udp_local_port, ...
    'udp_remote_ip', string(cfg.udp_remote_ip), ...
    'udp_remote_port', cfg.udp_remote_port, ...
    'use_ardupilot_json', logical(cfg.use_ardupilot_json));
packet.frame_type = string(cfg.frame_type);
packet.motor_order = cfg.motor_order(:);
packet.gravity_mps2 = params.gravity_mps2;
packet.position_ned_m = state.p_ned_m;
packet.velocity_ned_mps = velocity_ned_mps;
packet.v_body_mps = state.v_b_mps;
packet.q_nb = state.q_nb;
packet.w_b_radps = state.w_b_radps;
packet.imu = struct( ...
    'accel_b_mps2', accel_json_mps2, ...
    'gyro_b_radps', sensors.imu.gyro_b_radps, ...
    'accel_source_b_mps2', sensors.imu.accel_b_mps2, ...
    'accel_mode', string(accel_diag.mode), ...
    'accel_explanation', string(accel_diag.explanation));
packet.baro = struct( ...
    'alt_m', sensors.baro.alt_m, ...
    'pressure_pa', sensors.baro.pressure_pa);
packet.mag = struct( ...
    'field_b_uT', sensors.mag.field_b_uT);
packet.gnss = struct( ...
    'pos_ned_m', sensors.gnss.pos_ned_m, ...
    'vel_ned_mps', sensors.gnss.vel_ned_mps);
packet.estimator = struct( ...
    'q_nb', estimator.q_nb, ...
    'euler_rpy_rad', estimator.euler_rpy_rad, ...
    'alt_m', estimator.alt_m, ...
    'vz_mps', estimator.vz_mps);
packet.notes = cfg.notes;

packet = uav.ardupilot.validate_json_packet(packet);
end

function sensors = local_validate_sensor_snapshot(sensors)
%LOCAL_VALIDATE_SENSOR_SNAPSHOT Validate the canonical sensor snapshot.

if ~isstruct(sensors) || ~isscalar(sensors)
    error('uav:ardupilot:pack_json_fdm_packet:SensorsType', ...
        'Expected sensors to be a scalar struct.');
end

required_fields = {'imu', 'baro', 'mag', 'gnss'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(sensors, field_name)
        error('uav:ardupilot:pack_json_fdm_packet:MissingSensorsField', ...
            'Expected sensors.%s to be present.', field_name);
    end
end

sensors.imu.accel_b_mps2 = local_validate_vec( ...
    sensors.imu, 'accel_b_mps2', 3, 'sensors.imu.accel_b_mps2');
sensors.imu.gyro_b_radps = local_validate_vec( ...
    sensors.imu, 'gyro_b_radps', 3, 'sensors.imu.gyro_b_radps');
sensors.baro.alt_m = local_validate_scalar( ...
    sensors.baro, 'alt_m', 'sensors.baro.alt_m');
sensors.baro.pressure_pa = local_validate_scalar( ...
    sensors.baro, 'pressure_pa', 'sensors.baro.pressure_pa');
sensors.mag.field_b_uT = local_validate_vec( ...
    sensors.mag, 'field_b_uT', 3, 'sensors.mag.field_b_uT');
sensors.gnss.pos_ned_m = local_validate_vec( ...
    sensors.gnss, 'pos_ned_m', 3, 'sensors.gnss.pos_ned_m');
sensors.gnss.vel_ned_mps = local_validate_vec( ...
    sensors.gnss, 'vel_ned_mps', 3, 'sensors.gnss.vel_ned_mps');
end

function estimator = local_validate_estimator_snapshot(estimator)
%LOCAL_VALIDATE_ESTIMATOR_SNAPSHOT Проверить снимок оценивания состояния.

if ~isstruct(estimator) || ~isscalar(estimator)
    error('uav:ardupilot:pack_json_fdm_packet:EstimatorType', ...
        'Ожидалась скалярная структура estimator.');
end

estimator.q_nb = local_validate_vec( ...
    estimator, 'q_nb', 4, 'estimator.q_nb');
estimator.euler_rpy_rad = local_validate_vec( ...
    estimator, 'euler_rpy_rad', 3, 'estimator.euler_rpy_rad');
estimator.alt_m = local_validate_scalar( ...
    estimator, 'alt_m', 'estimator.alt_m');
estimator.vz_mps = local_validate_scalar( ...
    estimator, 'vz_mps', 'estimator.vz_mps');
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Проверить конфигурацию сопряжения с ArduPilot.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:ardupilot:pack_json_fdm_packet:CfgType', ...
        'Expected cfg to be a scalar struct.');
end

required_fields = { ...
    'udp_local_ip', 'udp_local_port', 'udp_remote_ip', 'udp_remote_port', ...
    'frame_type', 'motor_count', 'motor_order', 'use_ardupilot_json', 'notes'};

for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(cfg, field_name)
        error('uav:ardupilot:pack_json_fdm_packet:MissingCfgField', ...
            'Expected cfg.%s to be present.', field_name);
    end
end

validateattributes(cfg.motor_count, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'integer', 'positive'}, ...
    mfilename, 'cfg.motor_count');
validateattributes(cfg.motor_order, {'numeric'}, ...
    {'real', 'finite', 'numel', cfg.motor_count}, ...
    mfilename, 'cfg.motor_order');
end

function value = local_validate_scalar(data, field_name, label_name)
%LOCAL_VALIDATE_SCALAR Validate one scalar field from a struct.

if ~isfield(data, field_name)
    error('uav:ardupilot:pack_json_fdm_packet:MissingScalarField', ...
        'Expected %s to be present.', label_name);
end

value = data.(field_name);
validateattributes(value, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, mfilename, label_name);
end

function vec = local_validate_vec(data, field_name, expected_len, label_name)
%LOCAL_VALIDATE_VEC Validate one fixed-size numeric vector.

if ~isfield(data, field_name)
    error('uav:ardupilot:pack_json_fdm_packet:MissingVectorField', ...
        'Expected %s to be present.', label_name);
end

vec = data.(field_name);
validateattributes(vec, {'numeric'}, ...
    {'real', 'finite', 'numel', expected_len}, ...
    mfilename, label_name);
vec = vec(:);
end
