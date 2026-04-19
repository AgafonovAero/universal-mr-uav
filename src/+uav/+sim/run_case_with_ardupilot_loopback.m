function log = run_case_with_ardupilot_loopback(case_cfg)
%RUN_CASE_WITH_ARDUPILOT_LOOPBACK Выполнить проверочный прогон с ArduPilot.
% Description:
%   Использует существующие математическую модель движения, подсистему
%   датчиков и алгоритм оценивания состояния, формирует на каждом шаге
%   пакет данных для будущего `ArduPilot`, подставляет тестовый пакет
%   команд ШИМ вместо реального внешнего комплекса, преобразует ШИМ в
%   частоты вращения винтов и передает их объекту управления.
%
% Inputs:
%   case_cfg - struct with params, state0, dt_s, t_final_s, loopback_mode,
%              and optional ardupilot_cfg
%
% Outputs:
%   log - struct with state, sensors, estimator, packet, servo, and motor
%         histories together with quaternion norms
%
% Units:
%   SI only, frames follow NED for Earth and FRD for body vectors
%
% Assumptions:
%   В TASK-10 не используется реальный UDP-обмен или JSON-транспорт;
%   тестовый пакет применяется только для первичной проверки
%   работоспособности границы сопряжения.

case_cfg = local_validate_case_cfg(case_cfg);
n_steps = round(case_cfg.t_final_s / case_cfg.dt_s) + 1;
time_s = (0:(n_steps - 1)).' .* case_cfg.dt_s;

state_hist = repmat(case_cfg.state0, n_steps, 1);
sensor_hist = repmat(local_empty_sensor_sample(), n_steps, 1);
estimator_hist = repmat(local_empty_estimator_sample(), n_steps, 1);
quat_norm_true = zeros(n_steps, 1);
quat_norm_est = zeros(n_steps, 1);
motor_cmd_hist_radps = zeros(n_steps, case_cfg.ardupilot_cfg.motor_count);

ardupilot_packet_hist = struct([]);
servo_packet_hist = struct([]);
servo_hist = struct([]);

est_prev = local_empty_estimator_sample();
is_initialized = false;

for k = 1:n_steps
    state_k = uav.core.state_validate(state_hist(k));
    snapshot = local_snapshot_diag(state_k, case_cfg.params);
    sens_k = uav.sensors.sensors_step(state_k, snapshot, case_cfg.params);

    if ~is_initialized
        est_prev = uav.est.estimator_init(case_cfg.params, sens_k);
        dt_est_s = 0.0;
        is_initialized = true;
    else
        dt_est_s = case_cfg.dt_s;
    end

    [est_k, ~] = uav.est.estimator_step( ...
        est_prev, sens_k, dt_est_s, case_cfg.params);

    ardupilot_packet_k = uav.ardupilot.pack_json_fdm_packet( ...
        state_k, sens_k, est_k, time_s(k), case_cfg.params, ...
        case_cfg.ardupilot_cfg);
    servo_packet_k = uav.ardupilot.make_loopback_servo_packet( ...
        case_cfg.loopback_mode, time_s(k), case_cfg.ardupilot_cfg);
    servo_k = uav.ardupilot.unpack_servo_outputs( ...
        servo_packet_k, case_cfg.ardupilot_cfg);

    if ~servo_k.valid
        error('uav:sim:run_case_with_ardupilot_loopback:ServoPacket', ...
            'Пакет команд ШИМ для проверочного прогона некорректен: %s', servo_k.message);
    end

    motor_cmd_k_radps = uav.ardupilot.pwm_to_motor_radps( ...
        servo_k.motor_pwm_us, case_cfg.params, case_cfg.ardupilot_cfg);

    if k == 1
        ardupilot_packet_hist = repmat(ardupilot_packet_k, n_steps, 1);
        servo_packet_hist = repmat(servo_packet_k, n_steps, 1);
        servo_hist = repmat(servo_k, n_steps, 1);
    end

    sensor_hist(k) = sens_k;
    estimator_hist(k) = est_k;
    ardupilot_packet_hist(k) = ardupilot_packet_k;
    servo_packet_hist(k) = servo_packet_k;
    servo_hist(k) = servo_k;
    quat_norm_true(k) = snapshot.quat_norm;
    quat_norm_est(k) = norm(est_k.q_nb);
    motor_cmd_hist_radps(k, :) = motor_cmd_k_radps(:).';

    if k < n_steps
        [state_hist(k + 1), ~] = uav.sim.plant_step_struct( ...
            state_k, motor_cmd_k_radps, case_cfg.dt_s, case_cfg.params);
    end

    est_prev = est_k;
end

log = struct();
log.time_s = time_s;
log.state = state_hist;
log.sensors = sensor_hist;
log.estimator = estimator_hist;
log.ardupilot_packet = ardupilot_packet_hist;
log.servo_packet = servo_packet_hist;
log.servo = servo_hist;
log.motor_cmd_radps = motor_cmd_hist_radps;
log.quat_norm_true = quat_norm_true;
log.quat_norm_est = quat_norm_est;
log.loopback_mode = case_cfg.loopback_mode;
log.ardupilot_cfg = case_cfg.ardupilot_cfg;
end

function case_cfg = local_validate_case_cfg(case_cfg)
%LOCAL_VALIDATE_CASE_CFG Проверить конфигурацию сценария моделирования.

if ~isstruct(case_cfg) || ~isscalar(case_cfg)
    error('uav:sim:run_case_with_ardupilot_loopback:CaseCfgType', ...
        'Ожидалась скалярная структура case_cfg.');
end

required_fields = {'params', 'state0', 'dt_s', 't_final_s', 'loopback_mode'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(case_cfg, field_name)
        error('uav:sim:run_case_with_ardupilot_loopback:MissingField', ...
            'Ожидалось наличие поля case_cfg.%s.', field_name);
    end
end

if ~isfield(case_cfg, 'ardupilot_cfg') || isempty(case_cfg.ardupilot_cfg)
    case_cfg.ardupilot_cfg = uav.ardupilot.default_json_config();
end

validateattributes(case_cfg.dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'case_cfg.dt_s');
validateattributes(case_cfg.t_final_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'case_cfg.t_final_s');

n_intervals = round(case_cfg.t_final_s / case_cfg.dt_s);
if abs(n_intervals * case_cfg.dt_s - case_cfg.t_final_s) > 1.0e-12
    error('uav:sim:run_case_with_ardupilot_loopback:TimeGrid', ...
        'Ожидалось, что t_final_s является целым кратным dt_s.');
end

case_cfg.state0 = uav.core.state_validate(case_cfg.state0);
case_cfg.ardupilot_cfg = local_validate_ardupilot_cfg( ...
    case_cfg.ardupilot_cfg);
case_cfg.loopback_mode = local_normalize_mode_name(case_cfg.loopback_mode);

expected_dt_s = 1.0 / case_cfg.ardupilot_cfg.update_rate_hz;
if abs(expected_dt_s - case_cfg.dt_s) > 1.0e-12
    error('uav:sim:run_case_with_ardupilot_loopback:RateMismatch', ...
        'Ожидалось, что case_cfg.dt_s совпадает с 1/cfg.update_rate_hz.');
end
end

function cfg = local_validate_ardupilot_cfg(cfg)
%LOCAL_VALIDATE_ARDUPILOT_CFG Проверить конфигурацию средства сопряжения.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:sim:run_case_with_ardupilot_loopback:ArduPilotCfgType', ...
        'Ожидалась скалярная структура case_cfg.ardupilot_cfg.');
end

cfg = local_merge_default_cfg(cfg);

validateattributes(cfg.update_rate_hz, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'case_cfg.ardupilot_cfg.update_rate_hz');
validateattributes(cfg.motor_count, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'integer', 'positive'}, ...
    mfilename, 'case_cfg.ardupilot_cfg.motor_count');
validateattributes(cfg.motor_order, {'numeric'}, ...
    {'real', 'finite', 'numel', cfg.motor_count}, ...
    mfilename, 'case_cfg.ardupilot_cfg.motor_order');
validateattributes(cfg.pwm_min_us, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, ...
    mfilename, 'case_cfg.ardupilot_cfg.pwm_min_us');
validateattributes(cfg.pwm_max_us, {'numeric'}, ...
    {'real', 'scalar', 'finite'}, ...
    mfilename, 'case_cfg.ardupilot_cfg.pwm_max_us');
end

function cfg = local_merge_default_cfg(cfg_override)
%LOCAL_MERGE_DEFAULT_CFG Объединить пользовательскую и типовую конфигурацию.

cfg = uav.ardupilot.default_json_config();
override_fields = fieldnames(cfg_override);

for k = 1:numel(override_fields)
    field_name = override_fields{k};
    cfg.(field_name) = cfg_override.(field_name);
end
end

function mode_name = local_normalize_mode_name(mode_value)
%LOCAL_NORMALIZE_MODE_NAME Нормализовать обозначение режима прогона.

if isstring(mode_value) && isscalar(mode_value)
    mode_name = lower(strtrim(mode_value));
elseif ischar(mode_value)
    mode_name = lower(strtrim(string(mode_value)));
else
    error('uav:sim:run_case_with_ardupilot_loopback:ModeType', ...
        'Ожидалась строка или символьный вектор в case_cfg.loopback_mode.');
end

if mode_name ~= "hover" && mode_name ~= "yaw_step"
    error('uav:sim:run_case_with_ardupilot_loopback:UnsupportedMode', ...
        'Неподдерживаемый режим проверочного прогона "%s".', mode_name);
end
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Сформировать диагностический снимок состояния объекта.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function sens = local_empty_sensor_sample()
%LOCAL_EMPTY_SENSOR_SAMPLE Сформировать пустой отсчет подсистемы датчиков.

sens = struct();
sens.imu = struct( ...
    'accel_b_mps2', zeros(3, 1), ...
    'gyro_b_radps', zeros(3, 1));
sens.baro = struct( ...
    'alt_m', 0.0, ...
    'pressure_pa', 0.0);
sens.mag = struct( ...
    'field_b_uT', zeros(3, 1));
sens.gnss = struct( ...
    'pos_ned_m', zeros(3, 1), ...
    'vel_ned_mps', zeros(3, 1));
end

function est = local_empty_estimator_sample()
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Сформировать пустой отсчет алгоритма оценивания.

est = struct();
est.attitude = struct( ...
    'q_nb', [1.0; 0.0; 0.0; 0.0], ...
    'euler_rpy_rad', zeros(3, 1));
est.altitude = struct( ...
    'alt_m', 0.0, ...
    'vz_mps', 0.0);
est.q_nb = est.attitude.q_nb;
est.euler_rpy_rad = est.attitude.euler_rpy_rad;
est.alt_m = est.altitude.alt_m;
est.vz_mps = est.altitude.vz_mps;
end
