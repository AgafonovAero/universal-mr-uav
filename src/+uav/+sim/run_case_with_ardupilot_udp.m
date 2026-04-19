function log = run_case_with_ardupilot_udp(case_cfg)
%RUN_CASE_WITH_ARDUPILOT_UDP Выполнить расчетный сценарий обмена по UDP.
% Назначение:
%   Использует математическую модель движения, подсистему датчиков,
%   алгоритм оценивания состояния и транспортный уровень `UDP` для
%   пошаговой попытки обмена данными с `ArduPilot JSON SITL`.
%   При отсутствии ответа от `ArduPilot` функция не подменяет внешний
%   комплекс синтетическими командами и завершает прогон с явным статусом.
%
% Входы:
%   case_cfg - структура с полями `params`, `state0`, `dt_s`, `t_final_s`
%              и необязательным полем `ardupilot_cfg`
%
% Выходы:
%   log - структура с историями состояния, датчиков, алгоритма
%         оценивания, пакетов данных, строк JSON, выходов SITL,
%         команд частоты вращения винтов, норм кватернионов и статусов
%         обмена
%
% Единицы измерения:
%   используются единицы СИ; земная система координат - `NED`, связанная
%   система координат - `FRD`
%
% Допущения:
%   Текущий этап предназначен для первого воспроизводимого обмена
%   данными, а не для подтверждения устойчивого автоматического полета.

case_cfg = local_validate_case_cfg(case_cfg);
n_steps = round(case_cfg.t_final_s / case_cfg.dt_s) + 1;
time_s = (0:(n_steps - 1)).' .* case_cfg.dt_s;

state_hist = repmat(case_cfg.state0, n_steps, 1);
sensor_hist = repmat(local_empty_sensor_sample(), n_steps, 1);
estimator_hist = repmat(local_empty_estimator_sample(), n_steps, 1);
quat_norm_true = zeros(n_steps, 1);
quat_norm_est = zeros(n_steps, 1);
motor_cmd_hist_radps = nan(n_steps, case_cfg.ardupilot_cfg.motor_count);
json_text_hist = strings(n_steps, 1);
exchange_status = strings(n_steps, 1);

ardupilot_packet_hist = struct([]);
sitl_output_hist = struct([]);

transport = uav.ardupilot.json_udp_open(case_cfg.ardupilot_cfg);
transport_message_open = string(transport.message);

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
        state_k, ...
        sens_k, ...
        est_k, ...
        time_s(k), ...
        case_cfg.params, ...
        case_cfg.ardupilot_cfg);
    json_text_k = uav.ardupilot.encode_json_fdm_text(ardupilot_packet_k);

    [transport, sitl_output_k, udp_diag_k] = uav.ardupilot.json_udp_step( ...
        transport, json_text_k, case_cfg.ardupilot_cfg);

    if sitl_output_k.valid
        motor_cmd_k_radps = uav.ardupilot.pwm_to_motor_radps( ...
            sitl_output_k.motor_pwm_us, ...
            case_cfg.params, ...
            case_cfg.ardupilot_cfg);
    else
        motor_cmd_k_radps = nan(case_cfg.ardupilot_cfg.motor_count, 1);
    end

    if k == 1
        ardupilot_packet_hist = repmat(ardupilot_packet_k, n_steps, 1);
        sitl_output_hist = repmat(sitl_output_k, n_steps, 1);
    end

    sensor_hist(k) = sens_k;
    estimator_hist(k) = est_k;
    ardupilot_packet_hist(k) = ardupilot_packet_k;
    sitl_output_hist(k) = sitl_output_k;
    json_text_hist(k) = json_text_k;
    exchange_status(k) = local_exchange_status_text( ...
        transport, ...
        sitl_output_k, ...
        udp_diag_k);
    quat_norm_true(k) = snapshot.quat_norm;
    quat_norm_est(k) = norm(est_k.q_nb);
    motor_cmd_hist_radps(k, :) = motor_cmd_k_radps(:).';

    if k < n_steps
        if sitl_output_k.valid
            [state_hist(k + 1), ~] = uav.sim.plant_step_struct( ...
                state_k, ...
                motor_cmd_k_radps, ...
                case_cfg.dt_s, ...
                case_cfg.params);
        else
            state_hist(k + 1) = state_k;
        end
    end

    est_prev = est_k;
end

transport = uav.ardupilot.json_udp_close(transport);

log = struct();
log.time_s = time_s;
log.state = state_hist;
log.sensors = sensor_hist;
log.estimator = estimator_hist;
log.ardupilot_packet = ardupilot_packet_hist;
log.json_text = json_text_hist;
log.sitl_output = sitl_output_hist;
log.motor_cmd_radps = motor_cmd_hist_radps;
log.quat_norm_true = quat_norm_true;
log.quat_norm_est = quat_norm_est;
log.exchange_status = exchange_status;
log.ardupilot_cfg = case_cfg.ardupilot_cfg;
log.transport_message_open = transport_message_open;
log.transport_message_close = string(transport.message);
end

function case_cfg = local_validate_case_cfg(case_cfg)
%LOCAL_VALIDATE_CASE_CFG Проверить конфигурацию сценария моделирования.

if ~isstruct(case_cfg) || ~isscalar(case_cfg)
    error('uav:sim:run_case_with_ardupilot_udp:CaseCfgType', ...
        'Ожидалась скалярная структура case_cfg.');
end

required_fields = {'params', 'state0', 'dt_s', 't_final_s'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(case_cfg, field_name)
        error('uav:sim:run_case_with_ardupilot_udp:MissingField', ...
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
    error('uav:sim:run_case_with_ardupilot_udp:TimeGrid', ...
        'Ожидалось, что t_final_s является целым кратным dt_s.');
end

case_cfg.state0 = uav.core.state_validate(case_cfg.state0);
case_cfg.ardupilot_cfg = local_validate_ardupilot_cfg(case_cfg.ardupilot_cfg);
end

function cfg = local_validate_ardupilot_cfg(cfg)
%LOCAL_VALIDATE_ARDUPILOT_CFG Проверить конфигурацию средства сопряжения.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:sim:run_case_with_ardupilot_udp:ArduPilotCfgType', ...
        'Ожидалась скалярная структура case_cfg.ardupilot_cfg.');
end

validateattributes(cfg.update_rate_hz, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'case_cfg.ardupilot_cfg.update_rate_hz');
validateattributes(cfg.motor_count, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'integer', 'positive'}, ...
    mfilename, 'case_cfg.ardupilot_cfg.motor_count');

expected_dt_s = 1.0 / cfg.update_rate_hz;
validateattributes(expected_dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'expected_dt_s');
end

function text_value = local_exchange_status_text(transport, sitl_output, udp_diag)
%LOCAL_EXCHANGE_STATUS_TEXT Построить строку статуса шага обмена.

if sitl_output.valid
    text_value = "Принят двоичный пакет ArduPilot; " + udp_diag.tx_message;
elseif transport.is_open
    text_value = "Пакет от ArduPilot не получен; " + udp_diag.tx_message;
else
    text_value = "Средство UDP не открыто; " + transport.message;
end
end

function snapshot = local_snapshot_diag(state, params)
%LOCAL_SNAPSHOT_DIAG Сформировать диагностический снимок состояния.

fm = uav.core.forces_moments_sum(state.omega_m_radps, params);

snapshot = struct();
snapshot.forces_b_N = fm.forces_b_N;
snapshot.moments_b_Nm = fm.moments_b_Nm;
snapshot.quat_norm = norm(state.q_nb);
end

function sens = local_empty_sensor_sample()
%LOCAL_EMPTY_SENSOR_SAMPLE Построить пустой отсчет подсистемы датчиков.

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
%LOCAL_EMPTY_ESTIMATOR_SAMPLE Построить пустой отсчет оценивания состояния.

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
