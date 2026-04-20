%% RUN_ARDUPILOT_JSON_UDP_LIVE_BACKEND Выполнить 20-секундный расчетный обмен с ArduPilot.
% Назначение:
%   Сначала ожидает первый валидный двоичный пакет от `ArduPilot SITL`,
%   затем выполняет 20-секундный расчетный обмен по JSON и UDP между
%   математической моделью движения и внешним комплексом управления.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_json_udp_live_backend - структура результата в базовом
%   рабочем пространстве MATLAB
%
% Допущения:
%   `ArduPilot SITL` уже запущен отдельно и направляет двоичный поток на
%   локальный порт 9002.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
cfg = local_apply_cfg_overrides(cfg);

dt_s = 1.0 / cfg.update_rate_hz;
duration_s = local_read_duration_override(20.0);
max_steps = ceil(duration_s / dt_s) + 5;

state = uav.core.state_unpack(params.demo.initial_state_plant);
state_hist = repmat(state, max_steps, 1);
sensor_hist = repmat(local_empty_sensor_sample(), max_steps, 1);
estimator_hist = repmat(local_empty_estimator_sample(), max_steps, 1);
json_text_hist = strings(max_steps, 1);
sitl_output_hist = repmat(uav.ardupilot.decode_sitl_output_packet([], cfg), max_steps, 1);
exchange_diag_hist = repmat(local_empty_exchange_diag(cfg), max_steps, 1);
exchange_status_hist = strings(max_steps, 1);
motor_cmd_hist_radps = nan(max_steps, cfg.motor_count);
prearm_hold_hist = false(max_steps, 1);
quat_norm_true = nan(max_steps, 1);
quat_norm_est = nan(max_steps, 1);
time_s = nan(max_steps, 1);
loop_elapsed_s = nan(max_steps, 1);
model_step_elapsed_s = nan(max_steps, 1);
json_encode_elapsed_s = nan(max_steps, 1);
udp_step_elapsed_s = nan(max_steps, 1);
udp_read_elapsed_s = nan(max_steps, 1);
udp_write_elapsed_s = nan(max_steps, 1);
udp_rx_datagram_count_hist = zeros(max_steps, 1);
udp_valid_rx_count_hist = zeros(max_steps, 1);
udp_invalid_rx_count_hist = zeros(max_steps, 1);
json_tx_timestamp_s = nan(max_steps, 1);
response_tx_timestamp_s = nan(max_steps, 1);
valid_rx_timestamp_s = nan(max_steps, 1);
json_tx_time_count = 0;
response_tx_time_count = 0;
valid_rx_time_count = 0;

transport = uav.ardupilot.json_udp_open(cfg);
transport_message_open = string(transport.message);

state_est_prev = local_empty_estimator_sample();
is_initialized = false;
step_index = 0;

rx_datagram_count = 0;
valid_rx_count = 0;
invalid_rx_count = 0;
json_tx_count = 0;
response_tx_count = 0;
last_sender_address = "";
last_sender_port = 0;
last_magic = 0;
last_frame_count = 0;
last_pwm_us = nan(cfg.motor_count, 1);
last_motor_cmd_radps = nan(cfg.motor_count, 1);
last_exchange_status = "прием не подтвержден";
first_packet_received = false;
first_packet_magic = 0;
physics_time_s = 0.0;

wait_start = tic;
while toc(wait_start) < cfg.udp_handshake_timeout_s
    [transport, sitl_output_wait, wait_diag] = uav.ardupilot.json_udp_step( ...
        transport, ...
        "", ...
        cfg);
    rx_datagram_count = rx_datagram_count + double(wait_diag.rx_datagram_count);
    valid_rx_count = valid_rx_count + double(wait_diag.rx_valid_count);
    invalid_rx_count = invalid_rx_count + double(wait_diag.rx_invalid_count);

    if strlength(string(wait_diag.last_sender_address)) > 0
        last_sender_address = string(wait_diag.last_sender_address);
        last_sender_port = double(wait_diag.last_sender_port);
    end
    if double(wait_diag.last_magic) > 0
        last_magic = double(wait_diag.last_magic);
        last_frame_count = double(wait_diag.last_frame_count);
    end

    if sitl_output_wait.valid
        first_packet_received = true;
        first_packet_magic = sitl_output_wait.magic;
        break;
    end

    pause(cfg.udp_receive_pause_s);
end

if ~first_packet_received
    transport = uav.ardupilot.json_udp_close(transport);
    result = struct();
    result.cfg = cfg;
    result.transport_message_open = transport_message_open;
    result.transport_message_close = string(transport.message);
    result.first_packet_received = false;
    result.first_packet_magic = 0;
    result.time_s = zeros(0, 1);
    result.loop_elapsed_s = zeros(0, 1);
    result.model_step_elapsed_s = zeros(0, 1);
    result.json_encode_elapsed_s = zeros(0, 1);
    result.udp_step_elapsed_s = zeros(0, 1);
    result.udp_read_elapsed_s = zeros(0, 1);
    result.udp_write_elapsed_s = zeros(0, 1);
    result.udp_rx_datagram_count_hist = zeros(0, 1);
    result.udp_valid_rx_count_hist = zeros(0, 1);
    result.udp_invalid_rx_count_hist = zeros(0, 1);
    result.json_tx_timestamp_s = zeros(0, 1);
    result.response_tx_timestamp_s = zeros(0, 1);
    result.valid_rx_timestamp_s = zeros(0, 1);
    result.rx_datagram_count = rx_datagram_count;
    result.valid_rx_count = valid_rx_count;
    result.invalid_rx_count = invalid_rx_count;
    result.json_tx_count = 0;
    result.response_tx_count = 0;
    result.json_probe_tx_count = 0;
    result.last_sender_address = last_sender_address;
    result.last_sender_port = last_sender_port;
    result.last_magic = last_magic;
    result.last_frame_count = last_frame_count;
    result.last_pwm_us = nan(cfg.motor_count, 1);
    result.last_motor_cmd_radps = nan(cfg.motor_count, 1);
    result.last_exchange_status = "прием не подтвержден";
    result.quat_true_finite = true;
    result.quat_est_finite = true;
    assignin('base', 'ardupilot_json_udp_live_backend', result);

    fprintf('20-секундный расчетный обмен MATLAB-модели с ArduPilot\n');
    fprintf('  первый двоичный пакет не получен; расчетный обмен не начат\n');
    fprintf('  принято UDP-датаграмм                         : %d\n', rx_datagram_count);
    fprintf('  принято валидных двоичных пакетов             : %d\n', valid_rx_count);
    fprintf('  итоговый статус последнего шага               : прием не подтвержден\n');
    return;
end

snapshot = local_snapshot_diag(state, params);
sens_k = uav.sensors.sensors_step(state, snapshot, params);
state_est_prev = uav.est.estimator_init(params, sens_k);
is_initialized = true;

wall_start = tic;
next_tick_s = 0.0;
held_motor_cmd_radps = uav.ardupilot.pwm_to_motor_radps( ...
    sitl_output_wait.motor_pwm_us, ...
    params, ...
    cfg);
last_pwm_us = sitl_output_wait.motor_pwm_us;
last_motor_cmd_radps = held_motor_cmd_radps;

while toc(wall_start) < duration_s
    elapsed_s = toc(wall_start);

    if elapsed_s + 1.0e-9 < next_tick_s
        pause(min(0.001, next_tick_s - elapsed_s));
        continue;
    end

    step_index = step_index + 1;
    time_s(step_index) = elapsed_s;
    loop_tic = tic;

    model_tic = tic;
    prearm_hold_active = local_is_prearm_hold_active(cfg, last_pwm_us);
    prearm_hold_hist(step_index) = prearm_hold_active;
    if ~prearm_hold_active
        [state_next, ~] = uav.sim.plant_step_struct( ...
            state, ...
            held_motor_cmd_radps, ...
            dt_s, ...
            params);
        state = state_next;
    end
    physics_time_s = physics_time_s + dt_s;

    snapshot = local_snapshot_diag(state, params);
    sens_k = uav.sensors.sensors_step(state, snapshot, params);

    if ~is_initialized
        state_est_prev = uav.est.estimator_init(params, sens_k);
        dt_est_s = 0.0;
        is_initialized = true;
    else
        dt_est_s = dt_s;
    end

    [est_k, ~] = uav.est.estimator_step( ...
        state_est_prev, ...
        sens_k, ...
        dt_est_s, ...
        params);

    packet_k = uav.ardupilot.pack_json_fdm_packet( ...
        state, ...
        sens_k, ...
        est_k, ...
        physics_time_s, ...
        params, ...
        cfg);
    model_step_elapsed_s(step_index) = toc(model_tic);

    json_tic = tic;
    json_text_k = uav.ardupilot.encode_json_fdm_text(packet_k);
    json_encode_elapsed_s(step_index) = toc(json_tic);

    [transport, sitl_output_k, udp_diag_k] = uav.ardupilot.json_udp_step( ...
        transport, ...
        json_text_k, ...
        cfg);

    rx_datagram_count = rx_datagram_count + double(udp_diag_k.rx_datagram_count);
    valid_rx_count = valid_rx_count + double(udp_diag_k.rx_valid_count);
    invalid_rx_count = invalid_rx_count + double(udp_diag_k.rx_invalid_count);
    json_tx_count = json_tx_count + double(udp_diag_k.tx_ok);
    response_tx_count = response_tx_count + double(udp_diag_k.tx_ok && udp_diag_k.rx_valid);

    if strlength(string(udp_diag_k.last_sender_address)) > 0
        last_sender_address = string(udp_diag_k.last_sender_address);
    end
    if double(udp_diag_k.last_sender_port) > 0
        last_sender_port = double(udp_diag_k.last_sender_port);
    end
    if double(udp_diag_k.last_magic) > 0
        last_magic = double(udp_diag_k.last_magic);
        last_frame_count = double(udp_diag_k.last_frame_count);
    end

    if sitl_output_k.valid
        held_motor_cmd_radps = uav.ardupilot.pwm_to_motor_radps( ...
            sitl_output_k.motor_pwm_us, ...
            params, ...
            cfg);
        last_pwm_us = sitl_output_k.motor_pwm_us;
        last_motor_cmd_radps = held_motor_cmd_radps;
    end

    last_exchange_status = string(udp_diag_k.status);

    state_hist(step_index) = state;
    sensor_hist(step_index) = sens_k;
    estimator_hist(step_index) = est_k;
    json_text_hist(step_index) = json_text_k;
    sitl_output_hist(step_index) = sitl_output_k;
    exchange_diag_hist(step_index) = udp_diag_k;
    exchange_status_hist(step_index) = string(udp_diag_k.status);
    motor_cmd_hist_radps(step_index, :) = held_motor_cmd_radps(:).';
    quat_norm_true(step_index) = norm(state.q_nb);
    quat_norm_est(step_index) = norm(est_k.q_nb);
    udp_step_elapsed_s(step_index) = double(udp_diag_k.step_elapsed_s);
    udp_read_elapsed_s(step_index) = double(udp_diag_k.rx_elapsed_s);
    udp_write_elapsed_s(step_index) = double(udp_diag_k.tx_elapsed_s);
    udp_rx_datagram_count_hist(step_index) = double(udp_diag_k.rx_datagram_count);
    udp_valid_rx_count_hist(step_index) = double(udp_diag_k.rx_valid_count);
    udp_invalid_rx_count_hist(step_index) = double(udp_diag_k.rx_invalid_count);

    if logical(udp_diag_k.tx_ok)
        json_tx_time_count = json_tx_time_count + 1;
        json_tx_timestamp_s(json_tx_time_count) = elapsed_s;
    end

    if logical(udp_diag_k.tx_ok && udp_diag_k.rx_valid)
        response_tx_time_count = response_tx_time_count + 1;
        response_tx_timestamp_s(response_tx_time_count) = elapsed_s;
    end

    if logical(udp_diag_k.rx_valid)
        valid_rx_time_count = valid_rx_time_count + 1;
        valid_rx_timestamp_s(valid_rx_time_count) = elapsed_s;
    end

    loop_elapsed_s(step_index) = toc(loop_tic);

    state_est_prev = est_k;
    next_tick_s = next_tick_s + dt_s;
end

transport = uav.ardupilot.json_udp_close(transport);

time_s = time_s(1:step_index);
state_hist = state_hist(1:step_index);
sensor_hist = sensor_hist(1:step_index);
estimator_hist = estimator_hist(1:step_index);
json_text_hist = json_text_hist(1:step_index);
sitl_output_hist = sitl_output_hist(1:step_index);
exchange_diag_hist = exchange_diag_hist(1:step_index);
exchange_status_hist = exchange_status_hist(1:step_index);
motor_cmd_hist_radps = motor_cmd_hist_radps(1:step_index, :);
quat_norm_true = quat_norm_true(1:step_index);
quat_norm_est = quat_norm_est(1:step_index);
loop_elapsed_s = loop_elapsed_s(1:step_index);
model_step_elapsed_s = model_step_elapsed_s(1:step_index);
json_encode_elapsed_s = json_encode_elapsed_s(1:step_index);
udp_step_elapsed_s = udp_step_elapsed_s(1:step_index);
udp_read_elapsed_s = udp_read_elapsed_s(1:step_index);
udp_write_elapsed_s = udp_write_elapsed_s(1:step_index);
udp_rx_datagram_count_hist = udp_rx_datagram_count_hist(1:step_index);
udp_valid_rx_count_hist = udp_valid_rx_count_hist(1:step_index);
udp_invalid_rx_count_hist = udp_invalid_rx_count_hist(1:step_index);
json_tx_timestamp_s = json_tx_timestamp_s(1:json_tx_time_count);
response_tx_timestamp_s = response_tx_timestamp_s(1:response_tx_time_count);
valid_rx_timestamp_s = valid_rx_timestamp_s(1:valid_rx_time_count);

result = struct();
result.cfg = cfg;
result.transport_message_open = transport_message_open;
result.transport_message_close = string(transport.message);
result.first_packet_received = first_packet_received;
result.first_packet_magic = first_packet_magic;
result.time_s = time_s;
result.state = state_hist;
result.sensors = sensor_hist;
result.estimator = estimator_hist;
result.json_text = json_text_hist;
result.sitl_output = sitl_output_hist;
result.exchange_diag = exchange_diag_hist;
result.exchange_status = exchange_status_hist;
result.motor_cmd_radps = motor_cmd_hist_radps;
result.prearm_hold_hist = prearm_hold_hist(1:step_index);
result.quat_norm_true = quat_norm_true;
result.quat_norm_est = quat_norm_est;
result.loop_elapsed_s = loop_elapsed_s;
result.model_step_elapsed_s = model_step_elapsed_s;
result.json_encode_elapsed_s = json_encode_elapsed_s;
result.udp_step_elapsed_s = udp_step_elapsed_s;
result.udp_read_elapsed_s = udp_read_elapsed_s;
result.udp_write_elapsed_s = udp_write_elapsed_s;
result.udp_rx_datagram_count_hist = udp_rx_datagram_count_hist;
result.udp_valid_rx_count_hist = udp_valid_rx_count_hist;
result.udp_invalid_rx_count_hist = udp_invalid_rx_count_hist;
result.json_tx_timestamp_s = json_tx_timestamp_s;
result.response_tx_timestamp_s = response_tx_timestamp_s;
result.valid_rx_timestamp_s = valid_rx_timestamp_s;
result.rx_datagram_count = rx_datagram_count;
result.valid_rx_count = valid_rx_count;
result.invalid_rx_count = invalid_rx_count;
result.json_tx_count = json_tx_count;
result.response_tx_count = response_tx_count;
result.json_probe_tx_count = json_tx_count - response_tx_count;
result.last_sender_address = last_sender_address;
result.last_sender_port = last_sender_port;
result.last_magic = last_magic;
result.last_frame_count = last_frame_count;
result.last_pwm_us = last_pwm_us;
result.last_motor_cmd_radps = last_motor_cmd_radps;
result.last_exchange_status = local_final_exchange_status( ...
    valid_rx_count, ...
    response_tx_count, ...
    last_frame_count, ...
    last_exchange_status);
result.quat_true_finite = all(isfinite(quat_norm_true));
result.quat_est_finite = all(isfinite(quat_norm_est));
result.exchange_confirmed = valid_rx_count > 0 && response_tx_count > 0;

assignin('base', 'ardupilot_json_udp_live_backend', result);

fprintf('20-секундный расчетный обмен MATLAB-модели с ArduPilot\n');
fprintf('  первый двоичный пакет получен                : %s\n', ...
    local_bool_text(first_packet_received));
fprintf('  принято UDP-датаграмм                         : %d\n', rx_datagram_count);
fprintf('  принято валидных двоичных пакетов             : %d\n', valid_rx_count);
fprintf('  принято невалидных двоичных пакетов           : %d\n', invalid_rx_count);
fprintf('  отправлено строк JSON                         : %d\n', json_tx_count);
fprintf('  выполнено ответных передач                    : %d\n', response_tx_count);
fprintf('  последний sender address                      : %s\n', char(last_sender_address));
fprintf('  последний sender port                         : %d\n', last_sender_port);
fprintf('  последний magic                               : %d\n', last_magic);
fprintf('  последний frame_count                         : %d\n', last_frame_count);
fprintf('  нормы истинного кватерниона конечны           : %s\n', local_bool_text(result.quat_true_finite));
fprintf('  нормы оцененного кватерниона конечны          : %s\n', local_bool_text(result.quat_est_finite));

if all(isfinite(last_pwm_us))
    fprintf('  последние принятые ШИМ [us]                   : [%s]\n', ...
        local_format_vector(last_pwm_us));
else
    fprintf('  последние принятые ШИМ [us]                   : отсутствуют\n');
end

if all(isfinite(last_motor_cmd_radps))
    fprintf('  последние команды частоты вращения [rad/s]    : [%s]\n', ...
        local_format_vector(last_motor_cmd_radps));
else
    fprintf('  последние команды частоты вращения [rad/s]    : отсутствуют\n');
end

fprintf('  итоговый статус последнего шага               : %s\n', ...
    char(result.last_exchange_status));

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end

function duration_s = local_read_duration_override(default_value)
%LOCAL_READ_DURATION_OVERRIDE Прочитать необязательное переопределение длительности.

duration_s = default_value;

if evalin('base', 'exist(''ardupilot_live_backend_duration_s'', ''var'')')
    candidate_value = evalin('base', 'ardupilot_live_backend_duration_s');
    validateattributes(candidate_value, {'numeric'}, ...
        {'real', 'scalar', 'finite', 'positive'}, ...
        mfilename, 'ardupilot_live_backend_duration_s');
    duration_s = double(candidate_value);
end
end

function cfg = local_apply_cfg_overrides(cfg)
%LOCAL_APPLY_CFG_OVERRIDES Применить необязательное переопределение конфигурации обмена.

if evalin('base', 'exist(''ardupilot_live_backend_update_rate_hz'', ''var'')')
    candidate_value = evalin('base', 'ardupilot_live_backend_update_rate_hz');
    validateattributes(candidate_value, {'numeric'}, ...
        {'real', 'scalar', 'finite', 'positive'}, ...
        mfilename, 'ardupilot_live_backend_update_rate_hz');
    cfg.update_rate_hz = double(candidate_value);
end

if evalin('base', 'exist(''ardupilot_json_cfg_override'', ''var'')')
    override_cfg = evalin('base', 'ardupilot_json_cfg_override');
    if ~isstruct(override_cfg) || ~isscalar(override_cfg)
        error('uav:task15:live_backend:CfgOverride', ...
            'Ожидалась скалярная структура ardupilot_json_cfg_override.');
    end

    field_names = fieldnames(override_cfg);
    for idx = 1:numel(field_names)
        cfg.(field_names{idx}) = override_cfg.(field_names{idx});
    end
end
end

function text_value = local_final_exchange_status(valid_rx_count, response_tx_count, last_frame_count, fallback_status)
%LOCAL_FINAL_EXCHANGE_STATUS Вернуть честный итоговый статус длительного обмена.

if valid_rx_count > 50 && response_tx_count > 50 && last_frame_count > 0
    text_value = "устойчивый обмен подтвержден";
elseif valid_rx_count > 0 && response_tx_count > 0
    text_value = "непрерывный обмен не подтвержден; получены только единичные ответные передачи";
elseif valid_rx_count > 0
    text_value = "непрерывный обмен не подтвержден; принят только единичный двоичный пакет";
elseif strlength(string(fallback_status)) > 0
    text_value = string(fallback_status);
else
    text_value = "прием не подтвержден";
end
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку числового вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
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

function diag = local_empty_exchange_diag(cfg)
%LOCAL_EMPTY_EXCHANGE_DIAG Построить пустую диагностику шага обмена.

diag = struct();
diag.status = "прием не подтвержден";
diag.status_message = "";
diag.rx_received = false;
diag.rx_valid = false;
diag.rx_bytes_count = 0;
diag.rx_message = "";
diag.rx_datagram_count = 0;
diag.rx_valid_count = 0;
diag.rx_invalid_count = 0;
diag.tx_attempted = false;
diag.tx_ok = false;
diag.tx_count = 0;
diag.response_tx_count = 0;
diag.tx_kind = "none";
diag.tx_message = "";
diag.handshake_confirmed = false;
diag.used_remote_ip = string(cfg.udp_remote_ip);
diag.used_remote_port = double(cfg.udp_remote_port);
diag.last_sender_address = "";
diag.last_sender_port = 0;
diag.last_magic = 0;
diag.last_frame_count = 0;
diag.rx_elapsed_s = 0.0;
diag.tx_elapsed_s = 0.0;
diag.step_elapsed_s = 0.0;
diag.tx_payload_bytes = 0;
end

function is_hold = local_is_prearm_hold_active(cfg, last_pwm_us)
%LOCAL_IS_PREARM_HOLD_ACTIVE Detect whether the plant should stay on the ground.

is_hold = false;
if ~isfield(cfg, 'json_prearm_hold_enabled') || ~logical(cfg.json_prearm_hold_enabled)
    return;
end

if isempty(last_pwm_us) || ~all(isfinite(last_pwm_us))
    return;
end

threshold_us = 1005.0;
if isfield(cfg, 'json_prearm_pwm_threshold_us')
    threshold_us = double(cfg.json_prearm_pwm_threshold_us);
end

is_hold = max(double(last_pwm_us(:))) <= threshold_us;
end
