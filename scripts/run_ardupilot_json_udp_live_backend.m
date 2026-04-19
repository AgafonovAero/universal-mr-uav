%% RUN_ARDUPILOT_JSON_UDP_LIVE_BACKEND Выполнить 20-секундный расчетный обмен с ArduPilot.
% Назначение:
%   Запускает минимальный расчетный сценарий живого обмена по JSON и UDP
%   между существующей математической моделью движения и уже запущенным
%   ArduPilot SITL. Сценарий не заявляет устойчивый полет и предназначен
%   только для подтверждения факта приема двоичных пакетов, формирования
%   строк JSON и конечности расчетных величин.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_json_udp_live_backend - структура результата в базовом
%   рабочем пространстве MATLAB
%
% Единицы измерения:
%   время задается в секундах, команды ШИМ - в микросекундах, команды
%   частоты вращения винтов - в радианах в секунду
%
% Допущения:
%   ArduPilot SITL уже запущен отдельно. При отсутствии входящих пакетов
%   сценарий завершается штатно и честно фиксирует отсутствие
%   подтвержденного расчетного обмена.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = 20.0;
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_udp(case_cfg);

rx_valid_mask = [log.exchange_diag.rx_valid];
tx_ok_mask = [log.exchange_diag.tx_ok];
reply_mask = [log.exchange_diag.handshake_confirmed];

rx_valid_count = nnz(rx_valid_mask);
tx_ok_count = nnz(tx_ok_mask);
reply_count = nnz(reply_mask);

quat_true_finite = all(isfinite(log.quat_norm_true));
quat_est_finite = all(isfinite(log.quat_norm_est));

last_valid_pwm_us = [];
last_valid_motor_cmd_radps = [];

if any(rx_valid_mask)
    last_valid_idx = find(rx_valid_mask, 1, 'last');
    last_valid_pwm_us = log.sitl_output(last_valid_idx).motor_pwm_us;
    last_valid_motor_cmd_radps = log.motor_cmd_radps(last_valid_idx, :).';
end

result = struct();
result.cfg = cfg;
result.log = log;
result.rx_valid_count = rx_valid_count;
result.tx_ok_count = tx_ok_count;
result.reply_count = reply_count;
result.quat_true_finite = quat_true_finite;
result.quat_est_finite = quat_est_finite;
result.last_valid_pwm_us = last_valid_pwm_us;
result.last_valid_motor_cmd_radps = last_valid_motor_cmd_radps;
result.exchange_confirmed = rx_valid_count > 0 && tx_ok_count > 0;

assignin('base', 'ardupilot_json_udp_live_backend', result);

fprintf('20-секундный расчетный обмен MATLAB-модели с ArduPilot\n');
fprintf('  принято валидных двоичных пакетов          : %d\n', rx_valid_count);
fprintf('  отправлено строк JSON                      : %d\n', tx_ok_count);
fprintf('  выполнено ответных передач                 : %d\n', reply_count);
fprintf('  нормы истинного кватерниона конечны        : %s\n', local_bool_text(quat_true_finite));
fprintf('  нормы оцененного кватерниона конечны       : %s\n', local_bool_text(quat_est_finite));

if ~isempty(last_valid_pwm_us)
    fprintf('  последние принятые ШИМ [us]                : [%s]\n', ...
        local_format_vector(last_valid_pwm_us));
    fprintf('  последние команды частоты вращения [rad/s] : [%s]\n', ...
        local_format_vector(last_valid_motor_cmd_radps));
else
    fprintf('  последние принятые ШИМ [us]                : отсутствуют\n');
    fprintf('  последние команды частоты вращения [rad/s] : отсутствуют\n');
end

fprintf('  итоговый статус последнего шага            : %s\n', ...
    char(log.exchange_status(end)));

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку числового вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end
