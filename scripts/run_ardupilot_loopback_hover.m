%% RUN_ARDUPILOT_LOOPBACK_HOVER Выполнить проверочный прогон режима зависания.
% Назначение:
%   Запускает средство сопряжения с `ArduPilot` в режиме проверочного
%   замкнутого прогона с детерминированным пакетом команд ШИМ вместо
%   реального внешнего комплекса и печатает итоговые диагностические
%   величины.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_loopback_hover - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   SI only, printed PWM in microseconds and motor speed in rad/s
%
% Допущения:
%   Используемый пакет команд является проверочной заменой и не
%   представляет реальный закон управления `ArduPilot`.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = 4.0;
case_cfg.loopback_mode = "hover";
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_loopback(case_cfg);
final_state = log.state(end);
final_est = log.estimator(end);
final_servo = log.servo(end);
final_motor_cmd_radps = log.motor_cmd_radps(end, :).';

result = struct();
result.name = 'ardupilot_loopback_hover';
result.case_cfg = case_cfg;
result.log = log;

assignin('base', 'ardupilot_loopback_hover', result);

fprintf('Проверочный замкнутый прогон ArduPilot: режим зависания\n');
fprintf('  конечная высота [m]                    : %.6f\n', -final_state.p_ned_m(3));
fprintf('  конечная оцененная высота [m]          : %.6f\n', final_est.alt_m);
fprintf('  конечные команды ШИМ [us]              : [%s]\n', ...
    local_format_vector(final_servo.motor_pwm_us));
fprintf('  конечные команды частоты [rad/s]       : [%s]\n', ...
    local_format_vector(final_motor_cmd_radps));
fprintf('  нормы кватернионов [-]                 : ист=%.12f оц=%.12f\n', ...
    log.quat_norm_true(end), log.quat_norm_est(end));

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку для числового вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end
