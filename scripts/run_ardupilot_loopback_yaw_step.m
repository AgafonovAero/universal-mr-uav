%% RUN_ARDUPILOT_LOOPBACK_YAW_STEP Выполнить прогон ступенчатого рыскания.
% Description:
%   Запускает средство сопряжения с `ArduPilot` с синтетическим
%   ступенчатым изменением команд ШИМ по каналу рыскания вместо реального
%   внешнего комплекса и печатает итоговые диагностические величины.
%
% Inputs:
%   none
%
% Outputs:
%   ardupilot_loopback_yaw_step - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Units:
%   SI only, explicit degree conversion only for printed yaw
%
% Assumptions:
%   Пакет ступенчатого воздействия по рысканию является проверочной
%   заменой и не моделирует реальные переходы режимов внешнего автопилота.

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = 1.0 / cfg.update_rate_hz;
case_cfg.t_final_s = 5.0;
case_cfg.loopback_mode = "yaw_step";
case_cfg.ardupilot_cfg = cfg;

log = uav.sim.run_case_with_ardupilot_loopback(case_cfg);
final_state = log.state(end);
final_est = log.estimator(end);
final_servo = log.servo(end);
final_euler_rpy_rad = local_quat_to_euler_rpy(final_state.q_nb);

result = struct();
result.name = 'ardupilot_loopback_yaw_step';
result.case_cfg = case_cfg;
result.log = log;

assignin('base', 'ardupilot_loopback_yaw_step', result);

fprintf('Проверочный замкнутый прогон ArduPilot: ступенчатое воздействие по рысканию\n');
fprintf('  конечный угол рыскания [deg]           : %.6f\n', rad2deg(final_euler_rpy_rad(3)));
fprintf('  конечная угловая скорость [rad/s]      : %.6f\n', final_state.w_b_radps(3));
fprintf('  конечная высота [m]                    : %.6f\n', -final_state.p_ned_m(3));
fprintf('  конечная оцененная высота [m]          : %.6f\n', final_est.alt_m);
fprintf('  конечные команды ШИМ [us]              : [%s]\n', ...
    local_format_vector(final_servo.motor_pwm_us));
fprintf('  нормы кватернионов [-]                 : ист=%.12f оц=%.12f\n', ...
    log.quat_norm_true(end), log.quat_norm_est(end));

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать компактную строку для числового вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end

function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
%LOCAL_QUAT_TO_EULER_RPY Преобразовать q_nb в углы крена, тангажа и рыскания.

q_nb = uav.core.quat_normalize(q_nb);

qw = q_nb(1);
qx = q_nb(2);
qy = q_nb(3);
qz = q_nb(4);

sin_pitch = 2.0 .* (qw * qy - qz * qx);

euler_rpy_rad = [ ...
    atan2(2.0 .* (qw * qx + qy * qz), 1.0 - 2.0 .* (qx^2 + qy^2)); ...
    asin(max(min(sin_pitch, 1.0), -1.0)); ...
    atan2(2.0 .* (qw * qz + qx * qy), 1.0 - 2.0 .* (qy^2 + qz^2))];
end
