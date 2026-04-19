%% RUN_ARDUPILOT_LOOPBACK_HOVER Execute the ArduPilot loopback hover smoke.
% Description:
%   Runs the ArduPilot JSON adapter scaffold with a deterministic hover
%   loopback packet instead of a real SITL backend and prints compact
%   end-of-run diagnostics.
%
% Inputs:
%   none
%
% Outputs:
%   ardupilot_loopback_hover - assigned in base workspace
%
% Units:
%   SI only, printed PWM in microseconds and motor speed in rad/s
%
% Assumptions:
%   The loopback packet is a smoke-test placeholder, not a real ArduPilot
%   control law.

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

fprintf('ArduPilot loopback hover diagnostics:\n');
fprintf('  final altitude [m]          : %.6f\n', -final_state.p_ned_m(3));
fprintf('  final estimated altitude [m]: %.6f\n', final_est.alt_m);
fprintf('  final motor PWM [us]        : [%s]\n', ...
    local_format_vector(final_servo.motor_pwm_us));
fprintf('  final motor command [rad/s] : [%s]\n', ...
    local_format_vector(final_motor_cmd_radps));
fprintf('  final quat norms [-]        : true=%.12f est=%.12f\n', ...
    log.quat_norm_true(end), log.quat_norm_est(end));

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Format one numeric vector for compact printing.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end
