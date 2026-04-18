%% RUN_CASE_YAW_STEP Execute a hover-level yaw-step case through uav.sim.run_case.
% Description:
%   Builds a run_case configuration with hover-level total thrust and a
%   positive yaw-moment step, then prints a compact diagnostic.
%
% Inputs:
%   none
%
% Outputs:
%   demo - returned in base workspace when assigned by the caller
%
% Units:
%   SI only
%
% Assumptions:
%   Total thrust stays near hover while only the commanded yaw moment steps.

params = uav.sim.default_params_quad_x250();

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = params.demo.case_yaw_step_t_final_s;
case_cfg.command_fun = @local_yaw_step_command;

log = uav.sim.run_case(case_cfg);
final_state = log.state(end);
final_total_thrust_N = -log.forces_b_N(end, 3);
hover_weight_N = params.mass_kg * params.gravity_mps2;
final_yaw_rate_radps = final_state.w_b_radps(3);
final_yaw_cmd_moment_Nm = local_yaw_moment(log.time_s(end), params);

fprintf('Run-case yaw-step diagnostics:\n');
fprintf('  step time         : %.3f s\n', params.demo.yaw_step_time_s);
fprintf('  final yaw moment  : %.6f N*m\n', final_yaw_cmd_moment_Nm);
fprintf('  final yaw rate    : %.6f rad/s\n', final_yaw_rate_radps);
fprintf('  final rotor speeds: [%.3f %.3f %.3f %.3f] rad/s\n', ...
    log.omega_m_radps(end, :));
fprintf('  total thrust      : %.6f N\n', final_total_thrust_N);
fprintf('  hover weight      : %.6f N\n', hover_weight_N);
fprintf('  final quat norm   : %.12f\n', log.quat_norm(end));

if usejava('jvm')
    figure('Name', 'run_case_yaw_step');

    subplot(2, 1, 1);
    plot(log.time_s, log.omega_m_radps, 'LineWidth', 1.5);
    grid on;
    ylabel('Rotor speed, rad/s');
    title('Yaw-step case via run\_case');

    subplot(2, 1, 2);
    yaw_rate_hist_radps = local_extract_yaw_rate(log.state);
    plot(log.time_s, yaw_rate_hist_radps, 'LineWidth', 1.5);
    grid on;
    xlabel('Time, s');
    ylabel('w_z, rad/s');
end

demo = struct();
demo.case_cfg = case_cfg;
demo.log = log;
demo.final_total_thrust_N = final_total_thrust_N;
demo.hover_weight_N = hover_weight_N;
demo.final_yaw_rate_radps = final_yaw_rate_radps;

assignin('base', 'case_yaw_step_demo', demo);

function motor_cmd_radps = local_yaw_step_command(t_s, ~, params)
%LOCAL_YAW_STEP_COMMAND Build the hover-level command with a yaw step.

total_thrust_N = params.mass_kg * params.gravity_mps2;
body_moments_Nm = [0.0; 0.0; local_yaw_moment(t_s, params)];
[motor_cmd_radps, ~] = uav.vmg.mixer_quad_x(total_thrust_N, body_moments_Nm, params);
end

function yaw_moment_Nm = local_yaw_moment(t_s, params)
%LOCAL_YAW_MOMENT Return the scheduled yaw moment command.

yaw_moment_Nm = 0.0;
if t_s >= params.demo.yaw_step_time_s
    yaw_moment_Nm = params.demo.yaw_step_moment_Nm;
end
end

function yaw_rate_hist_radps = local_extract_yaw_rate(state_hist)
%LOCAL_EXTRACT_YAW_RATE Extract the body yaw-rate history from log.state.

yaw_rate_hist_radps = zeros(numel(state_hist), 1);
for k = 1:numel(state_hist)
    yaw_rate_hist_radps(k) = state_hist(k).w_b_radps(3);
end
end
