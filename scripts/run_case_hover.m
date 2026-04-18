%% RUN_CASE_HOVER Execute the open-loop hover case through uav.sim.run_case.
% Description:
%   Builds a canonical case_cfg struct, runs the Stage-1.5 case runner, and
%   prints a compact hover diagnostic for engineering verification.
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
%   The hover command is constant and identical for all four rotors.

params = uav.sim.default_params_quad_x250();

case_cfg = struct();
case_cfg.params = params;
case_cfg.state0 = uav.core.state_unpack(params.demo.initial_state_plant);
case_cfg.dt_s = params.demo.dt_s;
case_cfg.t_final_s = params.demo.case_hover_t_final_s;
case_cfg.command_fun = @local_hover_command;

log = uav.sim.run_case(case_cfg);
final_state = log.state(end);
final_total_thrust_N = -log.forces_b_N(end, 3);
final_weight_N = params.mass_kg * params.gravity_mps2;
final_az_mps2 = local_vertical_accel( ...
    final_state, log.forces_b_N(end, :).', log.moments_b_Nm(end, :).', params);

fprintf('Run-case hover diagnostics:\n');
fprintf('  hover command     : [%.3f %.3f %.3f %.3f] rad/s\n', ...
    log.motor_cmd_radps(end, :));
fprintf('  final rotor speeds: [%.3f %.3f %.3f %.3f] rad/s\n', ...
    log.omega_m_radps(end, :));
fprintf('  total thrust      : %.6f N\n', final_total_thrust_N);
fprintf('  weight            : %.6f N\n', final_weight_N);
fprintf('  vertical accel est: %.6e m/s^2\n', final_az_mps2);
fprintf('  final quat norm   : %.12f\n', log.quat_norm(end));

if usejava('jvm')
    figure('Name', 'run_case_hover');

    subplot(2, 1, 1);
    plot(log.time_s, log.omega_m_radps, 'LineWidth', 1.5);
    grid on;
    ylabel('Rotor speed, rad/s');
    title('Hover case via run\_case');

    subplot(2, 1, 2);
    plot(log.time_s, log.quat_norm, 'LineWidth', 1.5);
    grid on;
    xlabel('Time, s');
    ylabel('Quaternion norm');
end

demo = struct();
demo.case_cfg = case_cfg;
demo.log = log;
demo.final_total_thrust_N = final_total_thrust_N;
demo.final_weight_N = final_weight_N;
demo.final_az_mps2 = final_az_mps2;

assignin('base', 'case_hover_demo', demo);

function motor_cmd_radps = local_hover_command(~, ~, params)
%LOCAL_HOVER_COMMAND Return the constant hover motor command.

motor_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
end

function az_mps2 = local_vertical_accel(state, forces_b_N, moments_b_Nm, params)
%LOCAL_VERTICAL_ACCEL Estimate body-axis vertical acceleration from a state.

x_packed = uav.core.state_pack(state);
inputs = struct('forces_b_N', forces_b_N, 'moments_b_Nm', moments_b_Nm);
dx_rigid = uav.core.eom6dof_quat(0.0, x_packed(1:13), params, inputs);
az_mps2 = dx_rigid(6);
end
