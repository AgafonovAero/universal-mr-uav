%% RUN_MOTOR_SPOOL_DEMO Demonstrate first-order rotor spool-up dynamics.
% Description:
%   Simulates rotor speed convergence from zero to a constant hover command
%   using the Stage-1.5 motor kernel.
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
%   All four rotor commands are equal and constant.

params = uav.sim.default_params_quad_x250();
dt_s = params.demo.dt_s;
t_final_s = params.demo.motor_spool_t_final_s;
t_grid_s = (0.0:dt_s:t_final_s).';

omega_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
omega_hist_radps = zeros(numel(t_grid_s), 4);
omega_hist_radps(1, :) = zeros(1, 4);

for k = 2:numel(t_grid_s)
    [omega_next_radps, motor_diag] = uav.vmg.motor_esc_step( ...
        omega_hist_radps(k - 1, :).', omega_cmd_radps, dt_s, params.motor);
    omega_hist_radps(k, :) = omega_next_radps.';
end

fprintf('Motor spool demo diagnostics:\n');
fprintf('  tau               : %.4f s\n', params.motor.tau_s);
fprintf('  command           : %.3f rad/s\n', params.hover_omega_radps);
fprintf('  final rotor speeds: [%.3f %.3f %.3f %.3f] rad/s\n', omega_hist_radps(end, :));
fprintf('  command error     : [%.6f %.6f %.6f %.6f] rad/s\n', ...
    omega_cmd_radps.' - omega_hist_radps(end, :));
fprintf('  last domega       : [%.6f %.6f %.6f %.6f] rad/s^2\n', motor_diag.domega_radps2);

if usejava('jvm')
    figure('Name', 'run_motor_spool_demo');
    plot(t_grid_s, omega_hist_radps, 'LineWidth', 1.5);
    grid on;
    xlabel('Time, s');
    ylabel('Rotor speed, rad/s');
    title('Motor spool demo');
end

demo = struct();
demo.params = params;
demo.t_grid_s = t_grid_s;
demo.omega_cmd_radps = omega_cmd_radps;
demo.omega_hist_radps = omega_hist_radps;

assignin('base', 'motor_spool_demo', demo);
