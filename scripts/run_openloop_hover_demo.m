%% RUN_OPENLOOP_HOVER_DEMO Simulate an open-loop hover command with plant_step.
% Description:
%   Uses the Stage-1.5 plant kernel to simulate open-loop rotor spool-up
%   and near-hover force balance under a constant hover command.
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
%   The vehicle starts level and the command is a constant hover-speed
%   reference for all four rotors.

params = uav.sim.default_params_quad_x250();
dt_s = params.demo.dt_s;
t_final_s = params.demo.openloop_hover_t_final_s;
t_grid_s = (0.0:dt_s:t_final_s).';

x_hist = zeros(numel(t_grid_s), 17);
x_hist(1, :) = params.demo.initial_state_plant.';

hover_cmd_radps = repmat(params.hover_omega_radps, 4, 1);
thrust_hist_N = zeros(numel(t_grid_s), 1);
az_hist_mps2 = zeros(numel(t_grid_s), 1);

for k = 2:numel(t_grid_s)
    [x_next, step_diag] = uav.sim.plant_step(x_hist(k - 1, :).', hover_cmd_radps, dt_s, params);
    x_hist(k, :) = x_next.';
    thrust_hist_N(k) = -step_diag.forces_b_N(3);
    az_hist_mps2(k) = step_diag.dx_rigid(6);
end

final_omega_radps = x_hist(end, 14:17).';
final_totalThrust_N = thrust_hist_N(end);
weight_N = params.mass_kg * params.gravity_mps2;
vertical_accel_estimate_mps2 = az_hist_mps2(end);

fprintf('Open-loop hover demo diagnostics:\n');
fprintf('  hover command     : [%.3f %.3f %.3f %.3f] rad/s\n', hover_cmd_radps);
fprintf('  final rotor speeds: [%.3f %.3f %.3f %.3f] rad/s\n', final_omega_radps);
fprintf('  total thrust      : %.6f N\n', final_totalThrust_N);
fprintf('  weight            : %.6f N\n', weight_N);
fprintf('  vertical accel est: %.6e m/s^2\n', vertical_accel_estimate_mps2);

if usejava('jvm')
    figure('Name', 'run_openloop_hover_demo');

    subplot(2, 1, 1);
    plot(t_grid_s, x_hist(:, 14:17), 'LineWidth', 1.5);
    grid on;
    ylabel('Rotor speed, rad/s');
    title('Open-loop hover demo');

    subplot(2, 1, 2);
    plot(t_grid_s, az_hist_mps2, 'LineWidth', 1.5);
    grid on;
    xlabel('Time, s');
    ylabel('Estimated a_z(body), m/s^2');
end

demo = struct();
demo.params = params;
demo.t_grid_s = t_grid_s;
demo.hover_cmd_radps = hover_cmd_radps;
demo.x_hist = x_hist;
demo.thrust_hist_N = thrust_hist_N;
demo.az_hist_mps2 = az_hist_mps2;
demo.final_totalThrust_N = final_totalThrust_N;
demo.vertical_accel_estimate_mps2 = vertical_accel_estimate_mps2;

assignin('base', 'openloop_hover_demo', demo);
