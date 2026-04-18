function demo = run_hover_demo()
%RUN_HOVER_DEMO Run a minimal hover-balance simulation for the baseline quad-X.
% Description:
%   Builds hover rotor commands from the baseline parameter set, computes
%   the resulting body force, integrates the 6DOF model, and shows simple
%   diagnostics and plots.
%
% Inputs:
%   none
%
% Outputs:
%   demo - struct with command, force-balance, and state history data
%
% Units:
%   SI only
%
% Assumptions:
%   Level initial attitude, constant rotor commands, gravity-only
%   environment, and zero aerodynamic disturbance.

params = uav.sim.default_params_quad_x250();
targetThrust_N = params.mass_kg * params.gravity_mps2;

[omega_cmd_radps, thrust_cmd_N] = uav.vmg.mixer_quad_x(targetThrust_N, [0.0; 0.0; 0.0], params);
[rotorThrust_N, rotorTorque_Nm] = uav.vmg.rotor_simple( ...
    omega_cmd_radps, params.kT_N_per_radps2, params.kQ_Nm_per_radps2);

totalThrust_N = sum(rotorThrust_N);
bodyForces_N = [0.0; 0.0; -totalThrust_N];
bodyMoments_Nm = [0.0; 0.0; 0.0];

inputs = struct();
inputs.forces_b_N = bodyForces_N;
inputs.moments_b_Nm = bodyMoments_Nm;

x0 = params.demo.initial_state;
odefun = @(t, x) uav.core.eom6dof_quat(t, x, params, inputs);
[tHist_s, xHist] = ode45(odefun, [0.0, params.demo.t_final_s], x0);
initialDx = odefun(0.0, x0);

fprintf('Hover demo diagnostics:\n');
fprintf('  mass              : %.3f kg\n', params.mass_kg);
fprintf('  weight            : %.6f N\n', targetThrust_N);
fprintf('  total thrust      : %.6f N\n', totalThrust_N);
fprintf('  thrust imbalance  : %.6e N\n', totalThrust_N - targetThrust_N);
fprintf('  rotor commands    : [%.3f %.3f %.3f %.3f] rad/s\n', omega_cmd_radps);
fprintf('  rotor thrusts     : [%.3f %.3f %.3f %.3f] N\n', rotorThrust_N);
fprintf('  rotor torques     : [%.6f %.6f %.6f %.6f] N*m\n', rotorTorque_Nm);
fprintf('  initial a_z(body) : %.6e m/s^2\n', initialDx(6));

if usejava('jvm')
    figure('Name', 'uav.sim.run_hover_demo');

    subplot(2, 1, 1);
    plot(tHist_s, xHist(:, 3), 'LineWidth', 1.5);
    grid on;
    ylabel('Down position, m');
    title('Minimal hover demo');

    subplot(2, 1, 2);
    plot(tHist_s, xHist(:, 6), 'LineWidth', 1.5);
    grid on;
    xlabel('Time, s');
    ylabel('Body z velocity, m/s');
end

demo = struct();
demo.params = params;
demo.omega_cmd_radps = omega_cmd_radps;
demo.thrust_cmd_N = thrust_cmd_N;
demo.rotorThrust_N = rotorThrust_N;
demo.rotorTorque_Nm = rotorTorque_Nm;
demo.totalThrust_N = totalThrust_N;
demo.inputs = inputs;
demo.tHist_s = tHist_s;
demo.xHist = xHist;
demo.initialDx = initialDx;
end
