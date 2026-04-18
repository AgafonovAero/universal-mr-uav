function params = default_params_quad_x250()
%DEFAULT_PARAMS_QUAD_X250 Baseline demo parameters for a quad-X 250 mm UAV.
% Description:
%   Returns a compact parameter struct for a baseline quad-X 250 mm class
%   vehicle. Values are simple demo baselines, not identified hardware data.
%
% Inputs:
%   none
%
% Outputs:
%   params - parameter struct used by Stage-1 code
%
% Units:
%   SI only
%
% Assumptions:
%   The vehicle is symmetric, rigid, and intended only for baseline
%   demonstration and verification at this stage.

params = struct();

params.mass_kg = 1.00;
params.inertia_kgm2 = diag([5.0e-3, 5.0e-3, 9.0e-3]);
params.gravity_mps2 = 9.81;

% wheelbase_m is the diagonal motor-to-motor distance for opposite rotors.
params.wheelbase_m = 0.25;
params.motor_radius_m = params.wheelbase_m / 2.0;

motor_xy_abs_m = params.motor_radius_m / sqrt(2.0);
params.motor_xy_m = [ ...
     motor_xy_abs_m, -motor_xy_abs_m; ...
     motor_xy_abs_m,  motor_xy_abs_m; ...
    -motor_xy_abs_m,  motor_xy_abs_m; ...
    -motor_xy_abs_m, -motor_xy_abs_m];

params.kT_N_per_radps2 = 8.0e-6;
params.kQ_Nm_per_radps2 = 1.2e-7;
params.spin_dir = [1.0; -1.0; 1.0; -1.0];

params.rate_pid_gains = struct( ...
    'Kp', [0.10; 0.10; 0.08], ...
    'Ki', [0.02; 0.02; 0.01], ...
    'Kd', [0.002; 0.002; 0.001]);

params.hover_omega_radps = sqrt((params.mass_kg * params.gravity_mps2) / (4.0 * params.kT_N_per_radps2));
params.demo.t_final_s = 2.0;
params.demo.initial_state = [zeros(6, 1); 1.0; 0.0; 0.0; 0.0; zeros(3, 1)];
end
