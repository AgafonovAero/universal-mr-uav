function cfg = default_atc_bridge_config(external_repo_root, params)
%DEFAULT_ATC_BRIDGE_CONFIG Default configuration for the ATC MIL bridge.
% Description:
%   Creates the minimal configuration struct required to connect the
%   external `atc_controller` repository to the existing Stage-1.5+ kernel
%   without turning the `.slx` shell into the source of truth.
%
% Inputs:
%   external_repo_root - absolute path to the external ATC repository
%   params             - universal-mr-uav plant parameter struct
%
% Outputs:
%   cfg - validated scalar configuration struct for `uav.atc.*`
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   The external controller is used in direct MATLAB-call mode through
%   `FSW_Simulink_wrapper_step`.

if nargin < 2 || isempty(params)
    params = uav.sim.default_params_quad_x250();
end

if nargin < 1 || isempty(external_repo_root)
    universal_repo_root = fileparts(fileparts(fileparts(fileparts( ...
        mfilename('fullpath')))));
    workspace_root = fileparts(universal_repo_root);
    external_repo_root = fullfile(workspace_root, 'atc_controller');
end

hover_thrust_norm = local_hover_thrust_norm(params);

cfg = struct();
cfg.external_repo_root = char(external_repo_root);
cfg.enable_external_path_bootstrap = true;
cfg.setup_function = 'setup_paths';
cfg.controller_step_function = 'FSW_Simulink_wrapper_step';
cfg.controller_input_factory = 'FSW_make_default_in';
cfg.controller_param_factory = 'ATC_Params_default';
cfg.controller_mixer_factory = 'AP_MotorsMatrix_params_QuadX';
cfg.controller_actuator_to_thrust_function = ...
    'AP_MotorsMatrix_actuator_to_thrust';

cfg.mode_name = "hover";
cfg.mode_cmd = uint8(5);
cfg.arm_time_s = 0.0;
cfg.yaw_step_time_s = params.demo.yaw_step_time_s;
cfg.yaw_rate_step_radps = 30.0 * pi / 180.0;
cfg.roll_cmd_rad = 0.0;
cfg.pitch_cmd_rad = 0.0;
cfg.takeoff_alt_m = 1.0;
cfg.slew_yaw = false;

cfg.hover_thrust_norm = hover_thrust_norm;
cfg.manual_throttle_norm = hover_thrust_norm;
cfg.sim_motor_order_map = uint8([1; 2; 3; 4]);

cfg.use_estimator_attitude = true;
cfg.use_estimator_altitude = true;
cfg.use_gnss_xy = true;

cfg.rc_ok = true;
cfg.gps_ok = true;
cfg.of_ok = true;
cfg.battery_voltage_v = 16.0;
cfg.ground_contact_alt_m = 0.03;
cfg.ground_contact_vz_mps = 0.20;
cfg.require_direct_call = true;

cfg.controller_notes = ...
    "Direct MATLAB call through FSW_Simulink_wrapper_step with quad-X only.";
end

function hover_thrust_norm = local_hover_thrust_norm(params)
%LOCAL_HOVER_THRUST_NORM Convert vehicle hover to normalized thrust.
% Description:
%   Computes the hover thrust fraction relative to the physical maximum
%   thrust of the current universal-mr-uav rotor model.
%
% Inputs:
%   params - universal-mr-uav parameter struct
%
% Outputs:
%   hover_thrust_norm - scalar hover thrust fraction in [0, 1]
%
% Units:
%   dimensionless
%
% Assumptions:
%   `T = kT * omega^2` remains the plant propulsion law.

required_fields = {'mass_kg', 'gravity_mps2', 'rotor', 'motor'};
for k = 1:numel(required_fields)
    if ~isfield(params, required_fields{k})
        error('uav:atc:default_atc_bridge_config:MissingParamsField', ...
            'Expected params.%s to be present.', required_fields{k});
    end
end

if ~isfield(params.rotor, 'kT_N_per_radps2') || ...
        ~isfield(params.motor, 'omega_max_radps')
    error('uav:atc:default_atc_bridge_config:MissingRotorOrMotorField', ...
        'Expected params.rotor.kT_N_per_radps2 and params.motor.omega_max_radps.');
end

max_total_thrust_N = 4.0 * params.rotor.kT_N_per_radps2 * ...
    params.motor.omega_max_radps^2;
hover_total_thrust_N = params.mass_kg * params.gravity_mps2;

hover_thrust_norm = hover_total_thrust_N / max_total_thrust_N;
hover_thrust_norm = min(max(hover_thrust_norm, 0.0), 1.0);
end
