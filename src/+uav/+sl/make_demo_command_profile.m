function cmd = make_demo_command_profile(mode, params, dt_s, t_final_s)
%MAKE_DEMO_COMMAND_PROFILE Create a discrete motor-command profile.
% Description:
%   Generates simple hover and yaw-step command profiles for the thin MIL
%   shell demos and provides both numeric history and a From Workspace
%   compatible time series.
%
% Inputs:
%   mode      - command profile mode: 'hover' or 'yaw_step'
%   params    - parameter struct
%   dt_s      - discrete sample time [s]
%   t_final_s - final profile time [s]
%
% Outputs:
%   cmd - struct with time_s, motor_cmd_radps, and timeseries fields
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   t_final_s is an integer multiple of dt_s.

mode = validatestring(mode, {'hover', 'yaw_step'}, mfilename, 'mode');
validateattributes(dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'dt_s');
validateattributes(t_final_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 't_final_s');

n_intervals = round(t_final_s / dt_s);
if abs(n_intervals * dt_s - t_final_s) > 1.0e-12
    error('uav:sl:make_demo_command_profile:TimeGrid', ...
        'Expected t_final_s to be an integer multiple of dt_s.');
end

time_s = (0:n_intervals).' .* dt_s;
motor_cmd_radps = zeros(numel(time_s), 4);

switch mode
    case 'hover'
        motor_cmd_radps(:, :) = repmat(params.hover_omega_radps, numel(time_s), 4);

    case 'yaw_step'
        total_thrust_N = params.mass_kg * params.gravity_mps2;
        for k = 1:numel(time_s)
            body_moments_Nm = [0.0; 0.0; local_yaw_moment(time_s(k), params)];
            [motor_cmd_k_radps, ~] = uav.vmg.mixer_quad_x( ...
                total_thrust_N, body_moments_Nm, params);
            motor_cmd_radps(k, :) = motor_cmd_k_radps.';
        end
end

cmd = struct();
cmd.mode = mode;
cmd.dt_s = dt_s;
cmd.t_final_s = t_final_s;
cmd.time_s = time_s;
cmd.motor_cmd_radps = motor_cmd_radps;
cmd.timeseries = timeseries( ...
    reshape(motor_cmd_radps.', [4, 1, numel(time_s)]), ...
    time_s, ...
    'Name', 'motor_cmd_radps');
cmd.timeseries = setinterpmethod(cmd.timeseries, 'zoh');
end

function yaw_moment_Nm = local_yaw_moment(t_s, params)
%LOCAL_YAW_MOMENT Return the scheduled demo yaw moment.
% Description:
%   Applies a positive yaw moment after the configured demo step time.
%
% Inputs:
%   t_s    - current profile time [s]
%   params - parameter struct with demo yaw-step settings
%
% Outputs:
%   yaw_moment_Nm - commanded body yaw moment [N*m]
%
% Units:
%   SI only
%
% Assumptions:
%   The yaw-step schedule is configured in params.demo.

yaw_moment_Nm = 0.0;
if t_s >= params.demo.yaw_step_time_s
    yaw_moment_Nm = params.demo.yaw_step_moment_Nm;
end
end
