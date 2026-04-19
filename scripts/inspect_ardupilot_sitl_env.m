%% INSPECT_ARDUPILOT_SITL_ENV Inspect local readiness for ArduPilot SITL.
% Description:
%   Prints a compact inventory of local tooling needed for a future real
%   ArduPilot JSON SITL step. The script never fails just because
%   ArduPilot is not installed yet.
%
% Inputs:
%   none
%
% Outputs:
%   ardupilot_sitl_env - assigned in base workspace
%
% Units:
%   not applicable
%
% Assumptions:
%   TASK-10 checks only local readiness and loopback-scaffold status.

cfg = uav.ardupilot.default_json_config();
info = uav.ardupilot.inspect_sitl_environment(cfg);

assignin('base', 'ardupilot_sitl_env', info);

fprintf('ArduPilot JSON SITL environment inspection:\n');
fprintf('  ready for real SITL : %s\n', local_bool_text(info.is_ready));
fprintf('  has WSL             : %s\n', local_bool_text(info.has_wsl));
fprintf('  has Python          : %s\n', local_bool_text(info.has_python));
fprintf('  has sim_vehicle.py  : %s\n', local_bool_text(info.has_sim_vehicle));

if strlength(string(info.ardupilot_root)) > 0
    fprintf('  ArduPilot root      : %s\n', char(info.ardupilot_root));
else
    fprintf('  ArduPilot root      : <not found>\n');
end

fprintf('  messages:\n');
for k = 1:numel(info.messages)
    fprintf('    - %s\n', char(info.messages(k)));
end

function text_value = local_bool_text(value)
%LOCAL_BOOL_TEXT Convert a logical scalar to compact text.

if logical(value)
    text_value = 'yes';
else
    text_value = 'no';
end
end
