%% RUN_ARDUPILOT_ENVIRONMENT_BOOTSTRAP_CHECK Проверить подготовку среды.
% Назначение:
%   Выполняет сводную проверку внешней среды `ArduPilot SITL`
%   и печатает рекомендуемую последовательность действий для оператора.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_environment_bootstrap_check - структура результата
%   в базовом рабочем пространстве MATLAB
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Сценарий не выполняет административных действий и не требует
%   готовности `WSL` или внешнего каталога `ArduPilot`.

repo_root = local_repo_root();
if exist('uav.ardupilot.default_json_config', 'file') ~= 2
    addpath(fullfile(repo_root, 'src'));
end

cfg = uav.ardupilot.default_json_config();
env_info = uav.ardupilot.inspect_sitl_environment(cfg);
setup = uav.ardupilot.make_sitl_setup_commands(cfg, env_info);
start_cmd = uav.ardupilot.make_sitl_start_command(cfg);

result = struct();
result.cfg = cfg;
result.env_info = env_info;
result.setup = setup;
result.start_command = start_cmd;

assignin('base', 'ardupilot_environment_bootstrap_check', result);

fprintf('ArduPilot SITL environment bootstrap check\n');
fprintf('  WSL ready                                 : %s\n', ...
    local_bool_text(env_info.has_wsl));
fprintf('  external ArduPilot root ready             : %s\n', ...
    local_bool_text(env_info.has_ardupilot_root));
fprintf('  sim_vehicle.py found                      : %s\n', ...
    local_bool_text(env_info.has_sim_vehicle));
fprintf('  local environment ready                   : %s\n', ...
    local_bool_text(env_info.is_ready));

fprintf('\nPowerShell script commands\n');
fprintf('  Windows and WSL check                     : %s\n', ...
    setup.windows_commands(1));
fprintf('  WSL preparation                           : %s\n', ...
    setup.windows_commands(2));
fprintf('  ArduPilot setup inside WSL                : %s\n', ...
    setup.windows_commands(3));
fprintf('  ArduCopter JSON SITL start                : %s\n', ...
    setup.windows_commands(4));

fprintf('\nArduCopter JSON SITL start command\n');
fprintf('  %s\n', char(start_cmd.command_text));

fprintf('\nMATLAB commands for the next stage\n');
fprintf('  wait for first packet                     : %s\n', ...
    cfg.matlab_wait_for_packet_script);
fprintf('  handshake check                           : %s\n', ...
    cfg.matlab_handshake_script);

fprintf('\nRecommended sequence of actions\n');
for idx = 1:numel(setup.recommended_sequence)
    fprintf('  %s\n', char(setup.recommended_sequence(idx)));
end

fprintf('\nMissing environment items\n');
if isempty(env_info.missing_items)
    fprintf('  no missing items were detected.\n');
else
    for idx = 1:numel(env_info.missing_items)
        fprintf('  - %s\n', char(env_info.missing_items(idx)));
    end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в русскую строку.

if flag_value
    text_value = 'yes';
else
    text_value = 'no';
end
end

function repo_root = local_repo_root()
%LOCAL_REPO_ROOT Вернуть корневой каталог репозитория.

repo_root = fileparts(fileparts(mfilename('fullpath')));
end
