function tests = test_inspect_ardupilot_sitl_environment
%TEST_INSPECT_ARDUPILOT_SITL_ENVIRONMENT Проверки локальной среды.
% Назначение:
%   Подтверждает, что функция проверки локальной среды возвращает
%   структурированный результат и не завершается аварийно при отсутствии
%   реального `ArduPilot`.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testInspectionReturnsStructuredStatusWithoutThrowing(testCase)
%TESTINSPECTIONRETURNSSTRUCTUREDSTATUSWITHOUTTHROWING Проверить результат.

cfg = uav.ardupilot.default_json_config();
info = uav.ardupilot.inspect_sitl_environment(cfg);

verifyTrue(testCase, isstruct(info));
verifyTrue(testCase, isfield(info, 'is_ready'));
verifyTrue(testCase, isfield(info, 'has_wsl'));
verifyTrue(testCase, isfield(info, 'wsl_distros'));
verifyTrue(testCase, isfield(info, 'has_python'));
verifyTrue(testCase, isfield(info, 'has_git'));
verifyTrue(testCase, isfield(info, 'has_github_cli'));
verifyTrue(testCase, isfield(info, 'has_sim_vehicle'));
verifyTrue(testCase, isfield(info, 'has_ardupilot_root'));
verifyTrue(testCase, isfield(info, 'ardupilot_root'));
verifyTrue(testCase, isfield(info, 'sim_vehicle_path'));
verifyTrue(testCase, isfield(info, 'start_command'));
verifyTrue(testCase, isfield(info, 'can_form_start_command'));
verifyTrue(testCase, isfield(info, 'script_paths'));
verifyTrue(testCase, isfield(info, 'missing_items'));
verifyTrue(testCase, isfield(info, 'messages'));

verifyTrue(testCase, islogical(info.is_ready));
verifyTrue(testCase, islogical(info.has_python));
verifyTrue(testCase, islogical(info.has_git));
verifyTrue(testCase, islogical(info.has_github_cli));
verifyTrue(testCase, islogical(info.has_sim_vehicle));
verifyTrue(testCase, islogical(info.has_ardupilot_root));
verifyTrue(testCase, islogical(info.can_form_start_command));
verifyTrue(testCase, isstring(info.wsl_distros));
verifyTrue(testCase, isstring(info.missing_items));
verifyTrue(testCase, isstring(info.messages));
verifyGreaterThanOrEqual(testCase, numel(info.messages), 1);
verifyGreaterThan(testCase, strlength(string(info.start_command)), 0);
verifyTrue(testCase, isstruct(info.script_paths));
end
