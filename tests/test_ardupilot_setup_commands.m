function tests = test_ardupilot_setup_commands
%TEST_ARDUPILOT_SETUP_COMMANDS Проверки сценариев подготовки среды.
% Назначение:
%   Подтверждает, что `make_sitl_setup_commands` возвращает пути
%   к операторским сценариям и рекомендуемую последовательность действий
%   для Windows/WSL без автоматического выполнения опасных операций.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testSetupCommandsReturnScriptPaths(testCase)
%TESTSETUPCOMMANDSRETURNSCRIPTPATHS Проверить пути к сценариям.

cfg = uav.ardupilot.default_json_config();
envInfo = uav.ardupilot.inspect_sitl_environment(cfg);
setup = uav.ardupilot.make_sitl_setup_commands(cfg, envInfo);

verifyTrue(testCase, isstruct(setup.script_paths));
verifyTrue(testCase, isfield(setup.script_paths, 'test_windows_environment'));
verifyTrue(testCase, isfield(setup.script_paths, 'setup_wsl'));
verifyTrue(testCase, isfield(setup.script_paths, 'invoke_wsl_setup'));
verifyTrue(testCase, isfield(setup.script_paths, 'start_json_sitl'));
verifyTrue(testCase, isfield(setup.script_paths, 'wsl_setup_script'));
verifyTrue(testCase, isfield(setup.script_paths, 'wsl_start_script'));

verifyTrue(testCase, contains(string(setup.windows_commands(1)), 'Test-ArduPilotEnvironment.ps1'));
verifyTrue(testCase, contains(string(setup.windows_commands(2)), 'Setup-WSLForArduPilot.ps1'));
verifyTrue(testCase, contains(string(setup.windows_commands(3)), 'Invoke-ArduPilotWslSetup.ps1'));
verifyTrue(testCase, contains(string(setup.windows_commands(4)), 'Start-ArduPilotJsonSitl.ps1'));
verifyTrue(testCase, contains(join(setup.recommended_sequence, newline), "run_ardupilot_wait_for_packet.m"));
end
