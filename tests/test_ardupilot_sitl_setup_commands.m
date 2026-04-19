function tests = test_ardupilot_sitl_setup_commands
%TEST_ARDUPILOT_SITL_SETUP_COMMANDS Проверки команд подготовки среды.
% Назначение:
%   Подтверждает, что команды подготовки `ArduPilot SITL` формируются без
%   ошибок и содержат ожидаемые группы инструкций.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testSetupCommandsAreFormed(testCase)
%TESTSETUPCOMMANDSAREFORMED Проверить состав структуры setup.

cfg = uav.ardupilot.default_json_config();
info = uav.ardupilot.inspect_sitl_environment(cfg);
setup = uav.ardupilot.make_sitl_setup_commands(cfg, info);

verifyTrue(testCase, isstruct(setup));
verifyTrue(testCase, isfield(setup, 'windows_commands'));
verifyTrue(testCase, isfield(setup, 'wsl_commands'));
verifyTrue(testCase, isfield(setup, 'ardupilot_clone_commands'));
verifyTrue(testCase, isfield(setup, 'ardupilot_build_commands'));
verifyTrue(testCase, isfield(setup, 'sitl_start_commands'));
verifyTrue(testCase, isfield(setup, 'notes'));

verifyTrue(testCase, isstring(setup.windows_commands));
verifyTrue(testCase, isstring(setup.wsl_commands));
verifyTrue(testCase, isstring(setup.ardupilot_clone_commands));
verifyTrue(testCase, isstring(setup.ardupilot_build_commands));
verifyTrue(testCase, isstring(setup.sitl_start_commands));
verifyTrue(testCase, isstring(setup.notes));
verifyTrue(testCase, any(contains(setup.sitl_start_commands, "JSON:")));
end
