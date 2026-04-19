function tests = test_ardupilot_make_sitl_start_command
%TEST_ARDUPILOT_MAKE_SITL_START_COMMAND Проверки команды запуска SITL.
% Назначение:
%   Подтверждает, что формируемая командная строка запуска `ArduPilot`
%   содержит указание на режим `JSON`, тип аппарата и схему рамы.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testCommandContainsJsonModel(testCase)
%TESTCOMMANDCONTAINSJSONMODEL Проверить состав командной строки.

cfg = uav.ardupilot.default_json_config();
cmd = uav.ardupilot.make_sitl_start_command(cfg);

verifyTrue(testCase, isstruct(cmd));
verifyTrue(testCase, contains(string(cmd.command_text), "sim_vehicle.py"));
verifyTrue(testCase, contains(string(cmd.command_text), "ArduCopter"));
verifyTrue(testCase, contains(string(cmd.command_text), "-f quad"));
verifyTrue(testCase, contains(string(cmd.command_text), "JSON:"));
verifyGreaterThan(testCase, strlength(string(cmd.message)), 0);
verifyGreaterThan(testCase, strlength(string(cmd.wsl_command_text)), 0);
end
