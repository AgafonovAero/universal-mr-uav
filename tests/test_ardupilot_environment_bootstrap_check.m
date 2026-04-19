function tests = test_ardupilot_environment_bootstrap_check
%TEST_ARDUPILOT_ENVIRONMENT_BOOTSTRAP_CHECK Проверка сводного сценария.
% Назначение:
%   Подтверждает, что сценарий общей проверки подготовки среды
%   выполняется без ошибки и возвращает структуру результата.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testBootstrapCheckRunsWithoutError(testCase)
%TESTBOOTSTRAPCHECKRUNSWITHOUTERROR Проверить успешный запуск сценария.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
scriptPath = fullfile(repoRoot, 'scripts', 'run_ardupilot_environment_bootstrap_check.m');

cleanupObj = onCleanup(@() evalin('base', 'clear(''ardupilot_environment_bootstrap_check'')')); %#ok<NASGU>
outputText = evalc(sprintf('run(''%s'');', scriptPath));
result = evalin('base', 'ardupilot_environment_bootstrap_check');

verifyTrue(testCase, isstruct(result));
verifyTrue(testCase, isfield(result, 'env_info'));
verifyTrue(testCase, isfield(result, 'setup'));
verifyTrue(testCase, contains(string(outputText), "ArduPilot SITL environment bootstrap check"));
end
