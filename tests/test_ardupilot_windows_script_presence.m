function tests = test_ardupilot_windows_script_presence
%TEST_ARDUPILOT_WINDOWS_SCRIPT_PRESENCE Проверки операторских сценариев.
% Назначение:
%   Подтверждает наличие файлов подготовки Windows/WSL и фиксирует,
%   что сценарии работают в безопасном режиме без автоматического
%   выполнения опасных действий по умолчанию.

tests = functiontests(localfunctions);
end

function testWindowsAndWslScriptsExist(testCase)
%TESTWINDOWSANDWSLSCRIPTSEXIST Проверить наличие файлов подготовки.

repoRoot = fileparts(fileparts(mfilename('fullpath')));

expectedFiles = { ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Test-ArduPilotEnvironment.ps1'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Setup-WSLForArduPilot.ps1'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Invoke-ArduPilotWslSetup.ps1'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Start-ArduPilotJsonSitl.ps1'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'wsl', 'setup_ardupilot_wsl.sh'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'wsl', 'start_arducopter_json_sitl.sh')};

for idx = 1:numel(expectedFiles)
    verifyTrue(testCase, isfile(expectedFiles{idx}));
end
end

function testPowerShellScriptsExposeSafeExecutionMode(testCase)
%TESTPOWERSHELLSCRIPTSEXPOSESAFEEXECUTIONMODE Проверить наличие -Execute.

repoRoot = fileparts(fileparts(mfilename('fullpath')));

psScripts = { ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Setup-WSLForArduPilot.ps1'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Invoke-ArduPilotWslSetup.ps1'), ...
    fullfile(repoRoot, 'tools', 'ardupilot', 'windows', 'Start-ArduPilotJsonSitl.ps1')};

for idx = 1:numel(psScripts)
    text = fileread(psScripts{idx});
    verifyTrue(testCase, contains(text, '[switch]$Execute'));
end
end

function testBashScriptsUseExternalArduPilotRoot(testCase)
%TESTBASHSCRIPTSUSEEXTERNALARDUPILOTROOT Проверить внешний каталог ArduPilot.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
setupScript = fileread(fullfile(repoRoot, 'tools', 'ardupilot', 'wsl', 'setup_ardupilot_wsl.sh'));
startScript = fileread(fullfile(repoRoot, 'tools', 'ardupilot', 'wsl', 'start_arducopter_json_sitl.sh'));

verifyTrue(testCase, contains(setupScript, '$HOME/src/ardupilot'));
verifyTrue(testCase, contains(startScript, '$HOME/src/ardupilot'));
verifyFalse(testCase, contains(setupScript, 'universal-mr-uav/ardupilot'));
verifyFalse(testCase, contains(startScript, 'universal-mr-uav/ardupilot'));
end
