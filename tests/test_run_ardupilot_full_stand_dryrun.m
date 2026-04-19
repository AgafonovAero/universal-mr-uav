function tests = test_run_ardupilot_full_stand_dryrun
%TEST_RUN_ARDUPILOT_FULL_STAND_DRYRUN Проверка dry-run полного стенда TASK-14.
% Назначение:
%   Подтверждает, что однокнопочный сценарий стенда выполняется в
%   безопасном режиме без административных действий и возвращает
%   структурированный результат.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталоги исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'scripts'));
end

function testFullStandDryRunDoesNotFail(testCase)
%TESTFULLSTANDDRYRUNDOESNOTFAIL Проверить безопасный запуск dry-run.

repoRoot = fileparts(fileparts(mfilename('fullpath')));

cfg = uav.setup.default_stand_config();
cfg.execute_install = false;
cfg.execute_start = false;
cfg.execute_stop = false;
cfg.launch_mission_planner = false;
cfg.launch_qgroundcontrol = false;
cfg.report_summary = "artifacts/reports/task_14_summary_ru.md";

outputText = evalc('result = OneKeyArduPilotStand("full", cfg);'); %#ok<NASGU>

verifyTrue(testCase, exist('result', 'var') == 1);
verifyTrue(testCase, isstruct(result));
verifyTrue(testCase, isfield(result, 'check_result'));
verifyTrue(testCase, isfield(result, 'install_result'));
verifyTrue(testCase, result.requires_operator_action);
verifyTrue(testCase, contains(string(outputText), "OneKeyArduPilotStand(full)"));
verifyTrue(testCase, contains(string(outputText), "оператора") ...
    || contains(string(outputText), "оператор"));

fullLogPath = fullfile(repoRoot, char(cfg.log_full));
verifyTrue(testCase, isfile(fullLogPath));
end
