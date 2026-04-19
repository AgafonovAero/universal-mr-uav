function tests = test_run_ardupilot_json_udp_selfcheck
%TEST_RUN_ARDUPILOT_JSON_UDP_SELFCHECK Проверки самопроверки TASK-11.
% Назначение:
%   Подтверждает, что сценарий самопроверки формирует структуру результата
%   и не требует установленного `ArduPilot`.

tests = functiontests(localfunctions);
end

function testSelfcheckScriptReturnsStructuredResult(testCase)
%TESTSELFCHECKSCRIPTRETURNSSTRUCTUREDRESULT Проверить результат сценария.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(repoRoot, 'scripts', 'bootstrap_project.m'));
run(fullfile(repoRoot, 'scripts', 'run_ardupilot_json_udp_selfcheck.m'));

result = evalin('base', 'ardupilot_json_udp_selfcheck');

verifyTrue(testCase, isstruct(result));
verifyTrue(testCase, isfield(result, 'availability'));
verifyTrue(testCase, isfield(result, 'decoded_output'));
verifyTrue(testCase, isfield(result, 'motor_cmd_radps'));
verifyTrue(testCase, isfield(result, 'udp_exchange_ok'));
verifyTrue(testCase, result.decoded_output.valid);
verifyGreaterThan(testCase, norm(result.motor_cmd_radps), 0.0);
end
