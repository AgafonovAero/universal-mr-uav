function tests = test_ardupilot_json_udp_availability
%TEST_ARDUPILOT_JSON_UDP_AVAILABILITY Проверки доступности UDP.
% Назначение:
%   Подтверждает, что функция проверки доступности транспортного уровня
%   `UDP` возвращает структурированный результат и не завершается аварийно.

tests = functiontests(localfunctions);
end

function setupOnce(~)
%SETUPONCE Подключить каталог исходного кода.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
end

function testAvailabilityReturnsStructuredResult(testCase)
%TESTAVAILABILITYRETURNSSTRUCTUREDRESULT Проверить структуру результата.

info = uav.ardupilot.json_udp_is_available();

verifyTrue(testCase, isstruct(info));
verifyTrue(testCase, isfield(info, 'is_available'));
verifyTrue(testCase, isfield(info, 'method'));
verifyTrue(testCase, isfield(info, 'message'));
verifyTrue(testCase, islogical(info.is_available));
verifyTrue(testCase, isstring(info.method) || ischar(info.method));
verifyTrue(testCase, isstring(info.message) || ischar(info.message));
end
