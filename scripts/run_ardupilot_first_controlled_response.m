%% RUN_ARDUPILOT_FIRST_CONTROLLED_RESPONSE Выполнить первый управляемый расчетный прогон.
% Назначение:
%   После успешного взведения выполняет 20-секундный прогон обмена
%   ArduPilot и математической модели, фиксируя высоту, углы, угловые
%   скорости, ШИМ, команды частоты вращения и статус обмена.

arm_result = local_load_arm_result();
repo_root = fileparts(fileparts(mfilename('fullpath')));
live_backend_script = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

result = struct();
result.executed = false;
result.arm_result = arm_result;

if ~arm_result.arm_succeeded
    result.message = "Управляемый прогон не выполнен, так как автоматическое взведение не подтверждено.";
    assignin('base', 'ardupilot_first_controlled_response', result);

    fprintf('Первый управляемый расчетный прогон\n');
    fprintf('  выполнен                            : нет\n');
    fprintf('  причина                             : %s\n', char(arm_result.failure_reason));

    save(fullfile(reports_dir, 'task_16_first_controlled_response.mat'), 'result');
    writetable( ...
        table(string(result.message), 'VariableNames', {'message'}), ...
        fullfile(reports_dir, 'task_16_first_controlled_response.csv'));
    return;
end

assignin('base', 'ardupilot_live_backend_duration_s', 20.0);
cleanup_obj = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
run(live_backend_script);
live_result = evalin('base', 'ardupilot_json_udp_live_backend');

result.executed = true;
result.message = "Управляемый расчетный прогон выполнен.";
result.live_result = live_result;
result.valid_rx_count = double(live_result.valid_rx_count);
result.json_tx_count = double(live_result.json_tx_count);
result.response_tx_count = double(live_result.response_tx_count);
result.arducopter_alive_assumed = true;
result.state_changed_finite = local_state_changed_finite(live_result);

save(fullfile(reports_dir, 'task_16_first_controlled_response.mat'), 'result');
writetable( ...
    local_make_csv_table(live_result), ...
    fullfile(reports_dir, 'task_16_first_controlled_response.csv'));

assignin('base', 'ardupilot_first_controlled_response', result);

fprintf('Первый управляемый расчетный прогон\n');
fprintf('  выполнен                            : да\n');
fprintf('  valid_rx_count                      : %d\n', result.valid_rx_count);
fprintf('  json_tx_count                       : %d\n', result.json_tx_count);
fprintf('  response_tx_count                   : %d\n', result.response_tx_count);
fprintf('  конечный статус обмена              : %s\n', char(live_result.last_exchange_status));
fprintf('  состояние модели конечно            : %s\n', local_bool_text(result.state_changed_finite));

function arm_result = local_load_arm_result()
%LOCAL_LOAD_ARM_RESULT Прочитать результат попытки взведения.

if evalin('base', 'exist(''ardupilot_arm_attempt'', ''var'')')
    arm_result = evalin('base', 'ardupilot_arm_attempt');
    return;
end

repo_root = fileparts(fileparts(mfilename('fullpath')));
arm_attempt_path = fullfile(repo_root, 'artifacts', 'reports', 'task_16_arm_attempt.mat');
arm_response_path = fullfile(repo_root, 'artifacts', 'reports', 'task_16_arm_pwm_response.mat');

if isfile(arm_attempt_path)
    loaded = load(arm_attempt_path);
    arm_result = loaded.arm_result;
elseif isfile(arm_response_path)
    loaded = load(arm_response_path);
    arm_result = loaded.response.arm_attempt;
else
    arm_result = struct( ...
        'arm_succeeded', false, ...
        'failure_reason', "Результат попытки взведения отсутствует.");
end
end

function is_finite = local_state_changed_finite(live_result)
%LOCAL_STATE_CHANGED_FINITE Проверить конечность состояния модели.

is_finite = true;
for idx = 1:numel(live_result.state)
    state_k = live_result.state(idx);
    if any(~isfinite(state_k.p_ned_m)) ...
            || any(~isfinite(state_k.v_b_mps)) ...
            || any(~isfinite(state_k.q_nb)) ...
            || any(~isfinite(state_k.w_b_radps))
        is_finite = false;
        return;
    end
end
end

function csv_table = local_make_csv_table(live_result)
%LOCAL_MAKE_CSV_TABLE Сформировать таблицу ключевых показателей прогона.

n_rows = numel(live_result.time_s);
frame_count = arrayfun(@(s) double(s.frame_count), live_result.sitl_output);
pwm_matrix = nan(n_rows, 4);

for idx = 1:n_rows
    if live_result.sitl_output(idx).valid
        pwm_matrix(idx, :) = double(live_result.sitl_output(idx).motor_pwm_us(:)).';
    end
end

altitude_m = -arrayfun(@(s) double(s.p_ned_m(3)), live_result.state);
est_alt_m = arrayfun(@(e) double(e.alt_m), live_result.estimator);
roll_rad = arrayfun(@(e) double(e.euler_rpy_rad(1)), live_result.estimator);
pitch_rad = arrayfun(@(e) double(e.euler_rpy_rad(2)), live_result.estimator);
yaw_rad = arrayfun(@(e) double(e.euler_rpy_rad(3)), live_result.estimator);
yaw_rate_radps = arrayfun(@(s) double(s.w_b_radps(3)), live_result.state);

csv_table = table( ...
    live_result.time_s(:), ...
    altitude_m(:), ...
    est_alt_m(:), ...
    roll_rad(:), ...
    pitch_rad(:), ...
    yaw_rad(:), ...
    yaw_rate_radps(:), ...
    frame_count(:), ...
    pwm_matrix(:, 1), ...
    pwm_matrix(:, 2), ...
    pwm_matrix(:, 3), ...
    pwm_matrix(:, 4), ...
    live_result.motor_cmd_radps(:, 1), ...
    live_result.motor_cmd_radps(:, 2), ...
    live_result.motor_cmd_radps(:, 3), ...
    live_result.motor_cmd_radps(:, 4), ...
    string(live_result.exchange_status(:)), ...
    'VariableNames', { ...
        'time_s', 'altitude_m', 'est_altitude_m', 'roll_rad', ...
        'pitch_rad', 'yaw_rad', 'yaw_rate_radps', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_cmd_1_radps', 'motor_cmd_2_radps', ...
        'motor_cmd_3_radps', 'motor_cmd_4_radps', ...
        'exchange_status'});
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Преобразовать логический признак в строку.

if flag_value
    text_value = 'да';
else
    text_value = 'нет';
end
end
