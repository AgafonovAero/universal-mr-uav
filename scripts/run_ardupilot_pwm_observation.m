%% RUN_ARDUPILOT_PWM_OBSERVATION Наблюдать команды ШИМ ArduPilot в течение 20 секунд.
% Назначение:
%   Выполняет 20-секундный прогон обмена с ArduPilot, сохраняет историю
%   ШИМ, команд частоты вращения винтов, состояния и оценивания состояния,
%   а также формирует таблицу для последующего анализа.
%
% Входы:
%   none
%
% Выходы:
%   ardupilot_pwm_observation - структура результата в базовом рабочем
%   пространстве MATLAB
%
% Единицы измерения:
%   время - секунды, ШИМ - микросекунды, команды частоты вращения - рад/с
%
% Допущения:
%   ArduPilot SITL уже запущен и поддерживает устойчивый обмен по JSON/UDP.

repo_root = fileparts(fileparts(mfilename('fullpath')));
live_backend_script = fullfile(repo_root, 'scripts', 'run_ardupilot_json_udp_live_backend.m');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

assignin('base', 'ardupilot_live_backend_duration_s', 20.0);
cleanup_obj = onCleanup(@() evalin('base', 'clear(''ardupilot_live_backend_duration_s'');')); %#ok<NASGU>
run(live_backend_script);
live_result = evalin('base', 'ardupilot_json_udp_live_backend');

observation = local_build_observation(live_result);

save(fullfile(reports_dir, 'task_16_pwm_observation.mat'), 'observation');
writetable(observation.csv_table, fullfile(reports_dir, 'task_16_pwm_observation.csv'));

assignin('base', 'ardupilot_pwm_observation', observation);

fprintf('Наблюдение ШИМ ArduPilot\n');
fprintf('  valid_rx_count                        : %d\n', observation.valid_rx_count);
fprintf('  json_tx_count                         : %d\n', observation.json_tx_count);
fprintf('  response_tx_count                     : %d\n', observation.response_tx_count);
fprintf('  last_frame_count                      : %d\n', observation.last_frame_count);
fprintf('  диапазон motor_pwm_us [us]            : [%s] .. [%s]\n', ...
    local_format_vector(observation.motor_pwm_min_us), ...
    local_format_vector(observation.motor_pwm_max_us));
fprintf('  диапазон motor_cmd_radps [rad/s]      : [%s] .. [%s]\n', ...
    local_format_vector(observation.motor_cmd_min_radps), ...
    local_format_vector(observation.motor_cmd_max_radps));
fprintf('  итоговый статус                       : %s\n', char(observation.last_exchange_status));

function observation = local_build_observation(live_result)
%LOCAL_BUILD_OBSERVATION Сформировать структуру наблюдения из результата обмена.

n_samples = numel(live_result.time_s);
frame_count = zeros(n_samples, 1);
motor_pwm_us = nan(n_samples, 4);

for idx = 1:n_samples
    frame_count(idx) = double(live_result.sitl_output(idx).frame_count);
    if live_result.sitl_output(idx).valid
        motor_pwm_us(idx, :) = double(live_result.sitl_output(idx).motor_pwm_us(:)).';
    end
end

state = live_result.state;
estimator = live_result.estimator;
state_z_ned_m = arrayfun(@(s) double(s.p_ned_m(3)), state);
est_alt_m = arrayfun(@(e) double(e.alt_m), estimator);

csv_table = table( ...
    live_result.time_s(:), ...
    frame_count(:), ...
    motor_pwm_us(:, 1), ...
    motor_pwm_us(:, 2), ...
    motor_pwm_us(:, 3), ...
    motor_pwm_us(:, 4), ...
    live_result.motor_cmd_radps(:, 1), ...
    live_result.motor_cmd_radps(:, 2), ...
    live_result.motor_cmd_radps(:, 3), ...
    live_result.motor_cmd_radps(:, 4), ...
    state_z_ned_m(:), ...
    est_alt_m(:), ...
    string(live_result.exchange_status(:)), ...
    'VariableNames', { ...
        'time_s', 'frame_count', ...
        'pwm_1_us', 'pwm_2_us', 'pwm_3_us', 'pwm_4_us', ...
        'motor_cmd_1_radps', 'motor_cmd_2_radps', 'motor_cmd_3_radps', 'motor_cmd_4_radps', ...
        'state_z_ned_m', 'est_alt_m', 'exchange_status'});

observation = struct();
observation.time_s = live_result.time_s(:);
observation.frame_count = frame_count(:);
observation.pwm_us = motor_pwm_us;
observation.motor_pwm_us = motor_pwm_us;
observation.motor_cmd_radps = live_result.motor_cmd_radps;
observation.state = state;
observation.estimator = estimator;
observation.exchange_status = string(live_result.exchange_status(:));
observation.exchange_diag = live_result.exchange_diag;
observation.sitl_output = live_result.sitl_output;
observation.valid_rx_count = double(live_result.valid_rx_count);
observation.json_tx_count = double(live_result.json_tx_count);
observation.response_tx_count = double(live_result.response_tx_count);
observation.last_frame_count = double(live_result.last_frame_count);
observation.last_exchange_status = string(live_result.last_exchange_status);
observation.motor_pwm_min_us = local_column_min(motor_pwm_us);
observation.motor_pwm_max_us = local_column_max(motor_pwm_us);
observation.motor_cmd_min_radps = min(live_result.motor_cmd_radps, [], 1, 'omitnan');
observation.motor_cmd_max_radps = max(live_result.motor_cmd_radps, [], 1, 'omitnan');
observation.csv_table = csv_table;
end

function values = local_column_min(matrix_value)
%LOCAL_COLUMN_MIN Вернуть минимумы по столбцам с пропуском NaN.

values = nan(1, size(matrix_value, 2));
for idx = 1:size(matrix_value, 2)
    column_value = matrix_value(:, idx);
    column_value = column_value(isfinite(column_value));
    if ~isempty(column_value)
        values(idx) = min(column_value);
    end
end
end

function values = local_column_max(matrix_value)
%LOCAL_COLUMN_MAX Вернуть максимумы по столбцам с пропуском NaN.

values = nan(1, size(matrix_value, 2));
for idx = 1:size(matrix_value, 2)
    column_value = matrix_value(:, idx);
    column_value = column_value(isfinite(column_value));
    if ~isempty(column_value)
        values(idx) = max(column_value);
    end
end
end

function text_value = local_format_vector(vec)
%LOCAL_FORMAT_VECTOR Сформировать строку вектора.

text_value = sprintf('%.6f ', vec(:));
text_value = strtrim(text_value);
end
