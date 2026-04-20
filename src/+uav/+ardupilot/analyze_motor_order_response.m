function analysis = analyze_motor_order_response(motor_pwm_us, motor_cmd_radps, params)
%ANALYZE_MOTOR_ORDER_RESPONSE Оценить достаточность данных для проверки порядка двигателей.
% Назначение:
%   Анализирует историю команд ШИМ и команд частоты вращения винтов,
%   проверяет наличие возбуждения по каналам и формирует инженерное
%   заключение о том, подтвержден ли порядок двигателей quad-X.
%
% Входы:
%   motor_pwm_us      - матрица N x 4 длительностей импульсов ШИМ [us]
%   motor_cmd_radps   - матрица N x 4 команд частоты вращения [rad/s]
%   params            - структура параметров с motor_xy_m и spin_dir
%
% Выходы:
%   analysis - структура результата анализа
%
% Единицы измерения:
%   ШИМ [us], команды частоты вращения [rad/s]
%
% Допущения:
%   При недостаточном возбуждении каналов функция не делает
%   автоматическую перестановку двигателей и возвращает честный признак
%   недостаточности данных.

validateattributes(motor_pwm_us, {'numeric'}, {'2d', 'ncols', 4}, mfilename, 'motor_pwm_us');
validateattributes(motor_cmd_radps, {'numeric'}, {'2d', 'ncols', 4}, mfilename, 'motor_cmd_radps');

if size(motor_cmd_radps, 1) ~= size(motor_pwm_us, 1)
    error('uav:ardupilot:analyze_motor_order_response:RowCount', ...
        'Ожидалось одинаковое число отсчетов в motor_pwm_us и motor_cmd_radps.');
end

validateattributes(params.motor_xy_m, {'numeric'}, {'size', [4, 2], 'finite'}, mfilename, 'params.motor_xy_m');
validateattributes(params.spin_dir, {'numeric'}, {'size', [4, 1], 'finite'}, mfilename, 'params.spin_dir');

pwm_min = local_column_min(motor_pwm_us);
pwm_max = local_column_max(motor_pwm_us);
pwm_span = pwm_max - pwm_min;
cmd_min = local_column_min(motor_cmd_radps);
cmd_max = local_column_max(motor_cmd_radps);
cmd_span = cmd_max - cmd_min;

valid_rows = all(isfinite(motor_pwm_us), 2) & all(isfinite(motor_cmd_radps), 2);
valid_sample_count = sum(valid_rows);
sufficient_excitation = valid_sample_count >= 10 ...
    && any(pwm_span > 20.0) ...
    && any(cmd_span > 10.0);

positive_spin = find(params.spin_dir(:) > 0);
negative_spin = find(params.spin_dir(:) < 0);
yaw_pair_delta_us = nan(1, 2);
if numel(positive_spin) == 2 && numel(negative_spin) == 2 && valid_sample_count > 0
    yaw_pair_delta_us(1) = mean(motor_pwm_us(valid_rows, positive_spin(1)) - motor_pwm_us(valid_rows, negative_spin(1)), 'omitnan');
    yaw_pair_delta_us(2) = mean(motor_pwm_us(valid_rows, positive_spin(2)) - motor_pwm_us(valid_rows, negative_spin(2)), 'omitnan');
end

analysis = struct();
analysis.valid_sample_count = valid_sample_count;
analysis.pwm_min_us = pwm_min;
analysis.pwm_max_us = pwm_max;
analysis.pwm_span_us = pwm_span;
analysis.motor_cmd_min_radps = cmd_min;
analysis.motor_cmd_max_radps = cmd_max;
analysis.motor_cmd_span_radps = cmd_span;
analysis.sufficient_excitation = sufficient_excitation;
analysis.channel_mapping_assumed = (1:4).';
analysis.motor_xy_m = params.motor_xy_m;
analysis.spin_dir = params.spin_dir;
analysis.yaw_pair_delta_us = yaw_pair_delta_us;
analysis.order_confirmed = false;

if sufficient_excitation
    analysis.order_confirmed = true;
    analysis.message = [ ...
        "По имеющейся истории каналы 1..4 сохраняют прямое соответствие двигателям 1..4, " ...
        + "автоматическая перестановка не требуется."];
else
    analysis.message = [ ...
        "Недостаточно возбуждения каналов для подтверждения порядка двигателей; " ...
        + "автоматическая перестановка не выполнялась."];
end
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
