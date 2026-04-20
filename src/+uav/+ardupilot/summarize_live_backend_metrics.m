function metrics = summarize_live_backend_metrics(live_result)
%SUMMARIZE_LIVE_BACKEND_METRICS Свести метрики длительного обмена JSON/UDP.
% Назначение:
%   Формирует компактную структуру инженерных метрик по результату
%   сценария `run_ardupilot_json_udp_live_backend.m`. Сводка используется
%   в TASK-17 для оценки фактической частоты обмена, устойчивости приема
%   пакетов и готовности данных к попытке взведения `ArduPilot`.
%
% Входы:
%   live_result - структура результата длительного обмена
%
% Выходы:
%   metrics - скалярная структура со счетчиками, частотами, периодами,
%             временами отдельных операций и последними диагностическими
%             признаками обмена
%
% Единицы измерения:
%   время - секунды;
%   частота - герцы;
%   ШИМ - микросекунды;
%   команды частоты вращения винтов - рад/с
%
% Допущения:
%   Используются только уже рассчитанные истории результата обмена;
%   дополнительные вызовы к средству UDP не выполняются.

arguments
    live_result (1, 1) struct
end

metrics = struct();
metrics.valid_rx_count = local_read_numeric_field(live_result, 'valid_rx_count', 0);
metrics.json_tx_count = local_read_numeric_field(live_result, 'json_tx_count', 0);
metrics.response_tx_count = local_read_numeric_field(live_result, 'response_tx_count', 0);
metrics.json_probe_tx_count = local_read_numeric_field(live_result, 'json_probe_tx_count', 0);
metrics.invalid_rx_count = local_read_numeric_field(live_result, 'invalid_rx_count', 0);
metrics.udp_rx_datagram_count = local_read_numeric_field(live_result, 'rx_datagram_count', 0);

metrics.last_sender_address = local_read_string_field(live_result, 'last_sender_address', "");
metrics.last_sender_port = local_read_numeric_field(live_result, 'last_sender_port', 0);
metrics.last_magic = local_read_numeric_field(live_result, 'last_magic', 0);
metrics.last_frame_count = local_read_numeric_field(live_result, 'last_frame_count', 0);
metrics.last_exchange_status = local_read_string_field(live_result, 'last_exchange_status', "");

metrics.last_pwm_us = local_read_vector_field(live_result, 'last_pwm_us', nan(4, 1));
metrics.last_motor_cmd_radps = local_read_vector_field(live_result, 'last_motor_cmd_radps', nan(4, 1));

metrics.quat_true_finite = logical(local_read_numeric_field(live_result, 'quat_true_finite', true));
metrics.quat_est_finite = logical(local_read_numeric_field(live_result, 'quat_est_finite', true));

metrics.valid_rx_rate_hz = local_rate_from_timestamps(live_result, 'valid_rx_timestamp_s', metrics.valid_rx_count);
metrics.json_tx_rate_hz = local_rate_from_timestamps(live_result, 'json_tx_timestamp_s', metrics.json_tx_count);
metrics.response_tx_rate_hz = local_rate_from_timestamps(live_result, 'response_tx_timestamp_s', metrics.response_tx_count);

[metrics.valid_rx_period_mean_s, metrics.valid_rx_period_p95_s] = ...
    local_period_stats(live_result, 'valid_rx_timestamp_s');
[metrics.json_tx_period_mean_s, metrics.json_tx_period_p95_s] = ...
    local_period_stats(live_result, 'json_tx_timestamp_s');
[metrics.response_tx_period_mean_s, metrics.response_tx_period_p95_s] = ...
    local_period_stats(live_result, 'response_tx_timestamp_s');

metrics.model_step_elapsed_mean_s = local_mean_field(live_result, 'model_step_elapsed_s');
metrics.json_encode_elapsed_mean_s = local_mean_field(live_result, 'json_encode_elapsed_s');
metrics.udp_step_elapsed_mean_s = local_mean_field(live_result, 'udp_step_elapsed_s');
metrics.udp_read_elapsed_mean_s = local_mean_field(live_result, 'udp_read_elapsed_s');
metrics.udp_write_elapsed_mean_s = local_mean_field(live_result, 'udp_write_elapsed_s');
metrics.loop_elapsed_mean_s = local_mean_field(live_result, 'loop_elapsed_s');

metrics.frame_count_delta = local_frame_count_delta(live_result);
metrics.processed_sample_count = local_processed_sample_count(live_result);
end

function value = local_read_numeric_field(data, field_name, default_value)
%LOCAL_READ_NUMERIC_FIELD Прочитать числовое поле или вернуть значение по умолчанию.

if isfield(data, field_name) && ~isempty(data.(field_name))
    value = double(data.(field_name));
else
    value = double(default_value);
end
end

function value = local_read_string_field(data, field_name, default_value)
%LOCAL_READ_STRING_FIELD Прочитать строковое поле или вернуть значение по умолчанию.

if isfield(data, field_name) && ~isempty(data.(field_name))
    value = string(data.(field_name));
else
    value = string(default_value);
end
end

function value = local_read_vector_field(data, field_name, default_value)
%LOCAL_READ_VECTOR_FIELD Прочитать векторное поле или вернуть значение по умолчанию.

if isfield(data, field_name) && ~isempty(data.(field_name))
    value = double(data.(field_name)(:));
else
    value = double(default_value(:));
end
end

function rate_hz = local_rate_from_timestamps(data, field_name, count_value)
%LOCAL_RATE_FROM_TIMESTAMPS Оценить частоту по временным отметкам.

rate_hz = 0.0;
if ~isfield(data, field_name)
    return;
end

timestamps = double(data.(field_name)(:));
timestamps = timestamps(isfinite(timestamps));
if numel(timestamps) >= 2
    duration_s = timestamps(end) - timestamps(1);
    if duration_s > 0.0
        rate_hz = double(numel(timestamps) - 1) / duration_s;
        return;
    end
end

time_series = [];
if isfield(data, 'time_s')
    time_series = double(data.time_s(:));
    time_series = time_series(isfinite(time_series));
end

if numel(time_series) >= 2
    duration_s = time_series(end) - time_series(1);
    if duration_s > 0.0
        rate_hz = max(double(count_value) - 1.0, 0.0) / duration_s;
    end
end
end

function [period_mean_s, period_p95_s] = local_period_stats(data, field_name)
%LOCAL_PERIOD_STATS Вернуть средний период и 95-процентиль по временным отметкам.

period_mean_s = nan;
period_p95_s = nan;

if ~isfield(data, field_name)
    return;
end

timestamps = double(data.(field_name)(:));
timestamps = timestamps(isfinite(timestamps));
if numel(timestamps) < 2
    return;
end

periods = diff(timestamps);
periods = periods(isfinite(periods) & periods >= 0.0);
if isempty(periods)
    return;
end

period_mean_s = mean(periods);
period_p95_s = prctile(periods, 95);
end

function value = local_mean_field(data, field_name)
%LOCAL_MEAN_FIELD Вернуть среднее по полю массива с пропуском NaN.

value = nan;

if ~isfield(data, field_name)
    return;
end

samples = double(data.(field_name)(:));
samples = samples(isfinite(samples));
if isempty(samples)
    return;
end

value = mean(samples);
end

function delta_value = local_frame_count_delta(data)
%LOCAL_FRAME_COUNT_DELTA Вычислить приращение счетчика кадров SITL.

delta_value = 0;

if ~isfield(data, 'sitl_output') || isempty(data.sitl_output)
    return;
end

frame_counts = arrayfun(@(item) double(item.frame_count), data.sitl_output(:));
frame_counts = frame_counts(isfinite(frame_counts));
if isempty(frame_counts)
    return;
end

delta_value = frame_counts(end) - frame_counts(1);
end

function count_value = local_processed_sample_count(data)
%LOCAL_PROCESSED_SAMPLE_COUNT Вернуть количество обработанных шагов обмена.

count_value = 0;

if isfield(data, 'time_s') && ~isempty(data.time_s)
    count_value = numel(data.time_s);
end
end
