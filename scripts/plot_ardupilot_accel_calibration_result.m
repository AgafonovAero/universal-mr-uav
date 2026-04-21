%% PLOT_ARDUPILOT_ACCEL_CALIBRATION_RESULT Построить графики TASK-21.
% Назначение:
%   Формирует графики по результатам диагностики и калибровки акселерометра
%   `ArduPilot` в режиме внешней `MATLAB`-модели через `JSON/UDP`.
%
% Входы:
%   none
%
% Выходы:
%   task_21_accel_calibration_plots - структура с путями к рисункам в base workspace
%
% Единицы измерения:
%   ШИМ - микросекунды;
%   команды частоты вращения - рад/с.
%
% Допущения:
%   CSV-отчеты TASK-21 уже сформированы предыдущими сценариями.

repo_root = fileparts(fileparts(mfilename('fullpath')));
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
figures_dir = fullfile(repo_root, 'artifacts', 'figures');

params_csv = fullfile(reports_dir, 'task_21_accel_calibration_params.csv');
compare_csv = fullfile(reports_dir, 'task_21_internal_vs_json_accel_calibration.csv');
arm_csv = fullfile(reports_dir, 'task_21_arm_after_accel_calibration.csv');

if ~isfile(params_csv) || ~isfile(compare_csv) || ~isfile(arm_csv)
    error('uav:task21:plot:MissingInputs', ...
        'Не найдены все CSV-отчеты TASK-21 для построения графиков.');
end

param_table = readtable(params_csv, 'TextType', 'string');
compare_table = readtable(compare_csv, 'TextType', 'string');
arm_table = readtable(arm_csv, 'TextType', 'string');

fig_accel = fullfile(figures_dir, 'task_21_accel_calibration_params.png');
fig_pwm = fullfile(figures_dir, 'task_21_pwm_after_accel_calibration.png');
fig_motor = fullfile(figures_dir, 'task_21_motor_commands_after_accel_calibration.png');
fig_status = fullfile(figures_dir, 'task_21_arm_status.png');

local_prepare_parent(fig_accel);

local_save_figure(fig_accel, @() local_plot_accel_params(param_table, compare_table));
local_save_figure(fig_pwm, @() local_plot_pwm_ranges(arm_table));
local_save_figure(fig_motor, @() local_plot_motor_ranges(arm_table));
local_save_figure(fig_status, @() local_plot_arm_status(arm_table));

result = struct();
result.fig_accel = string(fig_accel);
result.fig_pwm = string(fig_pwm);
result.fig_motor = string(fig_motor);
result.fig_status = string(fig_status);
assignin('base', 'task_21_accel_calibration_plots', result);

fprintf('TASK-21: построение графиков калибровки акселерометра\n');
fprintf('  график параметров калибровки           : %s\n', fig_accel);
fprintf('  график диапазона ШИМ                   : %s\n', fig_pwm);
fprintf('  график диапазона команд двигателей     : %s\n', fig_motor);
fprintf('  график статуса взведения               : %s\n', fig_status);

function local_plot_accel_params(param_table, compare_table)
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
names = ["INS_ACCOFFS_X"; "INS_ACCOFFS_Y"; "INS_ACCOFFS_Z"; ...
    "INS_ACCSCAL_X"; "INS_ACCSCAL_Y"; "INS_ACCSCAL_Z"];
json_values = local_extract_values(compare_table, names, 'json_value');
internal_values = local_extract_values(compare_table, names, 'internal_value');
bar(categorical(cellstr(names)), [json_values, internal_values], 'grouped');
grid on;
ylabel('Значение параметра');
title('Калибровка основного акселерометра: JSON-режим и штатный SITL');
legend({'JSON-режим', 'Штатный SITL'}, 'Location', 'best');

nexttile;
names2 = ["INS_ENABLE_MASK"; "INS_USE"; "INS_USE2"; "INS_ACC_ID"; "INS_ACC2_ID"; "AHRS_ORIENTATION"];
json_values2 = local_extract_values(compare_table, names2, 'json_value');
internal_values2 = local_extract_values(compare_table, names2, 'internal_value');
bar(categorical(cellstr(names2)), [json_values2, internal_values2], 'grouped');
grid on;
ylabel('Значение параметра');
title('Активные ИНС и ориентация: JSON-режим и штатный SITL');
legend({'JSON-режим', 'Штатный SITL'}, 'Location', 'best');
xtickangle(20);
end

function local_plot_pwm_ranges(arm_table)
variant_names = categorical(cellstr(arm_table.variant_name));
min_values = double(arm_table.motor_pwm_min_us);
max_values = double(arm_table.motor_pwm_max_us);
bar(variant_names, [min_values, max_values], 'grouped');
grid on;
ylabel('ШИМ, мкс');
title('Диапазон команд ШИМ после калибровки акселерометра');
legend({'Минимум', 'Максимум'}, 'Location', 'best');
end

function local_plot_motor_ranges(arm_table)
variant_names = categorical(cellstr(arm_table.variant_name));
min_values = double(arm_table.motor_cmd_min_radps);
max_values = double(arm_table.motor_cmd_max_radps);
bar(variant_names, [min_values, max_values], 'grouped');
grid on;
ylabel('Команда частоты вращения, рад/с');
title('Диапазон команд винтомоторной группы после калибровки');
legend({'Минимум', 'Максимум'}, 'Location', 'best');
end

function local_plot_arm_status(arm_table)
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
x = 1:height(arm_table);
labels = cellstr(arm_table.variant_name);

nexttile;
bar(x, [double(arm_table.valid_rx_count), double(arm_table.response_tx_count)], 'grouped');
grid on;
ylabel('Счетчик');
title('Устойчивость обмена перед попыткой взведения');
legend({'valid\_rx\_count', 'response\_tx\_count'}, 'Location', 'best');
xticks(x);
xticklabels(labels);

nexttile;
yyaxis left;
bar(x, double(arm_table.command_ack));
ylabel('COMMAND\_ACK');
yyaxis right;
plot(x, double(arm_table.arm_succeeded), 'o-', 'LineWidth', 1.5);
ylabel('arm (0/1)');
grid on;
title('Статус команды взведения после калибровки');
xticks(x);
xticklabels(labels);
end

function values = local_extract_values(compare_table, names, field_name)
values = nan(numel(names), 1);
for idx = 1:numel(names)
    row_idx = find(string(compare_table.param_name) == names(idx), 1, 'first');
    if ~isempty(row_idx)
        values(idx) = double(compare_table.(field_name)(row_idx));
    end
end
end

function local_save_figure(path_value, plotter)
fig = figure('Visible', 'off');
cleanup_obj = onCleanup(@() close(fig)); %#ok<NASGU>
plotter();
exportgraphics(fig, path_value, 'Resolution', 150);
end

function local_prepare_parent(path_value)
folder_path = fileparts(path_value);
if ~isfolder(folder_path)
    mkdir(folder_path);
end
end
