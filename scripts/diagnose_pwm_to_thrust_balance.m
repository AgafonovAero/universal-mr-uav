%% DIAGNOSE_PWM_TO_THRUST_BALANCE Построить тяговый баланс по текущему преобразованию PWM.
% Назначение:
%   Для заданных уровней ШИМ вычисляет команды частоты вращения винтов,
%   тягу одного винта, суммарную тягу винтомоторной группы и отношение
%   суммарной тяги к весу математической модели БВС.
%
% Входы:
%   none
%
% Выходы:
%   task_23_pwm_to_thrust_balance - структура результата в base workspace
%
% Единицы измерения:
%   ШИМ - микросекунды;
%   частота вращения - рад/с;
%   тяга - Н;
%   масса - кг.
%
% Допущения:
%   Используется текущее линейное преобразование `PWM -> omega` из
%   `uav.ardupilot.pwm_to_motor_radps` и текущие коэффициенты ВМГ модели.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');
log_path = fullfile(logs_dir, 'task_23_pwm_to_thrust_balance.txt');
csv_path = fullfile(reports_dir, 'task_23_pwm_to_thrust_balance.csv');
mat_path = fullfile(reports_dir, 'task_23_pwm_to_thrust_balance.mat');

local_prepare_parent(log_path);
local_prepare_parent(csv_path);

params = uav.sim.default_params_quad_x250();
cfg = uav.ardupilot.default_json_config();
motor_result = local_load_motor_output_result(repo_root);
param_row = motor_result.param_row;

pwm_grid_us = (1000:50:2000).';
row_count = numel(pwm_grid_us);
omega_one_radps = nan(row_count, 1);
thrust_one_N = nan(row_count, 1);
thrust_total_N = nan(row_count, 1);
thrust_to_weight_ratio = nan(row_count, 1);

weight_N = double(params.mass_kg) * double(params.gravity_mps2);
for idx = 1:row_count
    pwm_vector = repmat(double(pwm_grid_us(idx)), 4, 1);
    motor_cmd_radps = uav.ardupilot.pwm_to_motor_radps(pwm_vector, params, cfg);
    [thrust_each_N, ~] = uav.vmg.rotor_simple( ...
        motor_cmd_radps, ...
        params.rotor.kT_N_per_radps2, ...
        params.rotor.kQ_Nm_per_radps2);

    omega_one_radps(idx) = motor_cmd_radps(1);
    thrust_one_N(idx) = thrust_each_N(1);
    thrust_total_N(idx) = sum(thrust_each_N);
    thrust_to_weight_ratio(idx) = thrust_total_N(idx) / weight_N;
end

[~, hover_idx] = min(abs(thrust_to_weight_ratio - 1.0));
hover_pwm_us = double(pwm_grid_us(hover_idx));
hover_ratio = double(thrust_to_weight_ratio(hover_idx));

mot_hover_equivalent_pwm_us = nan;
if isfinite(param_row.MOT_PWM_MIN) && isfinite(param_row.MOT_PWM_MAX) && isfinite(param_row.MOT_THST_HOVER)
    mot_hover_equivalent_pwm_us = ...
        param_row.MOT_PWM_MIN + ...
        param_row.MOT_THST_HOVER * (param_row.MOT_PWM_MAX - param_row.MOT_PWM_MIN);
end

balance_table = table( ...
    pwm_grid_us, ...
    omega_one_radps, ...
    thrust_one_N, ...
    thrust_total_N, ...
    repmat(weight_N, row_count, 1), ...
    thrust_to_weight_ratio, ...
    'VariableNames', { ...
        'pwm_us', ...
        'motor_cmd_radps', ...
        'thrust_one_N', ...
        'thrust_total_N', ...
        'weight_N', ...
        'thrust_to_weight_ratio'});

result = struct();
result.params = params;
result.cfg = cfg;
result.motor_param_result = motor_result;
result.table = balance_table;
result.hover_pwm_us = hover_pwm_us;
result.hover_ratio = hover_ratio;
result.mot_hover_equivalent_pwm_us = mot_hover_equivalent_pwm_us;
result.at_1100_ratio = thrust_to_weight_ratio(pwm_grid_us == 1100);
result.at_1100_enough_for_liftoff = any(result.at_1100_ratio >= 1.0);
result.weight_N = weight_N;
result.max_ratio = max(thrust_to_weight_ratio);

writetable(balance_table, csv_path);
save(mat_path, 'result');
assignin('base', 'task_23_pwm_to_thrust_balance', result);
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(result));

fprintf('TASK-23: тяговый баланс PWM -> omega -> thrust\n');
fprintf('  расчетный PWM зависания [us]            : %.0f\n', result.hover_pwm_us);
fprintf('  эквивалент MOT_THST_HOVER [us]          : %.1f\n', result.mot_hover_equivalent_pwm_us);
fprintf('  отношение тяги к весу при 1100 us       : %.4f\n', result.at_1100_ratio);
fprintf('  максимум отношения тяги к весу          : %.4f\n', result.max_ratio);

function motor_result = local_load_motor_output_result(repo_root)
%LOCAL_LOAD_MOTOR_OUTPUT_RESULT Загрузить результат чтения параметров моторов.

report_path = fullfile(repo_root, 'artifacts', 'reports', 'task_23_motor_output_params.mat');
if evalin('base', 'exist(''task_23_motor_output_params'', ''var'')')
    motor_result = evalin('base', 'task_23_motor_output_params');
elseif isfile(report_path)
    loaded = load(report_path);
    motor_result = loaded.result;
else
    error('uav:task23:thrustBalance:MissingMotorReport', ...
        'Не найден результат сценария diagnose_ardupilot_motor_output_params.m.');
end
end

function text_value = local_make_log_text(result)
%LOCAL_MAKE_LOG_TEXT Сформировать журнал тягового баланса.

row = result.motor_param_result.param_row;
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-23: расчет ШИМ -> частота вращения -> тяга";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "MOT_PWM_MIN [us]: " + string(row.MOT_PWM_MIN);
lines(end + 1, 1) = "MOT_PWM_MAX [us]: " + string(row.MOT_PWM_MAX);
lines(end + 1, 1) = "MOT_SPIN_ARM [-]: " + string(row.MOT_SPIN_ARM);
lines(end + 1, 1) = "MOT_SPIN_MIN [-]: " + string(row.MOT_SPIN_MIN);
lines(end + 1, 1) = "MOT_THST_HOVER [-]: " + string(row.MOT_THST_HOVER);
lines(end + 1, 1) = "Вес модели [N]: " + sprintf('%.6f', result.weight_N);
lines(end + 1, 1) = "Расчетный PWM зависания [us]: " + sprintf('%.0f', result.hover_pwm_us);
lines(end + 1, 1) = "Отношение тяги к весу в точке зависания: " + sprintf('%.6f', result.hover_ratio);
lines(end + 1, 1) = "Эквивалент MOT_THST_HOVER [us]: " + sprintf('%.3f', result.mot_hover_equivalent_pwm_us);
lines(end + 1, 1) = "Отношение тяги к весу при 1100 us: " + sprintf('%.6f', result.at_1100_ratio);
lines(end + 1, 1) = "1100 us достаточно для отрыва: " + local_bool_text(result.at_1100_enough_for_liftoff);
if result.at_1100_enough_for_liftoff
    lines(end + 1, 1) = "Вывод: даже уровень 1100 us уже достаточен для отрыва.";
else
    lines(end + 1, 1) = "Вывод: 1100 us недостаточно для отрыва; требуется рост ШИМ выше уровня idle arm.";
end
lines(end + 1, 1) = "Максимальное отношение тяги к весу на сетке: " + sprintf('%.6f', result.max_ratio);
text_value = strjoin(lines, newline) + newline;
end

function local_prepare_parent(path_value)
%LOCAL_PREPARE_PARENT Создать каталог для файла.

parent_dir = fileparts(path_value);
if strlength(string(parent_dir)) > 0 && ~isfolder(parent_dir)
    mkdir(parent_dir);
end
end

function text_value = local_bool_text(flag_value)
%LOCAL_BOOL_TEXT Представить логический признак по-русски.

if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
