%% COMPARE_JSON_ACCEL_WITH_ARDUPILOT_LOG Сравнить JSON и внутренние IMU ArduPilot.
% Назначение:
%   Сопоставляет первые JSON-кадры `MATLAB`-модели с внутренними
%   акселерометрами `ArduPilot`, извлеченными из `DataFlash`, и оценивает
%   расхождение между внешним `imu.accel_body` и экземплярами `IMU`.
%
% Входы:
%   none
%
% Выходы:
%   task_20_json_vs_ardupilot_accel - структура результата в base workspace
%
% Единицы измерения:
%   ускорение - в единицах, сохраненных источниками;
%   время - секунды
%
% Допущения:
%   `task_19_first_json_frames.jsonl` и `task_20_accel_instances.csv`
%   уже созданы.

repo_root = fileparts(fileparts(mfilename('fullpath')));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

jsonl_path = fullfile(reports_dir, 'task_19_first_json_frames.jsonl');
accel_csv_path = fullfile(reports_dir, 'task_20_accel_instances.csv');
csv_path = fullfile(reports_dir, 'task_20_json_vs_ardupilot_accel.csv');
mat_path = fullfile(reports_dir, 'task_20_json_vs_ardupilot_accel.mat');
log_path = fullfile(logs_dir, 'task_20_compare_json_accel_with_ardupilot_log.txt');

if ~isfile(jsonl_path)
    error('uav:task20:compareAccel:MissingJsonl', ...
        'Не найден файл первых JSON-кадров: %s', jsonl_path);
end

if ~isfile(accel_csv_path)
    error('uav:task20:compareAccel:MissingAccelCsv', ...
        'Не найден CSV с экземплярами акселерометров: %s', accel_csv_path);
end

json_frames = local_read_jsonl(jsonl_path);
accel_table = readtable(accel_csv_path, 'TextType', 'string');
accel_table = accel_table(strcmp(accel_table.msg_type, "IMU"), :);

instance_ids = unique(accel_table.instance_id);
instance_ids = instance_ids(:).';
json_count = numel(json_frames);
sample_count = min(json_count, 200);
rel_time_json = [json_frames(1:sample_count).timestamp_s]';
rel_time_json = rel_time_json - rel_time_json(1);

result_table = table();
result_table.index = (1:sample_count).';
result_table.time_s = rel_time_json;
result_table.json_acc_x = [json_frames(1:sample_count).accel_x]';
result_table.json_acc_y = [json_frames(1:sample_count).accel_y]';
result_table.json_acc_z = [json_frames(1:sample_count).accel_z]';
result_table.json_acc_norm = [json_frames(1:sample_count).accel_norm]';

max_instance_diff = nan(sample_count, 1);
max_json_to_instance_diff = nan(sample_count, 1);

for inst = instance_ids
    rows = accel_table(accel_table.instance_id == inst, :);
    if isempty(rows)
        continue;
    end

    rel_time_imu = double(rows.time_s) - double(rows.time_s(1));
    acc_x = nan(sample_count, 1);
    acc_y = nan(sample_count, 1);
    acc_z = nan(sample_count, 1);
    acc_norm = nan(sample_count, 1);
    for k = 1:sample_count
        [~, idx_min] = min(abs(rel_time_imu - rel_time_json(k)));
        acc_x(k) = rows.AccX(idx_min);
        acc_y(k) = rows.AccY(idx_min);
        acc_z(k) = rows.AccZ(idx_min);
        acc_norm(k) = rows.accel_norm(idx_min);
    end

    suffix = sprintf('_imu%d', inst);
    result_table.(['acc_x', suffix]) = acc_x;
    result_table.(['acc_y', suffix]) = acc_y;
    result_table.(['acc_z', suffix]) = acc_z;
    result_table.(['acc_norm', suffix]) = acc_norm;
end

for k = 1:sample_count
    json_vec = [result_table.json_acc_x(k), result_table.json_acc_y(k), result_table.json_acc_z(k)];
    per_instance_diff = [];
    instance_norms = [];
    for inst = instance_ids
        col_x = sprintf('acc_x_imu%d', inst);
        col_y = sprintf('acc_y_imu%d', inst);
        col_z = sprintf('acc_z_imu%d', inst);
        if ismember(col_x, result_table.Properties.VariableNames)
            imu_vec = [result_table.(col_x)(k), result_table.(col_y)(k), result_table.(col_z)(k)];
            if all(isfinite(imu_vec))
                per_instance_diff(end + 1, 1) = norm(json_vec - imu_vec); %#ok<AGROW>
                instance_norms(end + 1, 1) = norm(imu_vec); %#ok<AGROW>
            end
        end
    end
    if ~isempty(per_instance_diff)
        max_json_to_instance_diff(k) = max(per_instance_diff);
    end
    if numel(instance_norms) >= 2
        max_instance_diff(k) = max(instance_norms) - min(instance_norms);
    end
end

result_table.max_json_to_instance_diff = max_json_to_instance_diff;
result_table.max_instance_norm_difference = max_instance_diff;

summary = struct();
summary.instance_ids = instance_ids;
summary.sample_count = sample_count;
summary.max_json_to_instance_diff = max(max_json_to_instance_diff, [], 'omitnan');
summary.max_instance_norm_difference = max(max_instance_diff, [], 'omitnan');
summary.difference_gt_1_mps2 = summary.max_json_to_instance_diff > 1.0 || summary.max_instance_norm_difference > 1.0;
summary.jsonl_path = string(jsonl_path);
summary.accel_csv_path = string(accel_csv_path);
summary.csv_path = string(csv_path);

writetable(result_table, csv_path);
save(mat_path, 'result_table', 'summary');
assignin('base', 'task_20_json_vs_ardupilot_accel', struct('table', result_table, 'summary', summary));
uav.ardupilot.write_utf8_text_file(log_path, local_make_log_text(summary));

fprintf('Сравнение JSON accel и DataFlash IMU TASK-20\n');
fprintf('  instances                              : %s\n', char(join(string(summary.instance_ids), ", ")));
fprintf('  max json-to-instance diff              : %.6f\n', summary.max_json_to_instance_diff);
fprintf('  max instance norm diff                 : %.6f\n', summary.max_instance_norm_difference);

function frames = local_read_jsonl(file_path)
lines = splitlines(string(fileread(file_path)));
lines = strtrim(lines);
lines(lines == "") = [];

frames = repmat(struct( ...
    'timestamp_s', nan, ...
    'accel_x', nan, ...
    'accel_y', nan, ...
    'accel_z', nan, ...
    'accel_norm', nan), numel(lines), 1);

for idx = 1:numel(lines)
    item = jsondecode(char(lines(idx)));
    accel = double(item.imu.accel_body(:));
    frames(idx).timestamp_s = double(item.timestamp);
    frames(idx).accel_x = accel(1);
    frames(idx).accel_y = accel(2);
    frames(idx).accel_z = accel(3);
    frames(idx).accel_norm = norm(accel);
end
end

function text_value = local_make_log_text(summary)
lines = strings(0, 1);
lines(end + 1, 1) = "TASK-20: сравнение JSON accel_body и внутренних акселерометров ArduPilot";
lines(end + 1, 1) = "============================================================";
lines(end + 1, 1) = "JSONL: " + summary.jsonl_path;
lines(end + 1, 1) = "ACC CSV: " + summary.accel_csv_path;
lines(end + 1, 1) = "Instances: " + join(string(summary.instance_ids), ", ");
lines(end + 1, 1) = "Max JSON-to-instance diff: " + sprintf('%.6f', summary.max_json_to_instance_diff);
lines(end + 1, 1) = "Max instance norm diff: " + sprintf('%.6f', summary.max_instance_norm_difference);
lines(end + 1, 1) = "Difference > 1 m/s^2: " + local_bool_text(summary.difference_gt_1_mps2);
text_value = strjoin(lines, newline) + newline;
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
