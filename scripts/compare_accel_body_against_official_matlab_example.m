%% COMPARE_ACCEL_BODY_AGAINST_OFFICIAL_MATLAB_EXAMPLE Сравнить accel_body с официальным примером.
% Назначение:
%   Сопоставляет текущее представление accel_body с формулой официального
%   MATLAB-примера ArduPilot для типовых состояний покоя, висения и
%   вертикального ускорения.

repo_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(repo_root, 'scripts', 'bootstrap_project.m'));
logs_dir = fullfile(repo_root, 'artifacts', 'logs');
reports_dir = fullfile(repo_root, 'artifacts', 'reports');

if ~isfolder(logs_dir)
    mkdir(logs_dir);
end
if ~isfolder(reports_dir)
    mkdir(reports_dir);
end

log_path = fullfile(logs_dir, 'task_25_accel_body_official_comparison.txt');
csv_path = fullfile(reports_dir, 'task_25_accel_body_official_comparison.csv');
mat_path = fullfile(reports_dir, 'task_25_accel_body_official_comparison.mat');

params = uav.sim.make_deterministic_demo_params();
cfg = uav.ardupilot.default_json_config();
base_state = uav.core.state_unpack(params.demo.initial_state_plant);

cases = local_build_cases(base_state, params);
row_count = numel(cases);

case_name = strings(row_count, 1);
roll_rad = nan(row_count, 1);
pitch_rad = nan(row_count, 1);
yaw_rad = nan(row_count, 1);
current_ax = nan(row_count, 1);
current_ay = nan(row_count, 1);
current_az = nan(row_count, 1);
official_ax = nan(row_count, 1);
official_ay = nan(row_count, 1);
official_az = nan(row_count, 1);
delta_norm = nan(row_count, 1);
ground_clamped = false(row_count, 1);
diagnostic_ground_clamped = false(row_count, 1);

log_lines = strings(0, 1);
log_lines(end + 1, 1) = "Сравнение accel_body с официальным MATLAB-примером ArduPilot";
log_lines(end + 1, 1) = "============================================================";

for idx = 1:row_count
    state = cases(idx).state;
    snapshot = struct('forces_b_N', cases(idx).forces_b_N);
    sensors = struct();
    sensors.imu = struct( ...
        'accel_b_mps2', cases(idx).sensor_accel_b_mps2, ...
        'gyro_b_radps', state.w_b_radps);

    [sample, diag] = uav.ardupilot.convert_state_to_official_json( ...
        state, sensors, 0.0, params, cfg, 'SnapshotDiag', snapshot);
    official_accel_b_mps2 = local_compute_official_reference(state, cases(idx).forces_b_N, params);

    euler_rpy = local_quat_to_euler_rpy(state.q_nb);

    case_name(idx) = cases(idx).name;
    roll_rad(idx) = euler_rpy(1);
    pitch_rad(idx) = euler_rpy(2);
    yaw_rad(idx) = euler_rpy(3);
    current_ax(idx) = sample.imu.accel_body(1);
    current_ay(idx) = sample.imu.accel_body(2);
    current_az(idx) = sample.imu.accel_body(3);
    official_ax(idx) = official_accel_b_mps2(1);
    official_ay(idx) = official_accel_b_mps2(2);
    official_az(idx) = official_accel_b_mps2(3);
    delta_norm(idx) = norm(sample.imu.accel_body(:) - official_accel_b_mps2(:));
    ground_clamped(idx) = logical(diag.accel_diag.ground_clamped);
    diagnostic_ground_clamped(idx) = logical(diag.accel_diag.diagnostic_ground_clamped);

    log_lines(end + 1, 1) = sprintf('%s: current=[%.6f %.6f %.6f], official=[%.6f %.6f %.6f], delta_norm=%.6f, ground_clamped=%s', ...
        char(cases(idx).name), ...
        current_ax(idx), current_ay(idx), current_az(idx), ...
        official_ax(idx), official_ay(idx), official_az(idx), ...
        delta_norm(idx), local_bool_text(ground_clamped(idx)));
end

result_table = table( ...
    case_name, ...
    roll_rad, pitch_rad, yaw_rad, ...
    current_ax, current_ay, current_az, ...
    official_ax, official_ay, official_az, ...
    delta_norm, ...
    ground_clamped, ...
    diagnostic_ground_clamped);

result = struct();
result.table = result_table;
result.cases = cases;

writetable(result_table, csv_path);
save(mat_path, 'result');
uav.ardupilot.write_utf8_text_file(log_path, strjoin(log_lines, newline) + newline);
assignin('base', 'task_25_accel_body_official_comparison', result);

fprintf('Сравнение accel_body с официальным MATLAB-примером ArduPilot\n');
fprintf('  число сценариев                         : %d\n', row_count);
fprintf('  максимальное расхождение по норме [м/с^2]: %.6f\n', max(delta_norm));
fprintf('  отдельное наземное ограничение обнаружено : %s\n', local_bool_text(any(ground_clamped)));

function cases = local_build_cases(base_state, params)
g = params.gravity_mps2;
mass = params.mass_kg;

cases = repmat(struct( ...
    'name', "", ...
    'state', base_state, ...
    'forces_b_N', zeros(3, 1), ...
    'sensor_accel_b_mps2', zeros(3, 1)), 5, 1);

state_ground = base_state;
state_ground.p_ned_m = [0.0; 0.0; 0.0];
cases(1).name = "ground_rest";
cases(1).state = state_ground;
cases(1).forces_b_N = [0.0; 0.0; 0.0];
cases(1).sensor_accel_b_mps2 = [0.0; 0.0; 0.0];

state_hover = base_state;
state_hover.p_ned_m = [0.0; 0.0; -1.0];
cases(2).name = "hover";
cases(2).state = state_hover;
cases(2).forces_b_N = [0.0; 0.0; -mass * g];
cases(2).sensor_accel_b_mps2 = cases(2).forces_b_N ./ mass;

state_climb = base_state;
state_climb.p_ned_m = [0.0; 0.0; -1.0];
cases(3).name = "vertical_accel_up";
cases(3).state = state_climb;
cases(3).forces_b_N = [0.0; 0.0; -mass * (g + 1.0)];
cases(3).sensor_accel_b_mps2 = cases(3).forces_b_N ./ mass;

state_descend = base_state;
state_descend.p_ned_m = [0.0; 0.0; -1.0];
cases(4).name = "vertical_accel_down";
cases(4).state = state_descend;
cases(4).forces_b_N = [0.0; 0.0; -mass * (g - 1.0)];
cases(4).sensor_accel_b_mps2 = cases(4).forces_b_N ./ mass;

state_tilt = base_state;
state_tilt.p_ned_m = [0.0; 0.0; -1.0];
state_tilt.q_nb = local_euler_to_quat_rpy([deg2rad(5.0); deg2rad(-3.0); 0.0]);
cases(5).name = "small_roll_pitch_hover";
cases(5).state = state_tilt;
cases(5).forces_b_N = -mass .* (uav.core.quat_to_dcm(state_tilt.q_nb).' * [0.0; 0.0; g]);
cases(5).sensor_accel_b_mps2 = cases(5).forces_b_N ./ mass;
end

function accel_body_mps2 = local_compute_official_reference(state, forces_b_N, params)
c_nb = uav.core.quat_to_dcm(state.q_nb);
gravity_ned_mps2 = [0.0; 0.0; double(params.gravity_mps2)];
accel_body_linear_mps2 = double(forces_b_N(:)) ./ double(params.mass_kg);
accel_ned_mps2 = c_nb * accel_body_linear_mps2 + gravity_ned_mps2;

if double(state.p_ned_m(3)) >= 0.0 && accel_ned_mps2(3) > 0.0
    accel_ned_mps2(3) = 0.0;
end

accel_body_mps2 = c_nb.' * (accel_ned_mps2 + [0.0; 0.0; -double(params.gravity_mps2)]);
end

function q_nb = local_euler_to_quat_rpy(euler_rpy_rad)
half_rpy = 0.5 .* euler_rpy_rad(:);
cr = cos(half_rpy(1));
sr = sin(half_rpy(1));
cp = cos(half_rpy(2));
sp = sin(half_rpy(2));
cy = cos(half_rpy(3));
sy = sin(half_rpy(3));

q_nb = [ ...
    cr * cp * cy + sr * sp * sy; ...
    sr * cp * cy - cr * sp * sy; ...
    cr * sp * cy + sr * cp * sy; ...
    cr * cp * sy - sr * sp * cy];
q_nb = q_nb ./ norm(q_nb);
end

function euler_rpy_rad = local_quat_to_euler_rpy(q_nb)
q_nb = q_nb(:) ./ norm(q_nb);
q0 = q_nb(1);
q1 = q_nb(2);
q2 = q_nb(3);
q3 = q_nb(4);

roll_rad = atan2(2.0 .* (q0 .* q1 + q2 .* q3), ...
    1.0 - 2.0 .* (q1.^2 + q2.^2));
pitch_arg = 2.0 .* (q0 .* q2 - q3 .* q1);
pitch_arg = min(max(pitch_arg, -1.0), 1.0);
pitch_rad = asin(pitch_arg);
yaw_rad = atan2(2.0 .* (q0 .* q3 + q1 .* q2), ...
    1.0 - 2.0 .* (q2.^2 + q3.^2));
euler_rpy_rad = [roll_rad; pitch_rad; yaw_rad];
end

function text_value = local_bool_text(flag_value)
if flag_value
    text_value = "да";
else
    text_value = "нет";
end
end
