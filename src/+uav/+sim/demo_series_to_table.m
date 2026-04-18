function demo_table = demo_series_to_table(series)
%DEMO_SERIES_TO_TABLE Convert postprocessed demo histories into a flat table.
% Description:
%   Builds a CSV-friendly numeric table from the struct returned by
%   `uav.sim.postprocess_demo_log`, including both radian and degree
%   attitude channels together with motor-command and motor-speed traces.
%
% Inputs:
%   series - struct returned by uav.sim.postprocess_demo_log
%
% Outputs:
%   demo_table - flat MATLAB table for CSV export
%
% Units:
%   SI only, with explicit additional degree columns for plotting/reporting
%
% Assumptions:
%   All histories inside `series` are sampled on the same time base.

demo_table = table();
demo_table.time_s = series.time_s(:);
demo_table.altitude_ref_m = series.altitude_ref_m(:);
demo_table.altitude_m = series.altitude_m(:);
demo_table.altitude_est_m = series.altitude_est_m(:);

demo_table.vertical_speed_ref_mps = series.vertical_speed_ref_mps(:);
demo_table.vertical_speed_mps = series.vertical_speed_mps(:);
demo_table.vertical_speed_est_mps = series.vertical_speed_est_mps(:);

demo_table.pitch_ref_rad = series.pitch_ref_rad(:);
demo_table.pitch_rad = series.pitch_rad(:);
demo_table.pitch_est_rad = series.pitch_est_rad(:);
demo_table.pitch_ref_deg = rad2deg(series.pitch_ref_rad(:));
demo_table.pitch_deg = rad2deg(series.pitch_rad(:));
demo_table.pitch_est_deg = rad2deg(series.pitch_est_rad(:));

demo_table.pitch_estimation_error_deg = series.pitch_estimation_error_deg(:);
demo_table.accel_correction_weight = series.accel_correction_weight(:);
demo_table.accel_consistency_metric = series.accel_consistency_metric(:);

demo_table.quat_norm_true = series.quat_norm_true(:);
demo_table.quat_norm_est = series.quat_norm_est(:);

for motor_idx = 1:4
    cmd_name = sprintf('motor_cmd_%d_radps', motor_idx);
    speed_name = sprintf('motor_speed_%d_radps', motor_idx);
    demo_table.(cmd_name) = series.motor_cmd_radps(:, motor_idx);
    demo_table.(speed_name) = series.motor_speed_radps(:, motor_idx);
end
end
