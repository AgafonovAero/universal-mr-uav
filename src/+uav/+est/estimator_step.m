function [est, diag] = estimator_step(est_prev, sens, dt_s, params)
%ESTIMATOR_STEP Advance the combined Stage-1.5 estimator layer.
% Description:
%   Calls the attitude complementary filter and altitude filter, then
%   returns one aggregate estimator struct with top-level convenience
%   fields for downstream runners and demos.
%
% Inputs:
%   est_prev - previous estimator state from uav.est.estimator_init/step
%   sens     - current sensor sample
%   dt_s     - estimator time step [s]
%   params   - parameter struct
%
% Outputs:
%   est  - updated estimator state with attitude and altitude substructs
%   diag - diagnostic struct with attitude and altitude subdiagnostics
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   est_prev carries both estimator substates and uses the canonical Stage
%   1.5 sensor sample format.

est_prev = local_validate_estimator_state(est_prev);

[attitude, att_diag] = uav.est.attitude_cf_step( ...
    est_prev.attitude, sens, dt_s, params);
[altitude, alt_diag] = uav.est.altitude_filter_step( ...
    est_prev.altitude, sens, attitude, dt_s, params);

est = struct();
est.attitude = attitude;
est.altitude = altitude;
est.q_nb = attitude.q_nb;
est.euler_rpy_rad = attitude.euler_rpy_rad;
est.alt_m = altitude.alt_m;
est.vz_mps = altitude.vz_mps;

diag = struct();
diag.attitude = att_diag;
diag.altitude = alt_diag;
end

function est_prev = local_validate_estimator_state(est_prev)
%LOCAL_VALIDATE_ESTIMATOR_STATE Validate the combined estimator state.

if ~isstruct(est_prev) || ~isscalar(est_prev)
    error('uav:est:estimator_step:StateType', ...
        'Expected est_prev to be a scalar struct.');
end

required_fields = {'attitude', 'altitude'};
for k = 1:numel(required_fields)
    if ~isfield(est_prev, required_fields{k})
        error('uav:est:estimator_step:MissingField', ...
            'Expected est_prev.%s to be present.', required_fields{k});
    end
end
end
