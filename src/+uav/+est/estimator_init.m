function est = estimator_init(params, sens0)
%ESTIMATOR_INIT Initialize the combined Stage-1.5 estimator layer state.
% Description:
%   Builds the aggregate estimator state for attitude and altitude without
%   changing the underlying plant or sensor models.
%
% Inputs:
%   params - parameter struct with estimator settings
%   sens0  - initial sensor sample used to seed altitude from barometer
%
% Outputs:
%   est - struct with attitude, altitude, and top-level convenience fields
%
% Units:
%   SI only, angular quantities in radians
%
% Assumptions:
%   Initial attitude starts from identity and altitude is seeded from the
%   first barometric sample when available.

attitude = uav.est.attitude_cf_init(params);
altitude = uav.est.altitude_filter_init(params);

if nargin >= 2 && isstruct(sens0) && isfield(sens0, 'baro') && ...
        isfield(sens0.baro, 'alt_m')
    validateattributes(sens0.baro.alt_m, {'numeric'}, ...
        {'real', 'scalar', 'finite'}, mfilename, 'sens0.baro.alt_m');
    altitude.alt_m = sens0.baro.alt_m;
end

est = local_pack_estimator(attitude, altitude);
end

function est = local_pack_estimator(attitude, altitude)
%LOCAL_PACK_ESTIMATOR Build the combined estimator state struct.

est = struct();
est.attitude = attitude;
est.altitude = altitude;
est.q_nb = attitude.q_nb;
est.euler_rpy_rad = attitude.euler_rpy_rad;
est.alt_m = altitude.alt_m;
est.vz_mps = altitude.vz_mps;
end
