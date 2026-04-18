function alt_est = altitude_filter_init(params)
%ALTITUDE_FILTER_INIT Initialize the minimal altitude estimator state.
% Description:
%   Returns the estimator state for the transparent altitude complementary
%   filter.
%
% Inputs:
%   params - parameter struct with estimator.altitude settings
%
% Outputs:
%   alt_est - struct with alt_m and vz_mps
%
% Units:
%   alt_m [m], vz_mps [m/s]
%
% Assumptions:
%   The initial altitude and vertical speed are zero until explicit
%   initialization through uav.est.estimator_init or filter correction.

local_validate_altitude_params(params);

alt_est = struct();
alt_est.alt_m = 0.0;
alt_est.vz_mps = 0.0;
end

function local_validate_altitude_params(params)
%LOCAL_VALIDATE_ALTITUDE_PARAMS Validate required altitude estimator fields.

if ~isstruct(params) || ~isfield(params, 'estimator') || ...
        ~isfield(params.estimator, 'altitude')
    error('uav:est:altitude_filter_init:MissingParams', ...
        'Expected params.estimator.altitude to be present.');
end

validateattributes(params.estimator.altitude.k_baro, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, ...
    'params.estimator.altitude.k_baro');
validateattributes(params.estimator.altitude.use_imu_prediction, ...
    {'logical', 'numeric'}, {'real', 'scalar', 'finite'}, mfilename, ...
    'params.estimator.altitude.use_imu_prediction');
validateattributes(params.estimator.altitude.vz_damping, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, ...
    'params.estimator.altitude.vz_damping');
end
