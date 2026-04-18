function att_est = attitude_cf_init(params)
%ATTITUDE_CF_INIT Initialize the minimal attitude complementary filter.
% Description:
%   Returns the estimator state for the transparent quaternion-based
%   attitude complementary filter.
%
% Inputs:
%   params - parameter struct with estimator.attitude settings
%
% Outputs:
%   att_est - struct with q_nb and euler_rpy_rad
%
% Units:
%   q_nb [-], euler_rpy_rad [rad]
%
% Assumptions:
%   The initial estimate is identity attitude until measurements are
%   processed by uav.est.attitude_cf_step.

local_validate_attitude_params(params);

att_est = struct();
att_est.q_nb = [1.0; 0.0; 0.0; 0.0];
att_est.euler_rpy_rad = zeros(3, 1);
end

function local_validate_attitude_params(params)
%LOCAL_VALIDATE_ATTITUDE_PARAMS Validate required attitude estimator fields.

if ~isstruct(params) || ~isfield(params, 'estimator') || ...
        ~isfield(params.estimator, 'attitude')
    error('uav:est:attitude_cf_init:MissingParams', ...
        'Expected params.estimator.attitude to be present.');
end

validateattributes(params.estimator.attitude.k_acc, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, ...
    'params.estimator.attitude.k_acc');
validateattributes(params.estimator.attitude.k_mag, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'nonnegative'}, mfilename, ...
    'params.estimator.attitude.k_mag');
end
