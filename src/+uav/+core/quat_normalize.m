function q_unit = quat_normalize(q)
%QUAT_NORMALIZE Safely normalize a scalar-first quaternion.
% Description:
%   Normalizes a quaternion and falls back to the identity quaternion when
%   the input norm is too small for a safe division.
%
% Inputs:
%   q - 4x1 or 1x4 scalar-first quaternion [qw qx qy qz]
%
% Outputs:
%   q_unit - 4x1 unit quaternion
%
% Units:
%   Quaternion components are dimensionless
%
% Assumptions:
%   The quaternion uses scalar-first ordering.

q = q(:);
q_norm = norm(q);

if q_norm < sqrt(eps)
    q_unit = [1.0; 0.0; 0.0; 0.0];
    return;
end

q_unit = q ./ q_norm;

if q_unit(1) < 0.0
    q_unit = -q_unit;
end
end
