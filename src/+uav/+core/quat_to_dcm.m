function c_nb = quat_to_dcm(q)
%QUAT_TO_DCM Convert a scalar-first quaternion into a body-to-NED DCM.
% Description:
%   Converts q_nb into a 3x3 direction cosine matrix that maps body-frame
%   vectors into the NED frame.
%
% Inputs:
%   q - 4x1 or 1x4 scalar-first quaternion [qw qx qy qz]
%
% Outputs:
%   c_nb - 3x3 direction cosine matrix
%
% Units:
%   Quaternion and DCM are dimensionless
%
% Assumptions:
%   The quaternion is scalar-first and represents body-to-NED attitude.

q = uav.core.quat_normalize(q);

qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

c_nb = [ ...
    1.0 - 2.0 * (qy^2 + qz^2), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy); ...
    2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx^2 + qz^2), 2.0 * (qy * qz - qw * qx); ...
    2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx^2 + qy^2)];
end
