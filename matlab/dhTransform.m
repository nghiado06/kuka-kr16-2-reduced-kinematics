function A = dhTransform(alpha, a, d, theta)
% DHTRANSFORM Standard DH transformation matrix
%
% Input:
%   alpha, a, d, theta : standard DH parameters
%
% Output:
%   A : [4x4] homogeneous transformation matrix

    ca = cos(alpha);
    sa = sin(alpha);
    ct = cos(theta);
    st = sin(theta);

    A = [ ct,   -st*ca,  st*sa,  a*ct;
          st,    ct*ca, -ct*sa,  a*st;
          0,        sa,     ca,     d;
          0,         0,      0,     1 ];
end