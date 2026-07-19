function angles = rotmToRPY(R)
% ROTMTORPY Convert rotation matrix to roll-pitch-yaw
%
% Input:
%   R      : [3x3] rotation matrix
%
% Output:
%   angles : [3x1] = [roll; pitch; yaw] in radians
%
% Convention:
%   R = Rz(yaw) * Ry(pitch) * Rx(roll)

    if ~isequal(size(R), [3, 3])
        error('Input R must be a 3x3 matrix.');
    end

    pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));

    if abs(cos(pitch)) < 1e-9
        roll = 0;
        yaw  = atan2(-R(1,2), R(2,2));
    else
        roll = atan2(R(3,2), R(3,3));
        yaw  = atan2(R(2,1), R(1,1));
    end

    angles = [roll; pitch; yaw];
end