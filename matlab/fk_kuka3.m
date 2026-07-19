function [p_tip, angles, T03, T_tip] = fk_kuka3(q)
% FK_KUKA3 Forward kinematics for the first 3 joints + tip offset
%
% Input:
%   q     : [q1 q2 q3] joint angles in radians or degrees
%
% Output:
%   p_tip : [3x1] tip position in base frame
%   angles: [3x1] roll-pitch-yaw angles in radians
%   T03   : [4x4] transform from base to frame {3}
%   T_tip : [4x4] transform from base to tip
%
% Note:
%   - Standard DH is used:
%       A_i = Rotz(theta_i) * Tz(d_i) * Tx(a_i) * Rotx(alpha_i)
%   - Local tip coordinates in frame {3}: [0; 0; 476.5]

    if numel(q) ~= 3
        error('Input q must have exactly 3 joint angles.');
    end

    q = q(:);

    % If user accidentally inputs degrees, uncomment this line:
    % q = deg2rad(q);

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

    % DH parameters
    alpha = deg2rad([-90, 0, 90]);
    a     = [260, 680, 22.5];
    d     = [675, 0, 0];
    theta = [q1, q2, q3];

    % Component transforms
    A1 = dhTransform(alpha(1), a(1), d(1), theta(1));
    disp("T01: ");
    disp(A1);
    A2 = dhTransform(alpha(2), a(2), d(2), theta(2));
    disp("T12: ");
    disp(A2);
    A3 = dhTransform(alpha(3), a(3), d(3), theta(3));
    disp("T23: ");
    disp(A3);

    % Base to frame {3}
    T03 = A1 * A2 * A3;

    % Tip offset in local frame {3}
    T_tip_local = eye(4);
    T_tip_local(1:3, 4) = [0; 0; 476.5];

    % Base to tip
    T_tip = T03 * T_tip_local;

    % Position
    p_tip = T_tip(1:3, 4);

    % Orientation as roll-pitch-yaw
    R = T_tip(1:3, 1:3);
    angles = rotmToRPY(R);
end