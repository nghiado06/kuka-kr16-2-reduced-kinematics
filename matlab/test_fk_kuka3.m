clc;
clear;

q = deg2rad([0; 0; 0]);

[p_tip, angles, T03, T_tip] = fk_kuka3(q);

disp('T03 = ');
disp(T03);

disp('T_tip = ');
disp(T_tip);

disp('p_tip = ');
disp(p_tip*1000);

disp('angles (rad) = ');
disp(angles);

disp('angles (deg) = ');
disp(rad2deg(angles));