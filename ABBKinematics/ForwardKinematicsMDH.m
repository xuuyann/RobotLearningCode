function [p_xyz, T] = ForwardKinematicsMDH(theta, d, a, alp)
% 改进D-H建模
%Theta, d, a, Alpha are lists (1 row matrix) with n inputs
%       Where n is the number of links in the arm.
%
% theta:     Joint Angles (theta)
% d:     Link offset
% a:     Link Length
% alp:   Link Twist (alpha)
% n:     Robot degrees of freedom
n = length(theta);
% deg2rad可能需要可能不需要，视情况而定
th = theta;
% th = deg2rad(theta);
% alp = deg2rad(alp);

T1_n = eye(4);
T{n} = zeros(4);

for i = 1 : n
    Ti_n= [cos(th(i))             -sin(th(i))               0            a(i);
           sin(th(i))*cos(alp(i))  cos(th(i))*cos(alp(i))  -sin(alp(i))  -sin(alp(i))*d(i);
           sin(th(i))*sin(alp(i))  cos(th(i))*sin(alp(i))  cos(alp(i))   cos(alp(i))*d(i);
           0                        0                         0            1];
    T1_n = T1_n * Ti_n;
    T{i} = T1_n;
end

p_xyz = T{n}(1: 3, 4)';

end