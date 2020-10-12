% Standard DH
% five_dof robot
% 在关节4和关节5之间增加一个虚拟关节，便于逆运动学计算
clear;
clc;
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = pi/2;
th(2) = 0; d(2) = 0; a(2) = 1.04;alp(2) = 0;
th(3) = 0; d(3) = 0; a(3) = 0.96; alp(3) = 0;
th(4) = 0; d(4) = 0; a(4) = 0; alp(4) = 0;
th(5) = pi/2; d(5) = 0; a(5) = 0; alp(5) = pi/2;
th(6) = 0; d(6) = 0; a(6) = 0; alp(6) = 0;
th(7) = 0; d(7) = 1.63; a(7) = 0.28; alp(7) = 0;
% DH parameters  th     d    a    alpha  sigma
L1 = Link([th(1), d(1), a(1), alp(1), 0]);
L2 = Link([th(2), d(2), a(2), alp(2), 0]);
L3 = Link([th(3), d(3), a(3), alp(3), 0]);
L4 = Link([th(4), d(4), a(4), alp(4), 0]);
L5 = Link([th(5), d(5), a(5), alp(5), 0]); 
L6 = Link([th(6), d(6), a(6), alp(6), 0]);
L7 = Link([th(7), d(7), a(7), alp(7), 0]);
L1.qlim = [-pi, pi];
L2.qlim = [0, pi];
L3.qlim = [-115, 115]*pi/180;
L4.qlim = [-150, 115]*pi/180;
L6.qlim = [-pi, pi];
robot = SerialLink([L1, L2, L3, L4, L5, L6, L7]); 
robot.name='MyRobot-5-dof';
robot.display() 

% 起点关节角[0 90 30 60 90 0 0]*pi/180
% 终点关节角[0 90 60 60 90 0 0]*pi/180
% theta = [3.141592653589793   2.356194490192346  -2.094395102393196  -1.047197551196598 pi/2 -2.356194490192346 0];
% theta_S = [0 90 10 60 90 0 0]*pi/180;
theta_S = [-40 45 -100 55 90 0 0]*pi/180;
% theta_S = [-2.792526803190927+2*pi   0.211833212557015   0.872664625997165  -1.608096614152479+2*pi  pi/2  -3.141592653589793+2*pi 0];
robot.teach()
robot.plot(theta_S)
T = robot.fkine(theta_S)
ik_t = five_dof_ikine(T)
ik_theta = ik_t(1, 1:5);
T_ = robot.fkine([ik_theta(1), ik_theta(2), ik_theta(3), ik_theta(4), pi/2, ik_theta(5), 0])
% theta_D = [0.000000000000002   0.698186446370751   1.752890728927838   0.690515478291204 pi/2   0.000000000000000 0];
% robot.teach();
% robot.plot(theta_D);
% T_S = robot.fkine(theta_S)
% T_D = robot.fkine(theta_D)
% ik = five_dof_ikine(T)
% [alpha1 beta1 gama1] = RPY_angle(T_S)
% [alpha2 beta2 gama2] = RPY_angle(T_D)
% T_ = [0.421519753594445,2.96118056439183e-16,-0.906819219761954,-1.83132287523727;-9.17412030278573e-17,-1,-3.69190218334618e-16,-0.579679961692538;-0.906819219761954,2.38813656011632e-16,-0.421519753594445,0.457360077215306;0,0,0,1]
% ik_theta = five_dof_ikine(T_);
% ik_theta = ik_theta(2, 1:5)
% T__ = robot.fkine([ik_theta(1), ik_theta(2), ik_theta(3), ik_theta(4), pi/2, ik_theta(5), 0])
% robot.plot([ik_theta(1), ik_theta(2), ik_theta(3), ik_theta(4), pi/2, ik_theta(5), 0])




