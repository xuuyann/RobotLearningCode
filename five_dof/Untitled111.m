% Standard DH
% five_dof robot
% 在关节4和关节5之间增加一个虚拟关节，便于逆运动学计算
clear;
clc;
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = pi/2;
th(2) = 0; d(2) = 0; a(2) = 0.104;alp(2) = 0;
th(3) = 0; d(3) = 0; a(3) = 0.096; alp(3) = 0;
th(4) = 0; d(4) = 0; a(4) = 0; alp(4) = 0;
th(5) =0; d(5) = 0; a(5) = 0; alp(5) = pi/2;
th(6) = 0; d(6) = 0; a(6) = 0; alp(6) = 0;
th(7) = 0; d(7) = 0.163; a(7) = 0.028; alp(7) = 0;
% DH parameters  th     d    a    alpha  sigma
L1 = Link([th(1), d(1), a(1), alp(1), 0]);
L2 = Link([th(2), d(2), a(2), alp(2), 0]);
L3 = Link([th(3), d(3), a(3), alp(3), 0]);
L4 = Link([th(4), d(4), a(4), alp(4), 0]);
L5 = Link([th(5), d(5), a(5), alp(5), 0]); 
L6 = Link([th(6), d(6), a(6), alp(6), 0]);
L7 = Link([th(7), d(7), a(7), alp(7), 0]);
L2.offset=pi/2;
L4.offset = pi/2;
robot = SerialLink([L1, L2, L3, L4, L5, L6, L7]); 
view(3)
hold on
grid on
axis([-1, 1, -1, 1, -1, 1])
robot.name='MyRobot-5-dof';
robot.display() 
theta = [0 0 0 0 0 0 0]*pi/180;
% robot.teach();
robot.plot(theta); 
t = robot.fkine(theta)    %末端执行器位姿
ik_T = five_dof_ikine(t)
