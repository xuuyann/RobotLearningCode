% Modified DH
% ABB robot
clear;
clc;
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = 0;
th(2) = 0; d(2) = 0; a(2) = 0.320; alp(2) = pi/2;   
th(3) = 0; d(3) = 0; a(3) = 0.975; alp(3) = 0;
th(4) = 0; d(4) = 0.887; a(4) = 0.2; alp(4) = pi/2;
th(5) = 0; d(5) = 0; a(5) = 0; alp(5) = -pi/2;
th(6) = 0; d(6) = 0; a(6) = 0; alp(6) = pi/2;
% DH parameters  th     d    a    alpha  sigma
L1 = Link([th(1), d(1), a(1), alp(1), 0], 'modified');
L2 = Link([th(2), d(2), a(2), alp(2), 0], 'modified');
L3 = Link([th(3), d(3), a(3), alp(3), 0], 'modified');
L4 = Link([th(4), d(4), a(4), alp(4), 0], 'modified');
L5 = Link([th(5), d(5), a(5), alp(5), 0], 'modified');
L6 = Link([th(6), d(6), a(6), alp(6), 0], 'modified');
robot = SerialLink([L1, L2, L3, L4, L5, L6]); 
robot.name='ABBRobot-6-dof';
robot.display() 
% Forward Pose Kinematics
theta = [0, 120, -15, 0, 0, 0]*pi/180;
robot.teach();
robot.plot(theta); 
t0 = robot.fkine(theta)    %Ä©¶ËÖ´ÐÐÆ÷Î»×Ë
