% Standard DH
% five_dof robot
% 和five_dof_SDH的功能一样，不同点在于其速度规划方法为S型加减速曲线
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
theta_S = [-40 90 10 0 90 0 0]*pi/180;
theta_M = [-40 100 45 0 90 0 0]*pi/180;
theta_D = [-40 45 10 0 90 0 0]*pi/180;
% robot.teach();
% robot.plot(theta_D); 
% hold on
T_S = robot.fkine(theta_S)    %起点末端执行器位姿
T_M = robot.fkine(theta_M)
T_D = robot.fkine(theta_D)
% ik_T = five_dof_ikine(T_D)
S = T_S(1: 3, 4); % 起点对应的位置坐标
M = T_M(1: 3, 4);
D = T_D(1: 3, 4); % 终点对应的位置坐标
plot3(S(1), S(2), S(3), 'o')
hold on
plot3(M(1), M(2), M(3), '*')
hold on
plot3(D(1), D(2), D(3), 'o')
hold on
% D = [S(1), S(2), S(3)-1];
% 获取起点和终点的单位四元数
Qs = Quaternion(T_S)
Qd = Quaternion(T_D)
v0 = 0; v1 = 0; vmax = 0.5; amax = 0.1; jmax = 0.1;  % 直线插补速度参数
w0 = 0; w1 = 0; wmax = 50*pi/180; wamax = 80*pi/180; wjmax = 80*pi/180;  % 空间圆弧插补速度参数
t = 0.05; % 插补周期
% function [x y z qk N] = SpaceLine_Q(S, D, Qs, Qd, v0, v1, vmax, amax, jmax, t)
% function [x y z qk N] = SpaceCircle_Q(S, M, D, Qs_, Qd_, v0, v1, vmax, amax, jmax, t)
% [x y z qk N] = SpaceLine_Q(S, D, Qs, Qd, v0, v1, vmax, amax, jmax, t); % 单位四元数姿态插补
[x y z qk N] = SpaceCircle_Q(S', M', D', Qs, Qd, w0, w1, wmax, wamax, wjmax, t);
plot3(x, y, z, '.')
hold on
th1(1) = theta_S(1); th2(1) = theta_S(2); th3(1) = theta_S(3);
th4(1) = theta_S(4); th5(1) = theta_S(5); th6(1) = theta_S(6); th7(1) = theta_S(7);
% t = [0 0 0 0 0];
% theta_S_ = five_dof_ikine(T_S);
% th1(1) = theta_S_(2, 1); th2(1) = theta_S_(2, 2); th3(1) = theta_S_(2, 3);
% th4(1) = theta_S_(2, 4); th5(1) = pi/2; th6(1) = theta_S_(2, 5); th7(1) = 0;
T = {1, N};
R = {1, N};
R2 = {1, N};
for i = 1: N
%     R{i} = RPY_rot(alp(i), beta(i), gama(i)); % RPY角姿态插补，有奇异问题
    R{i} = q2tr(qk(i));
    T{i} = [R{i}(1, 1:3), x(i);
            R{i}(2, 1:3), y(i);
            R{i}(3, 1:3), z(i);
            0 0 0 1];
     theta = five_dof_ikine(T{i});
     theta_ = theta*180/pi;
     th = theta(2, 1:5); % 简单取1行逆解
     th1(i+1) = th(1);
     th2(i+1) = th(2);
     th3(i+1) = th(3);
     th4(i+1) = th(4);
     th5(i+1) = pi/2;
     th6(i+1) = th(5);
     th7(i+1) = 0;
     % 剔除跳变值
     if (abs(th1(i+1)) == abs(th1(i)) && th1(i+1) ~= th1(i))
         th1(i+1) = th1(i);
     end
     if (abs(th2(i+1)) == abs(th2(i)) && th2(i+1) ~= th2(i))
         th2(i+1) = th2(i);
     end
     if (abs(th3(i+1)) == abs(th3(i)) && th3(i+1) ~= th3(i))
         th3(i+1) = th3(i);
     end
     if (abs(th4(i+1)) == abs(th4(i)) && th4(i+1) ~= th4(i))
         th4(i+1) = th4(i);
     end
     if (abs(th6(i+1)) == abs(th6(i)) && th6(i+1) ~= th6(i))
         th6(i+1) = th6(i);
     end
end
for i = 1: N
%     T_ = robot.fkine([th1(i), th2(i), th3(i), th4(i), th5(i), th6(i), th7(i)]);
    traj = T_(1: 3, 4);
    a(i) = traj(1);
    b(i) = traj(2);
    c(i) = traj(3);
    plot3(traj(1), traj(2), traj(3), 'r*');
    hold on
    robot.plot([th1(i), th2(i), th3(i), th4(i), th5(i), th6(i), th7(i)])
end
figure(2)
subplot(5, 1, 1)
plot([1:N+1], th1*180/pi);
grid on
subplot(5, 1, 2)
% figure(3)
plot([1:N+1], th2*180/pi);
grid on

% figure(4)
subplot(5, 1, 3)
plot([1:N+1], th3*180/pi);
grid on
% figure(5)
subplot(5, 1, 4)
plot([1:N+1], th4*180/pi);
grid on
% figure(6)
subplot(5, 1, 5)
plot([1:N+1], th6*180/pi);
grid on
    


    


% plot3(x, y, z, '*')
% N=300;                                              %随机次数
% theta1 = -185/180*pi+(185/180*pi+185/180*pi)*rand(N,1); %关节1限制
% theta2 = -20/180*pi+(130/180*pi+20/180*pi)*rand(N,1);   %关节2限制
% theta3 = -60/180*pi+(184/180*pi+60/180*pi)*rand(N,1);  %关节3限制
% theta4 = -350/180*pi+(350/180*pi+350/180*pi)*rand(N,1); %关节4限制
% theta5 = -118/180*pi+(118/180*pi+118/180*pi)*rand(N,1); %关节5限制
% visual_th = (pi/2)*rand(N, 1);
% theta7 = 0*rand(N, 1);
% modmyt06 = {1,N}; %??
% for n = 1:1:N
%     modmyt06{n} = robot.fkine([theta1(n), theta2(n), theta3(n), theta4(n), visual_th(n), theta5(n), theta7(n)]);
%     plot3(modmyt06{n}(1,4),modmyt06{n}(2,4),modmyt06{n}(3,4),'b.','MarkerSize',0.5);
% %     robot.plot([theta1(n), theta2(n), theta3(n), theta4(n), visual_th(n), theta5(n), theta7(n)])
% end
% maxx = 2; maxy = 2; maxz = 2;
% for i = 1:1:N
%     if( maxx < modmyt06{i}(1,4))
%         maxx = modmyt06{i}(1,4);
%     end
%     if( maxy < modmyt06{i}(2,4))
%         maxy = modmyt06{i}(2,4);
%     end
%     if( maxz < modmyt06{i}(3,4))
%         maxz = modmyt06{i}(3,4);
%     end
% end


