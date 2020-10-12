% 单关节空间轨迹
%% 关于拐点对称的抛物线轨迹
% 已知起始和结束时刻的位置和速度
t0 = 0; t = 8; T = t - t0;
tf = T / 2; % 拐点
v0 = 0; v1 = 0;
theta_s = 0; theta = 10; theta_f = (theta_s + theta) / 2;
h = theta - theta_s;
% theta_a(t) = a10 + a11*(t - t0) + a12*(t - t0)^2 加速段
a10 = theta_s; a11 = v0; a12 = (2/T^2)*(h - v0*T);
% matlab中这样计算是很方便的，但是c里面无法直接合并向量，得借助循环遍历
Ta = linspace(t0, tf, 400);
theta_a = a10 + a11*(Ta - t0) + a12*power(Ta - t0, 2);
theta_ad = a11 + 2*a12*(Ta - t0);
theta_add = 2*a12;
% theta_b(t) = a20 + a21*(t - tf) + a22*(t - tf)^2 减速段
Tb = linspace(tf, t, 400);
Tn = [Ta, Tb(2: 400)];
a20 = theta_f; a21 = 2*(h/T) - v1; a22 = (2/T^2)*(v1*T - h); 
theta_b = a20 + a21*(Tb - tf) + a22*power(Tb - tf, 2);
theta_bd = a21 + 2*a22*(Tb - tf);
theta_bdd = 2*a22;
theta = [theta_a, theta_b(2: 400)];
theta_d = [theta_ad, theta_bd(2: 400)];
figure(1)
subplot(3, 1, 1)
plot(Tn, theta, 'r')
ylabel('position')
hold on
plot(tf, theta_f, 'or')
grid on
subplot(3, 1, 2)
plot(Tn, theta_d, 'b')
ylabel('velocity')
hold on
plot(tf, theta_d(400), 'ob')
grid on
for i = 1: 799
    theta_dd(i) = theta_add;
    if (i > 400)
        theta_dd(i) = theta_bdd;
    end
end
subplot(3, 1, 3)
plot(Tn, theta_dd, 'g')
ylabel('acceleration')
hold on
plot(tf, theta_dd(400), 'og')
grid on

%% 更一般的情况，拐点两边不对称，但是加速度对称恒定
% 已知起始和结束时刻的位置和速度，同时拐点的位置和速度具有连续性
t0 = 0; t1 = 8; T = t - t0; tf = 4;
ta = tf - t0; tb = t - tf;
v0 = 0.1; v1 = -1; 
q0 = 0; q1 = 10; h = q1 - q0;
% theta_a(t) = a10 + a11*(t - t0) + a12*(t - t0)^2 加速段
a10 = q0; a11 = v0; a12 = (2*h - v0*(T + ta) - v1*tb) / (2*T*tb);
Ta = linspace(t0, tf, (ta/T)*800);
q_a = a10 + a11*(Ta - t0) + a12*power(Ta - t0, 2);
q_ad = a11 + 2*a12*(Ta- t0);
q_add = 2*a12;
% theta_b(t) = a20 + a21*(t - tf) + a22*(t - tf)^2 减速段
a20 = (2*q1*ta + tb*(2*q0 + ta*(v0 - v1))) / (2*T);
a21 = (2*h - v0*ta - v1*tb) / T;
a22 = -(2*h - v0*ta - v1*(T + tb)) / (2*T*tb);
Tb = linspace(tf, t1, (tb/T)*800);
q_b = a20 + a21*(Tb - tf) + a22*power(Tb - tf, 2);
q_bd = a21 + 2*a22*(Tb - tf);
q_bdd = 2*a22;
Tn = [Ta, Tb(2: (tb/T)*800)];
q = [q_a, q_b(2: (tb/T)*800)];
q_d = [q_ad, q_bd(2: (tb/T)*800)];
for i = 1:799
   q_dd(i) = q_add;
   if (i > (ta/T)*800)
       q_dd(i) = q_bdd;
   end
end
figure(2)
subplot(3, 1, 1)
plot(Tn, q, 'r');
hold on
plot(tf, q_a((ta/T)*800), 'or')
grid on
ylabel('position')
subplot(3, 1, 2)
plot(Tn, q_d, 'b')
hold on
plot(tf, q_ad((ta/T)*800), 'ob')
grid on
subplot(3, 1, 3)
plot(Tn, q_dd, 'g')
grid on









