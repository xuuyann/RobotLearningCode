% 空间圆弧插补 + 梯形加减速归一化处理
% 参数： 起点S位置和RPY角, 中间点M位置, 终点D位置和RPY角，末端角速度，角加减速度
% 方便起见，角速度和角加速度均为角度制
function [x y z alp beta gama N] = SpaceCircle(S, M, D, S_, D_, ws, a)
x1 = S(1); x2 = M(1); x3 = D(1);
y1 = S(2); y2 = M(2); y3 = D(2);
z1 = S(3); z2 = M(3); z3 = D(3);
alp1 = S_(1); beta1 = S_(2); gama1 = S_(3);
alp2 = D_(1); beta2 = D_(2); gama2 = D_(3);

A1 = (y1 - y3)*(z2 - z3) - (y2 - y3)*(z1 - z3);
B1 = (x2 - x3)*(z1 - z3) - (x1 - x3)*(z2 - z3);
C1 = (x1 - x3)*(y2 - y3) - (x2 - x3)*(y1 - y3);
D1 = -(A1*x3 + B1*y3 + C1*z3);

A2 = x2 - x1;
B2 = y2 - y1;
C2 = z2 - z1;
D2 = -((x2^2 - x1^2) + (y2^2 - y1^2) + (z2^2 - z1^2)) / 2;

A3 = x3 - x2;
B3 = y3 - y2;
C3 = z3 - z2;
D3 = -((x3^2 - x2^2) + (y3^2 - y2^2) + (z3^2 - z2^2)) / 2;
A = [A1, B1, C1; A2, B2, C2; A3, B3, C3]
b = [-D1, -D2, -D3]'
% 圆心
C = Gauss_lie(3, A, b)
x0 = C(1); y0 = C(2); z0 = C(3);
plot3(x0, y0, z0, 'bo')
hold on
% 外接圆半径
r = sqrt(power(x1 - x0, 2) + power(y1 - y0, 2) + power(z1 - z0, 2));
% 新坐标系Z0的方向余弦
L = sqrt(A1^2 + B1^2 + C1^2);
ax = A1 / L; ay = B1 / L; az = C1 / L;
% 新坐标系X0的方向余弦
nx = (x1 - x0) / r;
ny = (y1 - y0) / r;
nz = (z1 - z0) / r;
% 新坐标系Y0的方向余弦
o = cross([ax, ay, az], [nx, ny, nz]);
ox = o(1);
oy = o(2);
oz = o(3);
% 相对于基座标系{O-XYZ}， 新坐标系{C-X0Y0Z0}的坐标变换矩阵
T = [nx ox ax x0;
     ny oy ay y0;
     nz oz az z0;
      0  0  0  1]
T_ni = T^-1
% 求在新坐标系{C-X0Y0Z0}下S、M和D的坐标
S_ = (T^-1)*[S'; 1]
M_ = (T^-1)*[M'; 1]
D_ = (T^-1)*[D'; 1]
x1_ = S_(1); y1_ = S_(2); z1_ = S_(3);
x2_ = M_(1); y2_ = M_(2); z2_ = M_(3);
x3_ = D_(1); y3_ = D_(2); z3_ = D_(3);
% 判断圆弧是顺时针还是逆时针，并求解圆心角
if (atan2(y2_, x2_) < 0)
    angle_SOM = atan2(y2_, x2_) + 2*pi;
else
    angle_SOM = atan2(y2_, x2_);
end
if (atan2(y3_, x3_) < 0)
    angle_SOD = atan2(y3_, x3_) + 2*pi;
else
    angle_SOD = atan2(y3_, x3_);
end
% 逆时针
if (angle_SOM < angle_SOD)
    flag = 1;
    theta = angle_SOD % 圆心角
end
% 顺时针
if (angle_SOM >= angle_SOD)
    flag = -1;
    theta = 2*pi - angle_SOD % 圆心角
end
% 插补点数N
P = 1; %插补参数，增加插值点数，避免过小
ws = ws*pi / 180; % 角度换成弧度
a = a*pi / 180;
N = ceil(P*theta / ws);
% 求归一化参数
lambda = Normalization(theta, ws, a, N);
figure(2)
plot(0: N, lambda)
hold on

% 插补原理: 在新平面上进行插补（简化）
% 在新坐标系下z1_,z2_,z3_均为0，即外接圆在新坐标系的XOY平面内
% 此时转化为平面圆弧插补问题
delta_ang = theta;
% 这里的姿态插补为RPY角
delta_alp = alp2 - alp1
delta_beta = beta2 - beta1;
delta_gama = gama2 - gama1;
for i = 1: N+1
    % 位置插补
    x_(i) = flag * r * cos(lambda(i)*delta_ang);
    y_(i) = flag * r * sin(lambda(i)*delta_ang);
    P = T*[x_(i); y_(i); 0; 1];
    x(i) = P(1);
    y(i) = P(2);
    z(i) = P(3);
    % 姿态插补为RPY角
    alp(i) = alp1 + delta_alp*lambda(i);
    beta(i) = beta1 + delta_beta*lambda(i);
    gama(i) = gama1 + delta_gama*lambda(i);
end
% % figure(1)
% % plot(x_, y_)
% 插补原理： 在原圆弧上进行插补
% 圆弧上任一点处沿前进方向的切向量
% x(1) = x1; y(1) = y1; z(1) = z1;
% for i = 1: N+1
%     m(i) = flag*(ay*(z(i) - z0) - az*(y(i) - y0));
%     n(i) = flag*(az*(x(i) - x0) - ax*(z(i) - z0));
%     l(i) = flag*(ax*(y(i) - y0) - ay*(x(i) - x0));
%     delta_s = delta_ang * r;
%     E = delta_s / (r*sqrt(ax^2 + ay^2 + az^2));
%     G = r / sqrt(r^2 + delta_s^2);
%     x(i+1) = x0 + G*(x(i) + E*m(i) - x0);
%     y(i+1) = y0 + G*(y(i) + E*n(i) - y0);
%     z(i+1) = z0 + G*(z(i) + E*l(i) - z0);
% end 

end




