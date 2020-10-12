% 空间单一圆弧位置插补与单位四元数姿态插补 + 梯形加减速归一化处理/S型加减速曲线归一化处理
% 参数： 起点S位置, 中间点M位置, 终点D位置
%       起点S和终点D的单位四元数Qs_,Qd_
%       起始速度v0，终止速度v1，最大速度vmax，最大加速度amax，最大加加速度jmax
%       插值周期t
% 返回值：插值点位置、单元四元数、插值点数
% 方便起见，角速度和角加速度均为角度制
function [x y z qk N] = SpaceCircle_Q(S, M, D, Qs_, Qd_, v0, v1, vmax, amax, jmax, t)
x1 = S(1); x2 = M(1); x3 = D(1);
y1 = S(2); y2 = M(2); y3 = D(2);
z1 = S(3); z2 = M(3); z3 = D(3);

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
x1_ = S_(1), y1_ = S_(2), z1_ = S_(3)
x2_ = M_(1), y2_ = M_(2), z2_ = M_(3)
x3_ = D_(1), y3_ = D_(2), z3_ = D_(3)
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
    theta = angle_SOD; % 圆心角
end
% 顺时针
if (angle_SOM >= angle_SOD)
    flag = -1;
    theta = 2*pi - angle_SOD; % 圆心角
end
% 插补点数N
% P = 1; %插补参数，增加插值点数，避免过小
% ws = ws*pi / 180; % 角度换成弧度
% a = a*pi / 180;
% N = ceil(P*theta / ws);
% % 求归一化参数：梯形加减速曲线
% lambda = Normalization(theta, ws, a, N);
% 求归一化参数：S型加减速曲线
% function lambda = Normalization_S(pos, v0, v1, vmax, amax, jmax, N)
[lambda, N] = Normalization_S(theta, v0, v1, vmax, amax, jmax, t);

% 插补原理: 在新平面上进行插补（简化）
% 在新坐标系下z1_,z2_,z3_均为0，即外接圆在新坐标系的XOY平面内
% 此时转化为平面圆弧插补问题
delta_ang = theta
% 计算两个四元数之间的夹角
dot_q = Qs_.s*Qd_.s + Qs_.v(1)*Qd_.v(1) + Qs_.v(2)*Qd_.v(2) + Qs_.v(3)*Qd_.v(3);
if (dot_q < 0)
    dot_q = -dot_q;
end

for i = 1: N
    % 位置插补
    x_(i) = flag * r * cos(lambda(i)*delta_ang);
    y_(i) = flag * r * sin(lambda(i)*delta_ang);
    P = T*[x_(i); y_(i); 0; 1];
    x(i) = P(1);
    y(i) = P(2);
    z(i) = P(3);
    % 单位四元数球面线性姿态插补
    % 插值点四元数
    if (dot_q > 0.9995)
        k0 = 1-lambda(i);
        k1 = lambda(i);
    else
        sin_t = sqrt(1 - power(dot_q, 2));
        omega = atan2(sin_t, dot_q);
        k0 = sin((1-lambda(i))*omega) / sin(omega);
        k1 = sin(lambda(i)*omega) / sin(omega);
    end
    qk(i) = Qs_ * k0 + Qd_ * k1;
end

end