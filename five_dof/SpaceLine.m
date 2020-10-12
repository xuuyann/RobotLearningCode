% 空间直线位置插补与RPY角姿态插补 + 梯形加减速归一化处理
% 参数：起点S位置， 终点D位置, 末端线速度vs， 加减速度a
%      起点S的RPY角、终点D的RPY角
% 返回值：插值点（不包括起点S和终点D）
function [x y z alp beta gama N] = SpaceLine(S, D, S_, D_, vs, a)
x1 = S(1); y1 = S(2); z1 = S(3);
x2 = D(1); y2 = D(2); z2 = D(3);
alp1 = S_(1); beta1 = S_(2); gama1 = S_(3);
alp2 = D_(1); beta2 = D_(2); gama2 = D_(3);
P = 1; % 插值参数，增加插值点数，避免过小
% 总位移S
s = sqrt(power(x2 - x1, 2) + power(y2 - y1, 2) + power(z2 - z1, 2))
% 插值点数N
N = ceil(P*s / vs)
% 求归一化参数
% function lambda = Normalization(pos, vel, accl, N)
lambda = Normalization(s, vs, a, N);
delta_x = x2 - x1;
delta_y = y2 - y1;
delta_z = z2 - z1;

delta_alp = alp2 - alp1;
delta_beta = beta2 - beta1;
delta_gama = gama2 - gama1;
for i = 1: N+1
    % 位置插补
    x(i) = x1 + delta_x*lambda(i);
    y(i) = y1 + delta_y*lambda(i);
    z(i) = z1 + delta_z*lambda(i);
    % RPY角的姿态插补
    alp(i) = alp1 + delta_alp*lambda(i);
    beta(i) = beta1 + delta_beta*lambda(i);
    gama(i) = gama1 + delta_gama*lambda(i);
end
end