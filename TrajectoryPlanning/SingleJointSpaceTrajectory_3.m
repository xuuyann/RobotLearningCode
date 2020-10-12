%% 与抛物线拟合的线性函数（线性轨迹）
% 梯形加减速，可限制最大速度和最大加速度
% 用户给定起始速度、终止速度、加速度、减速度、最大速度及位移参数
% 该算法需要计算出加速段、匀速段以及减速段对应的时间Ta Tv Td
% t0 = 0, p0 = 5, p1 = 30, v0 = 50, vmax = 150, v1 = 20, aa = 1000
% ad = -1500
t0 = 2;
p0 = 5; p1 = 30;
v0 = 50; vmax = 150; v1 = 50;
aa = 1000; ad = -aa;
h = p1 - p0;
% 可达到的最大速度
vf = sqrt((2.0*aa*ad*h - aa*v1^2 + ad*v0^2) / (ad - aa));
% 确定匀速阶段速度
if (vf < vmax)
    vv = vf;
else
    vv = vmax;
end
% 计算加速阶段的时间和位移
Ta = (vv - v0) / aa;
La = v0*Ta + (1.0/2.0)*aa*Ta^2;
% 计算匀速阶段的时间和位移
Tv = (h - (vv^2 - v0^2)/(2.0*aa) - (v1^2 - vv^2)/(2.0*ad)) / vv;
Lv = vv*Tv;
% 计算减速阶段的时间和位移
Td = (v1 - vv) / ad;
% Td = Ta;
Ld = vv*Td + (1.0/2.0)*ad*Td^2;
k = 1;
ts = 0.001;
% 计算轨迹的离散点
for t = t0: ts: (t0+Ta+Tv+Td)
    time(k) = t;
    t = t - t0;
    if (t >= 0 && t < Ta)
        p(k) = p0 + v0*t + (1.0/2.0)*aa*t^2;
        pd(k) = v0 + aa*t;
        pdd(k) = aa;
    elseif (t >= Ta && t < Ta+Tv)
        p(k) = p0 + La + vv*(t - Ta);
        pd(k) = vv;
        pdd(k) = 0;
    elseif (t >= Ta+Tv && t <= Ta+Tv+Td)
        p(k) = p0 + La + Lv + vv*(t - Ta - Tv) + (1.0/2.0)*ad*power(t - Ta - Tv, 2);
        pd(k) = vv + ad*(t - Ta - Tv);
        pdd(k) = ad;
    end
    k = k + 1;
end
figure(1)
subplot(3, 1, 1)
plot(time, p, 'r', 'LineWidth', 1.5)
ylabel('position')
grid on
subplot(3, 1, 2)
plot(time, pd, 'b', 'LineWidth', 1.5)
ylabel('velocity')
grid on
subplot(3, 1, 3)
plot(time, pdd, 'g', 'LineWidth', 1.5)
ylabel('acceleration')
grid on









