% 三次样条：指定初始、终止速度以及加速度，也就是v0，vn，a0，an已知
% 这个方法需要增加两个额外的点q1_和qn-1_
% q1_在原有q1和q2之间，qn-1_在原有的qn-1和qn之间
% 这两个额外点对应的时间t1_和tn-1_需要计算，可以任意选择，本程序选择取平均值
% Input：
%   q：给定点的位置
%   t：给定点位置对应的时间
%   v0：初始速度
%   vn：终止速度
%   a0：初始加速度
%   an：终止加速度
%   tt：插补周期
% Output：
%   yy dyy ddyy：样条曲线函数值、速度、加速度值
function [yy dyy ddyy q1 qn_1] = cubicSpline_4(q, t, v0, vn, a0, an, tt)
if length(q) ~= length(t)
    error('输入的数据应成对');
end
n = length(q); % 原来的点数量
% 增加两个额外点q1_和qn-1_，计算两点对应的时间点
t1_ = (t(1)+t(2)) / 2;
tn_1_ = (t(n-1)+t(n)) / 2;
% 更新时间向量
t = [t(1), t1_, t(2: n-1), tn_1_, t(n)];
% 更新点的数量
n = n + 2;
% 矩阵A是个(n-2)阶对角占优矩阵
A = zeros(n-2);
c = zeros(n-2, 1);
for i = 1: n-2
    Tk_1 = t(i+2) - t(i+1);
    Tk = t(i+1) - t(i);
    if i == 1
        A(i, i) = 2*Tk_1 + Tk*(3+Tk/Tk_1);
        A(i, i+1) = Tk_1;
        c(i, 1) = 6*((q(2)-q(1))/Tk_1-v0*(1+Tk/Tk_1)-a0*(0.5+Tk/(3*Tk_1))*Tk);
    elseif i == 2
        T0 = t(2)-t(1);
        A(i, i-1) = Tk - (T0^2)/Tk;
        A(i, i) = 2*(Tk + Tk_1);
        A(i, i+1) = Tk_1;
        c(i, 1) = 6*((q(3)-q(2))/Tk_1-(q(2)-q(1))/Tk+v0*(T0/Tk)+a0*(T0^2)/(3*Tk));
    elseif i == n-2-1
        Tn_1 = t(n) - t(n-1);
        A(i, i-1) = Tk;
        A(i, i) = 2*(Tk + Tk_1);
        A(i, i+1) = Tk_1 - (Tn_1^2)/Tk_1;
        c(i, 1) = 6*((q(n-2)-q(n-3))/Tk_1-(q(n-3)-q(n-4))/Tk-vn*(Tn_1/Tk_1)+an*(Tn_1^2)/(3*Tk_1));
    elseif i == n-2
        A(i, i) = 2*Tk + Tk_1*(3+Tk_1/Tk);
        A(i, i-1) = Tk;
        c(i, 1) = 6*((q(n-3)-q(n-2))/Tk+vn*(1+Tk_1/Tk)-an*(0.5+Tk_1/(3*Tk))*Tk_1);
    else
        A(i, i-1) = Tk;
        A(i, i) = 2*(Tk + Tk_1);
        A(i, i+1) = Tk_1;
        c(i, 1) = 6*((q(i+1)-q(i))/Tk_1-(q(i)-q(i-1))/Tk);
    end
end
% 经过上述步骤得到对角占优矩阵A和c
wk = A \ c; % 这一步matlab计算很慢，应换成追赶法求vk
% for i = 1: n-2
%     a(i) = A(i, i); % 对角线
%     if i == n-3
%         b(i) = A(i, i+1); % 上边
%         d(i) = A(i+1, i); % 下边
%         continue;
%     elseif i < n-2
%         b(i) = A(i, i+1); % 上边
%         d(i) = A(i+1, i); % 下边
%     end
% end
% [~, ~, wk_] = crout(a, b, d, c); % 追赶法
n_ = length(wk);
q1 = q(1) + T0*v0 + ((T0^2)/3)*a0 + ((T0^2)/6)*wk(1);
Tn_1 = t(n) - t(n-1);
qn_1 = q(n-2) - Tn_1*vn + ((Tn_1^2)/3)*an + ((Tn_1^2)/6)*wk(n_);
% 更新位置q向量
q = [q(1), q1, q(2: n-3), qn_1, q(n-2)];
% 更新加速度w向量
w = [a0, wk', an];

% 规划样条轨迹
T = t(n) - t(1); % 运行总时长
nn = T / tt; % 总点数
yy = zeros(1, nn);
dyy = zeros(1, nn);
ddyy = zeros(1, nn);
j = 1;
for i = 1: n-1
    Tk = t(i+1) - t(i);
    a0 = q(i);
    a1 = (q(i+1)-q(i))/Tk-(Tk/6)*(w(i+1)+2*w(i));
    a2 = w(i) / 2;
    a3 = (w(i+1)-w(i))/(6*Tk);
    
    for tk = t(i): tt: t(i+1)
        if i > 1 && tk == t(i)
            continue
        end
        yy(j) = a0 + a1*(tk-t(i)) + a2*power(tk-t(i), 2) + a3*power(tk-t(i), 3);
        dyy(j) = a1 + 2*a2*(tk-t(i)) + 3*a3*power(tk-t(i), 2);
        ddyy(j) = 2*a2 + 6*a3*(tk-t(i));
        j = j + 1;
    end
end

end




