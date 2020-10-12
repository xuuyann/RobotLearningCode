% 三次样条：周期三次样条，没有指定初始速度v0和终止速度vn，也就是v0和vn未知
% Input：
%   q：给定点的位置
%   t：给定点位置对应的时间
%   tt：插补周期
% Output：
%   yy dyy ddyy：样条曲线函数值、速度、加速度值
function [yy dyy ddyy] = cubicSpline_3(q, t, tt)
if length(q) ~= length(t)
    error('输入的数据应成对');
end
n = length(q);
c = zeros(n-1, 1);
% 矩阵A是个(n-1)*(n-1)的循环三对角矩阵
A = zeros(n-1);
for i = 1: n-1
    if i == 1
        Tn_1 = t(n) - t(n-1);
        T0 = t(i+1) - t(i);
        A(i, i) = 2*(Tn_1 + T0);
        A(i, i+1) = Tn_1;
        A(i, n-1) = T0;
        c(i, 1) = (3/(Tn_1*T0))*((Tn_1^2)*(q(i+1)-q(i))+(T0^2)*(q(n)-q(n-1)));
    else
        Tk_1 = t(i+1) - t(i);
        Tk = t(i) - t(i-1);
        c(i, 1) = (3/(Tk*Tk_1))*(Tk^2*(q(i+1)-q(i))+Tk_1^2*(q(i)-q(i-1)));
        if i == n-1
            A(i, 1) = Tk_1;
            A(i, i-1) = Tk_1;
            A(i, i) = 2*(Tk + Tk_1);
        else
            A(i, i-1) = Tk_1;
            A(i, i) = 2*(Tk + Tk_1);
            A(i, i+1) = Tk;
        end
    end
            
end
% 经过上述步骤得到矩阵A和c
vk = A \ c; % 这一步matlab计算很慢，应换成追赶法求vk
% 这个vk的第一个值为v0，然后v0和vn相等
% 得到中间插补点的速度vk，然后调用cubicSpline_1即可
v_ = [vk', vk(1)];
% v_ = [-2.28 -2.78 2.99 5.14 2.15 -1.8281 -2.28]
[yy dyy ddyy] = cubicSpline_1(q, t, v_, tt);

end






