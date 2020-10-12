% 三次样条：指定初始速度v0和终止速度vn，但是中间点速度未知
% Input：
%   q：给定点的位置
%   t：给定点位置对应的时间
%   v0：初始速度
%   vn：终止速度
%   tt：插补周期
% Output：
%   yy dyy ddyy：样条曲线函数值、速度、加速度值
function [yy dyy ddyy] = cubicSpline_2(q, t, v0, vn, tt);
if length(q) ~= length(t)
    error('输入的数据应成对');
end
n = length(q);
c = zeros(n-2, 1);
% 矩阵A是个(n-2)*(n-2)的对角占优矩阵
A = zeros(n-2);
for i = 1: n-2
    Tk_1 = t(i+2) - t(i+1);
    Tk = t(i+1) - t(i);
    if i == 1
        A(i, i) = 2*(Tk + Tk_1);
        A(i, i+1) = Tk;
        c(i, 1) = (3/(Tk*Tk_1))*(Tk^2*(q(i+2)-q(i+1))+Tk_1^2*(q(i+1)-q(i))) - Tk_1*v0;
    elseif i == n-2
        A(i, i-1) = Tk_1;
        A(i, i) = 2*(Tk + Tk_1);
        c(i, 1) = (3/(Tk*Tk_1))*(Tk^2*(q(i+2)-q(i+1))+Tk_1^2*(q(i+1)-q(i))) - Tk*vn;
    else
        A(i, i-1) = Tk_1;
        A(i, i) = 2*(Tk + Tk_1);
        A(i, i+1) = Tk;
        c(i, 1) = (3/(Tk*Tk_1))*(Tk^2*(q(i+2)-q(i+1))+Tk_1^2*(q(i+1)-q(i)));
        
    end
end
% 经过上述步骤得到对角占优矩阵A和c
% vk = A \ c; % 这一步matlab计算很慢，应换成追赶法求vk
for i = 1: n-2
    a(i) = A(i, i); % 对角线
    if i == n-3
        b(i) = A(i, i+1); % 上边
        d(i) = A(i+1, i); % 下边
        continue;
    elseif i < n-2
        b(i) = A(i, i+1); % 上边
        d(i) = A(i+1, i); % 下边
    end
end
[~, ~, vk] = crout(a, b, d, c); % 追赶法

% 得到中间插补点的速度vk，然后调用cubicSpline_1即可
v_ = [v0, vk, vn];
[yy dyy ddyy] = cubicSpline_1(q, t, v_, tt);

end

    
    
    
    
    
    
    