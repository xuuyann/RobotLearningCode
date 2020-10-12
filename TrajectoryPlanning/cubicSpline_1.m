% 三次样条：指定初始速度v0和终止速度vn，并且中间插补点的速度已知，这是最简单的情况
% Input：
%   q：给定点的位置
%   t：给定点位置对应的时间
%   v：包括给定起始、中间及终止速度的速度向量
%   tt：插补周期
% Output：
%   yy dyy ddyy：样条曲线函数值、速度、加速度值
function [yy dyy ddyy] = cubicSpline_1(q, t, v, tt)
if length(q) ~= length(t)
    error('输入的数据应成对')
end
n = length(q);
T = t(n) - t(1); % 运行总时长
nn = T / tt; % 总点数
yy = zeros(1, nn);
dyy = zeros(1, nn);
ddyy = zeros(1, nn);
j = 1;
for i = 1: n-1
    Tk = t(i+1) - t(i);
    a0 = q(i);
    a1 = v(i);
    a2 = (1/Tk) * ((3*(q(i+1)-q(i)))/Tk - 2*v(i) - v(i+1));
    a3 = (1/(Tk*Tk)) * ((2*(q(i)-q(i+1)))/Tk + v(i) + v(i+1));
    
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



