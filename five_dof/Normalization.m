% 归一化处理
% 梯形加减速曲线
% 输入参数：机械臂末端运动总位移（角度）pos 
%          机械臂末端线速度（角速度）vel
%          加速度减速度accl（设定加减速段accl相同）
%          插值点数N
function lambda = Normalization(pos, vel, accl, N)
% 加减速段的时间和位移
pos
T1 = vel / accl;
S1 = (1/2) * accl * T1^2
% 总时间
Te = 2*T1 + (pos - 2*S1) / vel;
% 归一化处理
S1_ = S1 / pos;
T1_ = T1 / Te;
T2_ = 1 - T1_;
accl_ = 2*S1_ / (T1_^2);
% lambda求解公式
for i = 0: N
    t = i / N;
    if (t >= 0 && t <= T1_)
        lambda(i+1) = (1/2) * accl_ * t^2;
    elseif (t > T1_ && t <= T2_)
        lambda(i+1) = (1/2)*accl_*T1_^2 + accl_*T1_*(t - T1_);
    elseif (t > T2_ && t <= 1)
        lambda(i+1) = (1/2)*accl_*T1_^2 + accl_*T1_*(t - T1_) - (1/2)*accl_*power(t - T2_, 2); % 重新考虑
    end
end
            
end


