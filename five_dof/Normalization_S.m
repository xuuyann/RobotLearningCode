% 归一化处理
% S型加减速曲线
% 	Copyright: xuuyann
% 	Author: xuuyann
% 	Description: 
% 输入参数：机械臂末端运动总位移（角度）pos 
%          机械臂末端线速度（角速度）初始v0，终止v1，最大速度vmax
%          最大加减速度amax、最大加加速度jmax
%          插补周期t
% 返回值： 插值点数N,qd,qdd(为了作图),运行时间T
function [lambda, N, qd, qdd, T] = Normalization_S(pos, v0, v1, vmax, amax, jmax, t)
% S曲线参数计算（S型速度规划，又称七段式轨迹）
% function para = STrajectoryPara(q0, q1, v0, v1, vmax, amax, jmax)
q0 = 0; q1 = pos;
para = STrajectoryPara(q0, q1, v0, v1, vmax, amax, jmax); % 获取S曲线参数
% 总的规划时间
T = para(1) + para(2) + para(3);
% 等时插补
N = ceil(T / t);
j = 1;
for i = 0 : t: T
   q(j) = S_position(i, para(1), para(2), para(3), para(4), para(5), para(6), para(7), para(8), para(9), para(10), para(11), para(12), para(13), para(14), para(15), para(16));
   qd(j) = S_velocity(i, para(1), para(2), para(3), para(4), para(5), para(6), para(7), para(8), para(9), para(10), para(11), para(12), para(13), para(14), para(15), para(16));
   qdd(j) = S_acceleration(i, para(1), para(2), para(3), para(4), para(5), para(6), para(7), para(8), para(9), para(10), para(11), para(12), para(13), para(14), para(15), para(16));
   lambda(j) = q(j) / pos;
   j = j + 1;
end

end