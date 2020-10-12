% 获取转接运动参数（速度加速度）
% 	Copyright: xuuyann
% 	Author: xuuyann
% 	Description: 
% 参数： 分段路径长度d1,d2,d3
%       P0、P1、P2的速度v0, v1, v2（P1为中间点）
%       最大加速度amax,最大加加速度jmax
% 返回值： 转接点速度vt（即为路径上能够达到的最大速度）
function [vt] = TransMotionPara(d1, d2, d3, v0, v1, v2, amax, jmax)
% 得到规划参数Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin
q0 = 0; q1 = d1 + d2 + d3
para = STrajectoryPara(q0, q1, v0, v2, v1, amax, jmax);
vt = para(10);
end