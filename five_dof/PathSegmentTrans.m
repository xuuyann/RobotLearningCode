% 拾取放置操作分段路径转接模型
% 	Copyright: xuuyann
% 	Author: xuuyann
% 	Description: 
% 参数： 用户设定的三个拾放路径点P0,P1,P2的位置
%       过渡半径r
%       插补周期t
% 返回值： 转接点Pt1，Pt2，分割后的小路径长度d1,d2,圆心C，圆心角theta
%         
function [Pt1, Pt2, d1, d2, C, theta] = PathSegmentTrans(P0, P1, P2, r, t)
% 求拐角theta
P1P0 = sqrt(power(P0(1) - P1(1), 2) + power(P0(2) - P1(2), 2) + power(P0(3) - P1(3), 2));
P1P2 = sqrt(power(P2(1) - P1(1), 2) + power(P2(2) - P1(2), 2) + power(P2(3) - P1(3), 2));
vec_P1P0 = [P0(1) - P1(1), P0(2) - P1(2), P0(3) - P1(3)];
vec_P1P2 = [P2(1) - P1(1), P2(2) - P1(2), P2(3) - P1(3)];
theta = acos((dot(vec_P1P0, vec_P1P2)) / (P1P0*P1P2));
% 求转接点Pt1、Pt2
vec_P1Pt1 = (r/tan(theta/2)/P1P0) * vec_P1P0;
vec_P1Pt2 = (r/tan(theta/2)/P1P2) * vec_P1P2;
Pt1 = P1 + vec_P1Pt1;
Pt2 = P1 + vec_P1Pt2;
% 求路径长度d1、弧长d2
d1 = sqrt(power(Pt1(1) - P0(1), 2) + power(Pt1(2) - P0(2), 2) + power(Pt1(3) - P0(3), 2));
d2 = (pi - theta) * r;
% % 求转接速度vt
% 这是考虑机械系统动力学匀速因素得到的转接速度
% a = sqrt(Amax * r);
% b = d2 / t;
% if (a > b)
%     vt  = b;
% else
%     vt = a;
% end
% 求圆心C
vec_Pt1M = (1/2) * (Pt2 - Pt1);
M = Pt1 + vec_Pt1M;
vec_P1M = M - P1;
P1M = sqrt(power(M(1) - P1(1), 2) + power(M(2) - P1(2), 2) + power(M(3) - P1(3), 2));
P1C = r / sin(theta/2);
vec_P1C = (P1C / P1M) * vec_P1M;
C = P1 + vec_P1C;
end