
% 计算圆弧AOC的圆心角AOC
% theta = CalCentralAngle(a,b,c,O,r) 
% 
% a -- 圆弧起始点
% b -- 圆弧中间点
% c -- 圆弧终止点
% O -- 圆弧的圆心
% r -- 圆半径
% theta -- 圆心角AOC 


function theta = CalCentralAngle(a,b,c,O,r) 

 % 计算法向量
 AB = a-b;
 BC = b-c;
 n = cross(AB,BC);

 OA = O-a;
 AC = a-c;
 n1 = cross(OA,AC);

 H = dot(n,n1);
 d = (c(1)-a(1)).^2 + (c(2)-a(2)).^2 + (c(3)-a(3)).^2;

 if H<0
    theta = 2*pi - 2*asin(sqrt(d)/(2*r));
 else
    theta = 2*asin(sqrt(d)/(2*r));
 end
    