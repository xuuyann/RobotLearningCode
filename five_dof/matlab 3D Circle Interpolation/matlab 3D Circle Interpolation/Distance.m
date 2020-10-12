
%  计算三维空间两点的距离
%  d = Distance(p1,p2)
% 
%  p1，p2 -- 空间中任意两点的坐标
%  d -- 两点间直线距离

function d = Distance(p1,p2)

 d = sqrt((p1(1)-p2(1)).^2 + (p1(2)-p2(2)).^2 + (p1(3)-p2(3)).^2);
 
 