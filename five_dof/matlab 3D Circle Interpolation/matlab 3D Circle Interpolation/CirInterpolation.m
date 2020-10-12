
%  空间圆弧插补函数
%  p = CirInterpolation(p1,p2,p3,N)
%  
%  p1 -- 圆弧起始点
%  p2 -- 圆弧中间点
%  p3 -- 圆弧终止点（注意p1,p2,p3这三点不在同一直线上）
%  N --  插补点数
%  p --  插补点矩阵 
%
%  算法原理：陈永明等《机器人的三种规则曲线插补算法》中的圆弧插补算法
%  作者：caleb     时间：2011-11-18



function p = CirInterpolation(p1,p2,p3,N)

 a = p1;
 b = p2;
 c = p3;

 O = CalCircleCenter(a,b,c);          % 计算过a,b,c三点圆的圆心坐标O
 R = Distance(a,O);                   % 计算圆半径R
 theta = CalCentralAngle(a,b,c,O,R);  % 计算圆弧对应的圆心角theta
 
 % 计算圆弧平面的法向量n
 AB = a-b;
 BC = b-c;
 n = cross(AB,BC);

 u = n(1);
 v = n(2);
 w = n(3);

 delta = theta/(N-1);             % 步距角
 ds = delta*R;                    % 插补中圆弧切向移动距离
 E = ds/( R*sqrt(u.^2+v.^2+w.^2));
 G = R/sqrt(R.^2+ds.^2);

 xO = O(1);
 yO = O(2);
 zO = O(3);

 p(1,:) = a;          % 圆弧起始坐标
 p(N,:) = c;          % 圆弧结束坐标

 % 计算中间插补点
for i=1:1:N-2,
    
  xi = p(i,1);
  yi = p(i,2);
  zi = p(i,3);

  mi = v*(zi-zO)-w*(yi-yO);
  ni = w*(xi-xO)-u*(zi-zO);
  li = u*(yi-yO)-v*(xi-xO);
  
  x = xO + G*(xi + E*mi-xO);
  y = yO + G*(yi + E*ni-yO);
  z = zO + G*(zi + E*li-zO);

  p(i+1,:) = [x y z]; 
  
end






