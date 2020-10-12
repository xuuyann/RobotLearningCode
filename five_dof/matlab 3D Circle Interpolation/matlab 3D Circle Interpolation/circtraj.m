
% 圆弧轨迹规划
% tt = circtraj(t0, t1, t2,n)
%
% t0 -- 圆弧起始点对应的齐次变换矩阵
% t1 -- 圆弧中间点对应的齐次变换矩阵
% t2 -- 圆弧结束点对应的齐次变换矩阵
% n --  插补点数
% tt --  各插补点齐次变换矩阵

function tt = circtraj(t0, t1, t2,n)
        
    p0 = transl(t0);
    p1 = transl(t1);
    p2 = transl(t2);
    N = n;
    
    pr = CirInterpolation(p0',p1',p2',N);  %位置插补，圆弧插补算法
    
    q0 = quaternion(t0);
  % q1 = quaternion(t1);       % 该参数未使用            
    q2 = quaternion(t2);
    
    tt = zeros(4,4,0);
    for i=1:1:N,    
        
        r=(i-1)/(n-1);
	    qr = qinterp(q0, q2, r);          %姿态插补，球形线性插值法
        
	    t = [qr.r pr(i,:)'; 0 0 0 1];
       
	    tt = cat(3, tt, t);      %三维矩阵，即各插补点的齐次变换矩阵
   end
    
    