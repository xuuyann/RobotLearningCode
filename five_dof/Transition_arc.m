% 过渡圆弧的插补算法，搭配ContinueSpaceLine使用
% 	Copyright: xuuyann
% 	Author: xuuyann
% 	Description: 
% 参数： Pt1,Pt2,P1(Pt1和Pt2的中间点),Qt1,Qt2
%       过渡圆弧弧长d、过渡半径r、插补周期t
%       圆弧插补速度vt
% 返回值： 插补点数N，插补时间Tt
function [x y z qk N Tt] = Transition_arc(Pt1, Pt2, P1, Qt1, Qt2, d, r, t, vt)
% 建立新坐标系UVW
% 新坐标轴V
vec_Pt1P1 = P1 - Pt1;
Pt1P1 = sqrt(power(P1(1) - Pt1(1), 2) + power(P1(2) - Pt1(2), 2) + power(P1(3) - Pt1(3), 2));
V = (1/Pt1P1) * vec_Pt1P1;
ox = V(1); oy = V(2); oz = V(3);
% 新坐标系W
vec_Pt2P1 = P1 - Pt2;
vec_W_ = cross(vec_Pt1P1, vec_Pt2P1);
W_ = sqrt(power(vec_W_(1), 2) + power(vec_W_(2), 2) + power(vec_W_(3), 2));
W = (1/W_) * vec_W_;
ax = W(1); ay = W(2); az = W(3);
% 新坐标系U
U = cross(V, W);
nx = U(1); ny = U(2); nz = U(3);
% 相对于基座标系{O-XYZ}， 新坐标系{C-UVW}的坐标变换矩阵
T = [nx ox ax Pt1(1);
     ny oy ay Pt1(2);
     nz oz az Pt1(3);
      0  0  0  1];

% 计算两个四元数之间的夹角
dot_q = Qt1.s*Qt2.s + Qt1.v(1)*Qt2.v(1) + Qt1.v(2)*Qt2.v(2) + Qt1.v(3)*Qt2.v(3);
if (dot_q < 0)
    dot_q = -dot_q;
end
% 插补时长
Tt = d / vt;
N = floor(Tt / t);
i = 1;
for j = 0: N
    % 位置插补
    x_(i) = r -  r * cos(vt*j*t/r);
    y_(i) = r * sin(vt*j*t/r);
    P = T*[x_(i); y_(i); 0; 1];
    x(i) = P(1);
    y(i) = P(2);
    z(i) = P(3);
    % 单位四元数球面线性姿态插补
    % 插值点四元数
    if (dot_q > 0.9995)
        k0 = 1-t;
        k1 = t;
    else
        sin_t = sqrt(1 - power(dot_q, 2));
        omega = atan2(sin_t, dot_q);
        k0 = sin((1-t*omega)) / sin(omega);
        k1 = sin(t*omega) / sin(omega);
    end
    qk(i) = Qt1 * k0 + Qt2 * k1;
    N = i;
    i = i + 1;
end

end