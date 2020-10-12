%% 门式拾取放置操作（位置+单元四元数姿态插补） + S型加减速曲线归一化处理
% 复现论文(在同一个平面内，论文中所有点的x值均不变)
% 四个初始点P0、P1、P2、P3
% 圆弧段匀速
% 转接点的姿态不知道
P0 = [369.72,-127.17,402.87];
P1 = [369.72,-127.12,650.87];
P2 = [369.72,-350.17,650.87];
P3 = [369.72,-350.17,402.87];
Q0 = Quaternion([1 0 0 0]);
Q1 = Quaternion([1 0 0 0]);
Q2 = Quaternion([1 0 0 0]);
Q3 = Quaternion([1 0 0 0]);
r = 18; t = 0.002;
v_p0 = 0; v_p3 = 0; vmax_1 = 180; vmax_2 = 360; vmax_3 = 360;
amax = 1500; jmax = 7500;

% 获得转接参数function [Pt1, Pt2, d1, d2, vt, C, theta] = PathSegmentTrans(P0, P1, P2, r, Amax, t)
[Pt1 Pt2 d1 d2 vt1 C1 theta1] = PathSegmentTrans(P0, P1, P2, r, amax, t);
[Pt3 Pt4 d3 d4 vt2 C2 theta2] = PathSegmentTrans(P1, P2, P3, r, amax, t);
% 基于S曲线的空间直线插补算法function [x y z qk N, qd, qdd, T] = SpaceLine_Q(S, D, Qs, Qd, v0, v1, vmax, amax, jmax, t)
[x1 y1 z1 qk1 N1 qd1 qdd1 T1] = SpaceLine_Q(P0, Pt1, Q0, Q1, v_p0, vt1, vmax_1, amax, jmax, t);

if (vt1 == 0)
    N2 = 0;
    T2 = 0;
else
    N2 = ceil((d2/vt1) / t); 
    T2 = d2 / vt1;
end

% 过渡圆弧的插补算法function [x y z qk] = Transition_arc(Pt1, Pt2, P1, Qt1, Qt2, d, r, t, vt)
[x2 y2 z2 qk2] = Transition_arc(Pt1, Pt2, P1, Q2, Q3, d2, r, t, vt1);

[x3 y3 z3 qk3 N3 qd3 qdd3 T3] = SpaceLine_Q(Pt2, Pt3, Q2, Q3, vt1, vt2, vmax_2, amax, jmax, t);

if (vt2 == 0)
    N4 = 0;
    T4 = 0;
else
    N4 = ceil((d4/vt2) / t); 
    T4 = d4 / vt2;
end
% 过渡圆弧的插补算法function [x y z qk] = Transition_arc(Pt1, Pt2, P1, Qt1, Qt2, d, r, t, vt)
[x4 y4 z4 qk4 N4 T4] = Transition_arc(Pt3, Pt4, P2, Q2, Q3, d2, r, t, vt2);

[x5 y5 z5 qk5 N5 qd5 qdd5 T5] = SpaceLine_Q(Pt4, P3, Q2, Q3, vt2, v_p3, vmax_3, amax, jmax, t);

figure(1)
plot3(C1(1), C1(2), C1(3), '*')
hold on
plot3(C2(1), C2(2), C2(3), '*')
hold on
plot3(x1, y1, z1, '*')
grid on
hold on
plot3(x2, y2, z2, '*')
hold on
plot3(x3, y3, z3, '*')
hold on 
plot3(x4, y4, z4, '*')
hold on
plot3(x5, y5, z5, '*')

for i = 1: N2
    vt_1(i) = vt1;
    a1(i) = 0;
end
for i = 1: N4
    vt_2(i) = vt2;
    a2(i) = 0;
end
figure(2)
subplot(2, 1, 1)
plot([0: t: T1], qd1)   
hold on
plot([T1: t: T2+T1], vt_1)
hold on
plot([T2+T1: t: T3+T2+T1], qd3)
hold on
plot([T3+T2+T1: t: T4+T3+T2+T1], vt_2)
hold on 
plot([T4+T3+T2+T1: t: T5+T4+T3+T2+T1], qd5)
subplot(2, 1, 2)
plot([0: t: T1], qdd1)   
hold on
plot([T1: t: T2+T1], a1)
hold on
plot([T2+T1: t: T3+T2+T1], qdd3)
hold on
plot([T3+T2+T1: t: T4+T3+T2+T1], a2)
hold on 
plot([T4+T3+T2+T1: t: T5+T4+T3+T2+T1], qdd5)







