%% 三角式拾取放置操作（位置+单元四元数姿态插补） + S型加减速曲线归一化处理
% 复现论文(在同一个平面内，论文中所有点的x值均不变)
% 圆弧段匀速
% 转接点的姿态不知道
P0 = [369.72,-127.17,402.87];
P1 = [369.72,-160.89,450.87];
P2 = [369.72,-190.57,402.87];
P3 = [369.72,-290.57,650.79];
P4 = [369.72,-390.57,402.87];

Q0 = Quaternion([1 0 0 0]);
Q1 = Quaternion([1 0 0 0]);
Q2 = Quaternion([1 0 0 0]);
Q3 = Quaternion([1 0 0 0]);
Q4 = Quaternion([1 0 0 0]);
t = 0.002;
r1 = 4; r2 = 10; r3 = 20;
v_p0 = 0; v_p4 = 0; v_p1 = 360; v_p2 = 144; v_p3 = 288;
amax = 1200; jmax = 7500;

% 获得转接参数function [Pt1, Pt2, d1, d2, C, theta] = PathSegmentTrans(P0, P1, P2, r)
[Pt1 Pt2 d1 d2 C1 theta1] = PathSegmentTrans(P0, P1, P2, r1);
[Pt3 Pt4 d3 d4 C2 theta2] = PathSegmentTrans(P1, P2, P3, r2);
[Pt5 Pt6 d5 d6 C3 theta3] = PathSegmentTrans(P2, P3, P4, r3);
% 最后一段分段长度d7
d7 = sqrt(power(P4(1) - Pt6(1), 2) + power(P4(2) - Pt6(2), 2) + power(P4(3) - Pt6(3), 2));
% 获取转接运动参数（速度）function [vt, alima, jmax] = TransMotionPara(d1, d2, d3, v0, v1, v2, amax, jmax)
[vt1] = TransMotionPara(d1, d2, d3, v_p0, v_p1, v_p2, amax, jmax);
[vt2] = TransMotionPara(d3, d4, d5, vt1, v_p2, v_p3, amax, jmax);
[vt3] = TransMotionPara(d5, d6, d7, vt2, v_p3, v_p4, amax, jmax);

% 进行插补，直线段的最大速度均为圆弧连接点速度，圆弧均为匀速段
% 直线段均为加速或减速曲线，因此需要比较两端点速度大小，以决定最大速度
% 基于S曲线的空间直线插补算法function [x y z qk N, qd, qdd, T] = SpaceLine_Q(S, D, Qs, Qd, v0, v1, vmax, amax, jmax, t)
[x1 y1 z1 qk1 N1 qd1 qdd1 T1] = SpaceLine_Q(P0, Pt1, Q0, Q1, v_p0, vt1, vt1, amax, jmax, t);

% 过渡圆弧的插补算法，搭配pick_and_place使用
% function [x y z qk N Tt] = Transition_arc(Pt1, Pt2, P1, Qt1, Qt2, d, r, t, vt)
[x2 y2 z2 qk2 N2 T2] = Transition_arc(Pt1, Pt2, P1, Q0, Q1, d2, r1, t, vt1);

[x3 y3 z3 qk3 N3 qd3 qdd3 T3] = SpaceLine_Q(Pt2, Pt3, Q1, Q2, vt1, vt2, vt1, amax, jmax, t);

[x4 y4 z4 qk4 N4 T4] = Transition_arc(Pt3, Pt4, P2, Q1, Q2, d4, r2, t, vt2);

[x5 y5 z5 qk5 N5 qd5 qdd5 T5] = SpaceLine_Q(Pt4, Pt5, Q3, Q4, vt2, vt3, vt3, amax, jmax, t);

[x6 y6 z6 qk6 N6 T6] = Transition_arc(Pt5, Pt6, P3, Q3, Q4, d6, r3, t, vt3);

[x7 y7 z7 qk7 N7 qd7 qdd7 T7] = SpaceLine_Q(Pt6, P4, Q3, Q4, vt3, v_p4, vt3, amax, jmax, t); % 就是这个有问题



figure(1)
plot3(C1(1), C1(2), C1(3), '*')
hold on
plot3(C2(1), C2(2), C2(3), '*')
hold on
plot3(C3(1), C3(2), C3(3), '*')
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
hold on 
plot3(x6, y6, z6, '*')
hold on
plot3(x7, y7, z7, '*')

for i = 1: N2
    vt_1(i) = vt1;
    a1(i) = 0;
end
for i = 1: N4
    vt_2(i) = vt2;
    a2(i) = 0;
end
for i = 1: N6
    vt_3(i) = vt3;
    a3(i) = 0;
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
hold on 
plot([T5+T4+T3+T2+T1: t: T6+T5+T4+T3+T2+T1], vt_3)
hold on
plot([T6+T5+T4+T3+T2+T1: t: T7+T6+T5+T4+T3+T2+T1], qd7)

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
hold on
plot([T5+T4+T3+T2+T1: t: T6+T5+T4+T3+T2+T1], a3)
hold on
plot([T6+T5+T4+T3+T2+T1: t: T7+T6+T5+T4+T3+T2+T1], qdd7)







