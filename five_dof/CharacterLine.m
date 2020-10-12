%% 字符轮廓线（位置+单元四元数姿态插补） + S型加减速曲线归一化处理
% 	Copyright: xuuyann
% 	Author: xuuyann
% 	Description: 
%   圆弧段匀速
%   转接点的姿态不知道
%   目前是“S”字符

P0 = [5.1353, 0, 20];
P1 = [45.1912, 0, 20];
P2 = [45.1912, 22, 20];
P3 = [12.3248, 28, 20];
P4 = [12.3248, 39, 20];
P5 = [46.2183, 39, 20];
P6 = [46.2183, 45.9184, 20];
P7 = [5.1353, 46.0409, 20];
P8 = [5.1115, 22, 20];
P9 = [38.0017, 16, 20];
P10 = [38.0017, 7, 20];
P11 = [5.1353, 7, 20];
% 本程序虽然没有考虑具体姿态，但是给姿态留了参数的位置，便于以后加入
Q0 = Quaternion([1 0 0 0]);
Q1 = Quaternion([1 0 0 0]);
Q2 = Quaternion([1 0 0 0]);
Q3 = Quaternion([1 0 0 0]);
Q4 = Quaternion([1 0 0 0]);
Q5 = Quaternion([1 0 0 0]);
Q6 = Quaternion([1 0 0 0]);
Q7 = Quaternion([1 0 0 0]);
Q8 = Quaternion([1 0 0 0]);
Q9 = Quaternion([1 0 0 0]);
Q10 = Quaternion([1 0 0 0]);
Q11 = Quaternion([1 0 0 0]);
t = 0.002;

r1 = 4; r2 = 4; r3 = 4; r4 = 4; r5 = 3; 
r6 = 3; r7 = 4; r8 = 4; r9 = 4; r10 = 4; r11 = 4;
    
v_p0 = 0; v_p1 = 54; v_p2 = 54; v_p3 = 54; v_p4 = 54;
v_p5 = 54; v_p6 = 54; v_p7 = 54; v_p8 = 54; v_p9 = 54;
v_p10 = 54; v_p11 = 54;
% v_p0 = 0; v_p1 = 0; v_p2 = 0; v_p3 = 0; v_p4 = 0;
% v_p5 = 0; v_p6 = 0; v_p7 = 0; v_p8 = 0; v_p9 = 0;
% v_p10 = 0; v_p11 = 0;
% r1 = 0; r2 = 0; r3 = 0; r4 = 0; r5 = 0; 
% r6 = 0; r7 = 0; r8 = 0; r9 = 0; r10 = 0; r11 = 0;
amax = 600; jmax = 3000;

% 获得转接参数function [Pt1, Pt2, d1, d2, C, theta] = PathSegmentTrans(P0, P1, P2, r)
% S字符总共十一段圆弧
[Pt1 Pt2 d1 d2 C1 theta1] = PathSegmentTrans(P0, P1, P2, r1);
[Pt3 Pt4 d3 d4 C2 theta2] = PathSegmentTrans(P1, P2, P3, r2);
[Pt5 Pt6 d5 d6 C3 theta3] = PathSegmentTrans(P2, P3, P4, r3);
[Pt7 Pt8 d7 d8 C4 theta4] = PathSegmentTrans(P3, P4, P5, r4);
[Pt9 Pt10 d9 d10 C5 theta5] = PathSegmentTrans(P4, P5, P6, r5);
[Pt11 Pt12 d11 d12 C6 theta6] = PathSegmentTrans(P5, P6, P7, r6);
[Pt13 Pt14 d13 d14 C7 theta7] = PathSegmentTrans(P6, P7, P8, r7);
[Pt15 Pt16 d15 d16 C8 theta8] = PathSegmentTrans(P7, P8, P9, r8);
[Pt17 Pt18 d17 d18 C9 theta9] = PathSegmentTrans(P8, P9, P10, r9);
[Pt19 Pt20 d19 d20 C10 theta10] = PathSegmentTrans(P9, P10, P11, r10);
[Pt21 Pt22 d21 d22 C11 theta11] = PathSegmentTrans(P10, P11, P0, r11);

% 最后一段分段长度d23
d23 = sqrt(power(P0(1) - Pt22(1), 2) + power(P0(2) - Pt22(2), 2) + power(P0(3) - Pt22(3), 2));
% 获取转接运动参数（速度）function [vt, alima, jmax] = TransMotionPara(d1, d2, d3, v0, v1, v2, amax, jmax)
% 有几段圆弧就有几个转接速度
vt1 = TransMotionPara(d1, d2, d3, v_p0, v_p1, v_p2, amax, jmax);
vt2 = TransMotionPara(d3, d4, d5, vt1, v_p2, v_p3, amax, jmax);
vt3 = TransMotionPara(d5, d6, d7, vt2, v_p3, v_p4, amax, jmax);
vt4 = TransMotionPara(d7, d8, d9, vt3, v_p4, v_p5, amax, jmax);
vt5 = TransMotionPara(d9, d10, d11, vt4, v_p5, v_p6, amax, jmax);
vt6 = TransMotionPara(d11, d12, d13, vt5, v_p6, v_p7, amax, jmax);
vt7 = TransMotionPara(d13, d14, d15, vt6, v_p7, v_p8, amax, jmax);
vt8 = TransMotionPara(d15, d16, d17, vt7, v_p8, v_p9, amax, jmax);
vt9 = TransMotionPara(d17, d18, d19, vt8, v_p9, v_p10, amax, jmax);
vt10 = TransMotionPara(d19, d20, d21, vt9, v_p10, v_p11, amax, jmax);
vt11 = TransMotionPara(d21, d22, d23, vt10, v_p11, v_p0, amax, jmax);

% 进行插补，直线段的最大速度均为圆弧连接点速度，圆弧均为匀速段
% 直线段均为加速或减速曲线，因此需要比较两端点速度大小，以决定最大速度
% 基于S曲线的空间直线插补算法function [x y z qk N, qd, qdd, T] = SpaceLine_Q(S, D, Qs, Qd, v0, v1, vmax, amax, jmax, t)
[x1 y1 z1 qk1 N1 qd1 qdd1 T1] = SpaceLine_Q(P0, Pt1, Q0, Q1, v_p0, vt1, vt1, amax, jmax, t);

% 过渡圆弧的插补算法，搭配pick_and_place使用
% function [x y z qk N Tt] = Transition_arc(Pt1, Pt2, P1, Qt1, Qt2, d, r, t, vt)
[x2 y2 z2 qk2 N2 T2] = Transition_arc(Pt1, Pt2, P1, Q0, Q1, d2, r1, t, vt1);

[x3 y3 z3 qk3 N3 qd3 qdd3 T3] = SpaceLine_Q(Pt2, Pt3, Q1, Q2, vt1, vt2, vt2, amax, jmax, t);

[x4 y4 z4 qk4 N4 T4] = Transition_arc(Pt3, Pt4, P2, Q1, Q2, d4, r2, t, vt2);

[x5 y5 z5 qk5 N5 qd5 qdd5 T5] = SpaceLine_Q(Pt4, Pt5, Q3, Q4, vt2, vt3, vt2, amax, jmax, t);

[x6 y6 z6 qk6 N6 T6] = Transition_arc(Pt5, Pt6, P3, Q3, Q4, d6, r3, t, vt3);

[x7 y7 z7 qk7 N7 qd7 qdd7 T7] = SpaceLine_Q(Pt6, Pt7, Q3, Q4, vt3, vt4, vt4, amax, jmax, t); 

[x8 y8 z8 qk8 N8 T8] = Transition_arc(Pt7, Pt8, P4, Q5, Q6, d8, r4, t, vt4);

[x9 y9 z9 qk9 N9 qd9 qdd9 T9] = SpaceLine_Q(Pt8, Pt9, Q5, Q6, vt4, vt5, vt5, amax, jmax, t);

[x10 y10 z10 qk10 N10 T10] = Transition_arc(Pt9, Pt10, P5, Q6, Q7, d10, r5, t, vt5);

[x11 y11 z11 qk11 N11 qd11 qdd11 T11] = SpaceLine_Q(Pt10, Pt11, Q6, Q7, vt5, vt6, vt5, amax, jmax, t);

[x12 y12 z12 qk12 N12 T12] = Transition_arc(Pt11, Pt12, P6, Q6, Q7, d12, r6, t, vt6);

[x13 y13 z13 qk13 N13 qd13 qdd13 T13] = SpaceLine_Q(Pt12, Pt13, Q7, Q8, vt6, vt7, vt7, amax, jmax, t);

[x14 y14 z14 qk14 N14 T14] = Transition_arc(Pt13, Pt14, P7, Q7, Q8, d14, r7, t, vt7);

[x15 y15 z15 qk15 N15 qd15 qdd15 T15] = SpaceLine_Q(Pt14, Pt15, Q8, Q9, vt7, vt8, vt8, amax, jmax, t);

[x16 y16 z16 qk16 N16 T16] = Transition_arc(Pt15, Pt16, P8, Q9, Q10, d16, r8, t, vt8);

[x17 y17 z17 qk17 N17 qd17 qdd17 T17] = SpaceLine_Q(Pt16, Pt17, Q9, Q10, vt8, vt9, vt9, amax, jmax, t);

[x18 y18 z18 qk18 N18 T18] = Transition_arc(Pt17, Pt18, P9, Q9, Q10, d18, r9, t, vt9);

[x19 y19 z19 qk19 N19 qd19 qdd19 T19] = SpaceLine_Q(Pt18, Pt19, Q10, Q11, vt9, vt10, vt10, amax, jmax, t);

[x20 y20 z20 qk20 N20 T20] = Transition_arc(Pt19, Pt20, P10, Q10, Q11, d20, r10, t, vt10);

[x21 y21 z21 qk21 N21 qd21 qdd21 T21] = SpaceLine_Q(Pt20, Pt21, Q10, Q11, vt10, vt11, vt11, amax, jmax, t);

[x22 y22 z22 qk22 N22 T22] = Transition_arc(Pt21, Pt22, P11, Q10, Q11, d22, r11, t, vt11);

[x23 y23 z23 qk23 N23 qd23 qdd23 T23] = SpaceLine_Q(Pt22, P0, Q10, Q11, vt11, v_p0, vt11, amax, jmax, t);

figure(1)
plot3(P0(1), P0(2), P0(3), 'r*')
grid on
hold on
plot3(P1(1), P1(2), P1(3), 'b*')
hold on
plot3(P2(1), P2(2), P2(3), 'b*')
hold on
plot3(P3(1), P3(2), P3(3), 'b*')
hold on
plot3(P4(1), P4(2), P4(3), 'b*')
hold on
plot3(P5(1), P5(2), P5(3), 'b*')
hold on
plot3(P6(1), P6(2), P6(3), 'b*')
hold on
plot3(P7(1), P7(2), P7(3), 'b*')
hold on
plot3(P8(1), P8(2), P8(3), 'b*')
hold on
plot3(P9(1), P9(2), P9(3), 'b*')
hold on
plot3(P10(1), P10(2), P10(3), 'b*')
hold on
plot3(P11(1), P11(2), P11(3), 'r*')
hold on

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
hold on
plot3(x8, y8, z8, '*')
hold on
plot3(x9, y9, z9, '*')
hold on
plot3(x10, y10, z10, '*')
hold on
plot3(x11, y11, z11, '*')
hold on
plot3(x12, y12, z12, '*')
hold on
plot3(x13, y13, z13, '*')
hold on
plot3(x14, y14, z14, '*')
hold on
plot3(x15, y15, z15, '*')
hold on
plot3(x16, y16, z16, '*')
hold on
plot3(x17, y17, z17, '*')
hold on
plot3(x18, y18, z18, '*')
hold on
plot3(x19, y19, z19, '*')
hold on
plot3(x20, y20, z20, '*')
hold on
plot3(x21, y21, z21, '*')
hold on
plot3(x22, y22, z22, '*')
hold on
plot3(x23, y23, z23, '*')
hold on


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
for i = 1: N8
    vt_4(i) = vt4;
    a4(i) = 0;
end
for i = 1: N10
    vt_5(i) = vt5;
    a5(i) = 0;
end
for i = 1: N12
    vt_6(i) = vt6;
    a6(i) = 0;
end
for i = 1: N14
    vt_7(i) = vt7;
    a7(i) = 0;
end
for i = 1: N16
    vt_8(i) = vt8;
    a8(i) = 0;
end
for i = 1: N18
    vt_9(i) = vt9;
    a9(i) = 0;
end
for i = 1: N20
    vt_10(i) = vt10;
    a10(i) = 0;
end
for i = 1: N22
    vt_11(i) = vt11;
    a11(i) = 0;
end
% 速度曲线
figure(2)
subplot(2, 1, 1)
plot([0: t: T1], qd1)  
grid on
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
hold on
plot([T7+T6+T5+T4+T3+T2+T1: t: T8+T7+T6+T5+T4+T3+T2+T1], vt_4)
hold on
plot([T8+T7+T6+T5+T4+T3+T2+T1: t: T9+T8+T7+T6+T5+T4+T3+T2+T1], qd9)
hold on
plot([T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_5)
hold on
plot([T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd11)
hold on
plot([T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_6)
hold on
plot([T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd13)
hold on
plot([T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_7)
hold on
plot([T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd15)
hold on
plot([T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_8)
hold on
plot([T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd17)
hold on
plot([T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_9)
hold on
plot([T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd19)
hold on
plot([T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_10)
hold on
plot([T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd21)
hold on
plot([T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T22+T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], vt_11)
hold on
plot([T22+T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T23+T22+T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd23)
hold on
% 加速度曲线
subplot(2, 1, 2)
plot([0: t: T1], qdd1)  
grid on
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
hold on
plot([T7+T6+T5+T4+T3+T2+T1: t: T8+T7+T6+T5+T4+T3+T2+T1], a4)
hold on
plot([T8+T7+T6+T5+T4+T3+T2+T1: t: T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd9)
hold on
plot([T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a5)
hold on
plot([T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd11)
hold on
plot([T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a6)
hold on
plot([T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd13)
hold on
plot([T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a7)
hold on
plot([T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd15)
hold on
plot([T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a8)
hold on
plot([T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd17)
hold on
plot([T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a9)
hold on
plot([T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd19)
hold on
plot([T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a10)
hold on
plot([T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd21)
hold on
plot([T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T22+T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], a11)
hold on
plot([T22+T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T23+T22+T21+T20+T19+T18+T17+T16+T15+T14+T13+T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd23)
hold on




