%% 这是常规方法，即每一段路径的首尾速度均为0
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

v_p0 = 0; v_p1 = 0; v_p2 = 0; v_p3 = 0; v_p4 = 0;
v_p5 = 0; v_p6 = 0; v_p7 = 0; v_p8 = 0; v_p9 = 0;
v_p10 = 0; v_p11 = 0;

amax = 600; jmax = 3000;

vmax = 54;
[x1_ y1_ z1_ qk1_ N1_, qd1_, qdd1_, T1] = SpaceLine_Q(P0, P1, Q0, Q1, v_p0, v_p1, vmax, amax, jmax, t);
[x2_ y2_ z2_ qk2_ N2_, qd2_, qdd2_, T2] = SpaceLine_Q(P1, P2, Q1, Q2, v_p1, v_p2, vmax, amax, jmax, t);
[x3_ y3_ z3_ qk3_ N3_, qd3_, qdd3_, T3] = SpaceLine_Q(P2, P3, Q2, Q3, v_p2, v_p3, vmax, amax, jmax, t);
[x4_ y4_ z4_ qk4_ N4_, qd4_, qdd4_, T4] = SpaceLine_Q(P3, P4, Q3, Q4, v_p3, v_p4, vmax, amax, jmax, t);
[x5_ y5_ z5_ qk5_ N5_, qd5_, qdd5_, T5] = SpaceLine_Q(P4, P5, Q4, Q5, v_p4, v_p5, vmax, amax, jmax, t);
[x6_ y6_ z6_ qk6_ N6_, qd6_, qdd6_, T6] = SpaceLine_Q(P5, P6, Q5, Q6, v_p5, v_p6, vmax, amax, jmax, t);
[x7_ y7_ z7_ qk7_ N7_, qd7_, qdd7_, T7] = SpaceLine_Q(P6, P7, Q6, Q7, v_p6, v_p7, vmax, amax, jmax, t);
[x8_ y8_ z8_ qk8_ N8_, qd8_, qdd8_, T8] = SpaceLine_Q(P7, P8, Q7, Q8, v_p7, v_p8, vmax, amax, jmax, t);
[x9_ y9_ z9_ qk9_ N9_, qd9_, qdd9_, T9] = SpaceLine_Q(P8, P9, Q8, Q9, v_p8, v_p9, vmax, amax, jmax, t);
[x10_ y10_ z10_ qk10_ N10_, qd10_, qdd10_, T10] = SpaceLine_Q(P9, P10, Q9, Q10, v_p9, v_p10, vmax, amax, jmax, t);
[x11_ y11_ z11_ qk11_ N11_, qd11_, qdd11_, T11] = SpaceLine_Q(P10, P11, Q10, Q11, v_p10, v_p11, vmax, amax, jmax, t);
[x12_ y12_ z12_ qk12_ N12_, qd12_, qdd12_, T12] = SpaceLine_Q(P11, P0, Q11, Q11, v_p11, v_p0, vmax, amax, jmax, t);

subplot(2, 1, 1)
plot([0: t: T1], qd1_)  
grid on
hold on
plot([T1: t: T2+T1], qd2_)
hold on
plot([T2+T1: t: T3+T2+T1], qd3_)
hold on
plot([T3+T2+T1: t: T4+T3+T2+T1], qd4_)
hold on 
plot([T4+T3+T2+T1: t: T5+T4+T3+T2+T1], qd5_)
hold on 
plot([T5+T4+T3+T2+T1: t: T6+T5+T4+T3+T2+T1], qd6_)
hold on
plot([T6+T5+T4+T3+T2+T1: t: T7+T6+T5+T4+T3+T2+T1], qd7_)
hold on
plot([T7+T6+T5+T4+T3+T2+T1: t: T8+T7+T6+T5+T4+T3+T2+T1], qd8_)
hold on
plot([T8+T7+T6+T5+T4+T3+T2+T1: t: T9+T8+T7+T6+T5+T4+T3+T2+T1], qd9_)
hold on
plot([T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd10_)
hold on
plot([T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd11_)
hold on
plot([T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qd12_)
subplot(2, 1, 2)
plot([0: t: T1], qdd1_)  
grid on
hold on
plot([T1: t: T2+T1], qdd2_)
hold on
plot([T2+T1: t: T3+T2+T1], qdd3_)
hold on
plot([T3+T2+T1: t: T4+T3+T2+T1], qdd4_)
hold on 
plot([T4+T3+T2+T1: t: T5+T4+T3+T2+T1], qdd5_)
hold on 
plot([T5+T4+T3+T2+T1: t: T6+T5+T4+T3+T2+T1], qdd6_)
hold on
plot([T6+T5+T4+T3+T2+T1: t: T7+T6+T5+T4+T3+T2+T1], qdd7_)
hold on
plot([T7+T6+T5+T4+T3+T2+T1: t: T8+T7+T6+T5+T4+T3+T2+T1], qdd8_)
hold on
plot([T8+T7+T6+T5+T4+T3+T2+T1: t: T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd9_)
hold on
plot([T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd10_)
hold on
plot([T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd11_)
hold on
plot([T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1: t: T12+T11+T10+T9+T8+T7+T6+T5+T4+T3+T2+T1], qdd12_)


