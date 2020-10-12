% 四元数转换为旋转矩阵
function R = q2tr(q)
r11 = power(q.s, 2) + power(q.v(1), 2) - power(q.v(2), 2) - power(q.v(3), 2);
r12 = 2*(q.v(1)*q.v(2) - q.s*q.v(3));
r13 = 2*(q.v(1)*q.v(3) + q.s*q.v(2));
r21 = 2*(q.v(1)*q.v(2) + q.s*q.v(3));
r22 = power(q.s, 2) - power(q.v(1), 2) + power(q.v(2), 2) - power(q.v(3), 2);
r23 = 2*(q.v(2)*q.v(3) - q.s*q.v(1));
r31 = 2*(q.v(1)*q.v(3) - q.s*q.v(2));
r32 = 2*(q.v(2)*q.v(3) + q.s*q.v(1));
r33 = power(q.s, 2) - power(q.v(1), 2) - power(q.v(2), 2) + power(q.v(3), 2);

R = [r11, r12, r13;
     r21, r22, r23;
     r31, r32, r33];
end