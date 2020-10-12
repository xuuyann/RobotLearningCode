%% 计算加速度
function qdd = S_acceleration(t, Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin)
T = Ta + Tv + Td;
if (t >= 0 && t < Tj1)
    qdd = jmax*t;
elseif (t >= Tj1 && t < Ta - Tj1)
    qdd = alima;
elseif (t >= Ta - Tj1 && t < Ta)
    qdd = -jmin*(Ta - t);
% 匀速段
elseif (t >= Ta && t < Ta + Tv)
    qdd = 0;
% 减速段
elseif (t >= Ta + Tv && t < T - Td + Tj2)
    qdd = -jmax*(t - T + Td);
elseif (t >= T - Td + Tj2 && t < T - Tj2)
    qdd = alimd;
elseif (t >= T - Tj2 && t <= T)
    qdd = -jmax*(T - t);
end

end