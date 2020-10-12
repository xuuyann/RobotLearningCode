%% 计算加加速度
function qddd = S_jerk(t, Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin)
T = Ta + Tv + Td;
if (t >= 0 && t < Tj1)
    qddd = jmax;
elseif (t >= Tj1 && t < Ta - Tj1)
    qddd = 0;
elseif (t >= Ta - Tj1 && t < Ta)
    qddd = jmin;
% 匀速段
elseif (t >= Ta && t < Ta + Tv)
    qddd = 0;
% 减速段
elseif (t >= Ta + Tv && t < T - Td + Tj2)
    qddd = -jmax;
elseif (t >= T - Td + Tj2 && t < T - Tj2)
    qddd = 0;
elseif (t >= T - Tj2 && t <= T)
    qddd = jmax;
end

end