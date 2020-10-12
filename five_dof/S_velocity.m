%% 计算速度
function qd = S_velocity(t, Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin)
T = Ta + Tv + Td;
if (t >= 0 && t < Tj1)
    qd = v0 + jmax*(t^2/2);
elseif (t >= Tj1 && t < Ta - Tj1)
    qd = v0 + alima*(t - Tj1/2);
elseif (t >= Ta - Tj1 && t < Ta)
    qd = vlim + jmin*(power(Ta - t, 2)/2);
% 匀速段
elseif (t >= Ta && t < Ta + Tv)
    qd = vlim;
% 减速段
elseif (t >= Ta + Tv && t < T - Td + Tj2)
    qd = vlim - jmax*(power(t - T + Td, 2)/2);
elseif (t >= T - Td + Tj2 && t < T - Tj2)
    qd = vlim + alimd*(t - T + Td - Tj2/2);
elseif (t >= T - Tj2 && t <= T)
    qd = v1 + jmax*(power(t - T, 2)/2);
end

end