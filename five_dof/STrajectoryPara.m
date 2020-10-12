% 单关节空间规划
%% S曲线参数计算（S型速度规划，又称七段式轨迹）
% 	Copyright: xuuyann
% 	Author: xuuyann
function para = STrajectoryPara(q0, q1, v0, v1, vmax, amax, jmax)
% 得到规划参数Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin
% 用户给定参数
% 边界条件q0 = 10, q1 = 0, v0 = -7, v1 = 0
% 约束条件vmax = 10, amax = 10, jamx = 30
% q0 = 0; q1 = 10; 
% v0 = 7.5; v1 = 0;
% vmax = 10; amax = 10; jmax = 30;
vmin = -vmax; amin = -amax; jmin = -jmax;

%% 利用公式（3.31）（3.32）转化得到实际的q_0、q_1、v_max、a_max
sigma = sign(q1 - q0);  
q_0 = sigma*q0;
q_1 = sigma*q1;
v_0 = sigma*v0;
v_1 = sigma*v1;
v_max = ((sigma+1)/2)*vmax + ((sigma-1)/2)*vmin;
v_min = ((sigma+1)/2)*vmin + ((sigma-1)/2)*vmax;
a_max = ((sigma+1)/2)*amax + ((sigma-1)/2)*amin;
a_min = ((sigma+1)/2)*amin + ((sigma-1)/2)*amax;
j_max = ((sigma+1)/2)*jmax + ((sigma-1)/2)*jmin;
j_min = ((sigma+1)/2)*jmin + ((sigma-1)/2)*jmax;

%% 判断是否达到最大速度
if ((v_max - v_0)*j_max < a_max^2) 
    Tj1 = sqrt((v_max - v_0) / j_max); % 达不到a_max
    Ta = 2*Tj1;
    a_lima = j_max * Tj1;
else
    Tj1 = a_max / j_max; % 能够达到a_max
    Ta = Tj1 + (v_max - v_0) / a_max;
    a_lima = a_max; 
end
if ((v_max - v_1)*j_max < a_max^2)
    Tj2 = sqrt((v_max - v_1) / j_max); % 达不到a_min
    Td = 2*Tj2;
    a_limd =  -j_max * Tj2;
else
    Tj2 = a_max / j_max; % 能够达到a_min
    Td = Tj2 + (v_max - v_1) / a_max;
    a_limd = -a_max;
end
% 根据（3.25）计算匀速段时间
Tv = (q_1 - q_0)/v_max - (Ta/2)*(1 + v_0/v_max) - (Td/2)*(1 + v_1/v_max);

%% 对Tv进行讨论
if (Tv > 0)
    % 达到最大速度v_max，即存在匀速阶段
    vlim = v_max;
    T = Ta + Tv + Td;
    para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
    return;
else
    % 达不到最大速度，即匀速阶段Tv=0
    % 假设最大加速度和最小加速度均能达到
    Tv = 0;
    Tj = a_max / j_max;
    Tj1 = Tj;
    Tj2 = Tj;
    delta = (a_max^4/j_max^2) + 2*(v_0^2 + v_1^2) + a_max*(4*(q_1 - q_0) - 2*(a_max/j_max)*(v_0 + v_1));
    Ta = ((power(a_max, 2)/j_max) - 2.0*v_0 + sqrt(delta)) / (2.0*a_max);
    Td = ((power(a_max, 2)/j_max) - 2.0*v_1 + sqrt(delta)) / (2.0*a_max);
    % 对Ta和Td进行讨论
    if (Ta < 0 || Td < 0)
        if (Ta < 0)
            % 没有加速段，只有减速段
            Ta = 0; Tj1 = 0;
            Td = 2*(q_1 - q_0) / (v_0 + v_1);
            Tj2 = (j_max*(q_1 - q_0) - sqrt(j_max*(j_max*power(q_1 - q_0, 2) + power(v_1 + v_0, 2)*(v_1 - v_0)))) / (j_max*(v_1 + v_0));
            a_lima = 0;
            a_limd = -j_max*Tj2;
            vlim = v0;
            para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
            return;
        elseif (Td < 0)
            % 没有减速段，只有加速段
            Td = 0; Tj2 = 0;
            Ta = 2*(q_1 - q_0) / (v_0 + v_1);
            Tj1 = (j_max*(q_1 - q_0) - sqrt(j_max*(j_max*power(q_1 - q_0, 2)) - power(v_1 + v_0, 2)*(v_1 - v_0))) / (j_max*(v_1 + v_0));
            a_lima = j_max*Tj1;
            a_limd = 0;
            vlim = v0 + a_lima*(Ta - Tj1);
            para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
            return;
        end
    elseif (Ta >= 2*Tj && Td >= 2*Tj)
        % 加速段和减速段都能达到最大加速度
        a_lima = a_max;
        a_limd = -a_max;
        vlim = v0 + a_lima*(Ta - Tj);
        para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
        return;
    else
        % 加速和减速阶段至少有一段不能达到最大加速度
        lambda = 0.99; % 系统取0<lambda<1
        while (Ta < 2*Tj || Td < 2*Tj)
            % 循环
            a_max = lambda*a_max;
            Tv = 0;
            Tj = a_max / j_max;
            Tj1 = Tj;
            Tj2 = Tj;
            delta = (a_max^4/j_max^2) + 2*(v_0^2 + v_1^2) + a_max*(4*(q_1 - q_0) - 2*(a_max/j_max)*(v_0 + v_1));
            Ta = ((power(a_max, 2)/j_max) - 2.0*v_0 + sqrt(delta)) / (2.0*a_max);
            Td = ((power(a_max, 2)/j_max) - 2.0*v_1 + sqrt(delta)) / (2.0*a_max);
            if (Ta < 0 || Td < 0)
                if (Ta < 0)
                    % 没有加速段，只有减速段
                    Ta = 0; Tj1 = 0;
                    Td = 2*(q_1 - q_0) / (v_0 + v_1);
                    Tj2 = (j_max*(q_1 - q_0) - sqrt(j_max*(j_max*power(q_1 - q_0, 2) + power(v_1 + v_0, 2)*(v_1 - v_0)))) / (j_max*(v_1 + v_0));
                    a_lima = 0;
                    a_limd = -j_max*Tj2;
                    vlim = v0;
                    para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
                    return;
                elseif (Td < 0)
                    % 没有减速段，只有加速段
                    Td = 0; Tj2 = 0;
                    Ta = 2*(q_1 - q_0) / (v_0 + v_1);
                    Tj1 = (j_max*(q_1 - q_0) - sqrt(j_max*(j_max*power(q_1 - q_0, 2)) - power(v_1 + v_0, 2)*(v_1 - v_0))) / (j_max*(v_1 + v_0));
                    a_lima = j_max*Tj1;
                    a_limd = 0;
                    vlim = v0 + a_lima*(Ta - Tj1);
                    para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
                    return;
                end
            elseif (Ta >= 2*Tj && Td >= 2*Tj)
                % 加速段和减速段都能达到最大加速度
                a_lima = a_max;
                a_limd = -a_max;
                vlim = v0 + a_lima*(Ta - Tj);
                para = [Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min];
                return;
            end
        end
    end
end
end
