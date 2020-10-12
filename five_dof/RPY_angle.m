    % 求RPY角（没有解决奇点问题）
% 目前返回值是弧度制
function [alpha beta gama] = RPY_angle(fk_T)
nx = fk_T(1, 1); ox = fk_T(1, 2); ax = fk_T(1, 3); 
ny = fk_T(2, 1); oy = fk_T(2, 2); ay = fk_T(2, 3); 
nz = fk_T(3, 1); oz = fk_T(3, 2); az = fk_T(3, 3); 
beta = atan2(-nz, sqrt(nx^2 + ny^2));
if (cos(beta) < 1e-5)
    fprintf('Singularity!The beta = +-pi/2\n');
else
    alpha = atan2(ny/cos(beta), nx/cos(beta));
    gama = atan2(oz/cos(beta), az/cos(beta));
    
end