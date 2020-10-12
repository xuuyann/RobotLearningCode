% 求叉积 ab X bc = ui+vj+wk
% 等于matlab函数 cross(ab,bc)

function n = xj(a,b,c)

u = (b(2)-a(2))*(c(3)-b(3))-(b(3)-a(3))*(c(2)-b(2));
v = (b(3)-a(3))*(c(1)-b(1))-(b(1)-a(1))*(c(3)-b(3));
w = (b(1)-a(1))*(c(2)-b(2))-(b(2)-a(2))*(c(1)-b(1));

n=[u v w];

