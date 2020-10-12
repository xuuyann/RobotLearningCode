% S = [369.72,-127.17,402.87];
% M = [113.3 603.4 194.6];
% D = [369.72,-127.12,650.87];
S = [1 2 3];
M = [3 4 5];
D = [9 7 6];


Qs_ = Quaternion([0 0 0 0]);
Qd_ = Quaternion([0 0 0 0]);
v0 = 0; v1 = 0.180; vmax = 0.500; amax = 0.1500; jmax = 0.7500; 
% N = 8;
% S_ = [-2.96278366186295,1.57079632679490,0.178808991726845];
% D_ = [0,0.698131700797732,3.14159265358979];

t = 0.02;

% [x y z qk N] = SpaceLine_Q(S, D, Qs_, Qd_, v0, v1, vmax, amax, jmax, t);
[x y z qk N] = SpaceCircle_Q(S, M, D, Qs_, Qd_, v0, v1, vmax, amax, jmax, t);
% figure(1)
plot3(x, y, z, '.')
grid on
hold on 
plot3(S(1), S(2), S(3), 'go')
% plot3(M(1), M(2), M(3), 'bo')
plot3(D(1), D(2), D(3), 'ro')
% figure(2)
% subplot(3, 1, 1);
% plot([0: t: 1.311], x)
% ylabel('X')
% grid on
% subplot(3, 1, 2);
% plot([0: t: 1.311], y)
% ylabel('Y')
% grid on
% subplot(3, 1, 3)
% plot([0: t: 1.311], z)
% ylabel('Z')
% grid on
% j = 1;
% for i = 0: t: 1.311
%     if (j == N)
%         break
%     end
%     vx(j) = (x(j+1) - x(j))/t;
%     vy(j) = (y(j+1) - y(j))/t;
%     vz(j) = (z(j+1) - z(j))/t;
%     j = j + 1;
% end
% figure(3)
% subplot(3, 1, 1);
% plot([0: t: 1.3111-t], vx)
% ylabel('VX')
% grid on
% subplot(3, 1, 2);
% plot([0: t: 1.3111-t], vy)
% ylabel('VY')
% grid on
% subplot(3, 1, 3)
% plot([0: t: 1.3111-t], vz)
% ylabel('VZ')
% grid on
    
    
    