% function [x y z alp beta gama N] = SpaceCircle(S, M, D, S_, D_, ws, a)
S = [0 0.2 0];
M = [-1.886613719554240;1.583056876251960;2.280400460880988];
D = [0.25 0.2 0];
S_ = [0 0 0]; D_ = [0 0 0];
ws = 10; wa = 5;
vs = 0.01; va = 0.001;

% [x y z alp beta gama N] = SpaceCircle(S', M', D', S_, D_, ws, wa);
[x y z alp beta gama N] = SpaceLine(S, D, S_, D_, vs, va);

plot3(x, y, z, '.')
hold on
grid on
plot3(S(1), S(2), S(3), 'g*')
% plot3(M(1), M(2), M(3), 'g*')
plot3(D(1), D(2), D(3), 'g*')
