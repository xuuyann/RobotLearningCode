clear
%figure
axis equal
p1 = [5,0,0];
p2 = [0,5,0];
p3 = [0,0,5];
N = 100;
p = CirInterpolation(p1,p2,p3,N);
n = size(p);
for i=1:n
    plot3(p(i,1),p(i,2),p(i,3),'-*r');
    hold on
end
grid on
