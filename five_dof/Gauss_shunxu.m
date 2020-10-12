function x = Gauss_shunxu(n, A, b)
for i = 1: n
    for j = 1: n
        if (A(i, j) == 0)
            fprintf('Gauss_shunxu no unique solution\n');
            return;
        end
    end
end
% 消元
for k = 1: n-1
    for i = k+1: n
        L(i) = A(i, k) / A(k, k);
        for j = k+1: n
            A(i, j) = A(i, j) - L(i)*A(k, j);
        end
        b(i) = b(i) - L(i)*b(k);
    end
end
for i = 1: n
    for j = 1: n
        if (A(i, j) == 0)
            fprintf('Gauss_shunxu no unique solution\n');
            return;
        end
    end
end
% 回代
x(n) = b(n) / A(n, n);
for i = n-1: -1: 1
    sum_a = 0;
    for j = i+1: n
        sum_a = sum_a + A(i, j)*x(j);
    end
    x(i) = (b(i) - sum_a) / A(i, i);
end
end



