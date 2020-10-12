%% 高斯列主元消去法
function x = Gauss_lie(n, A, b)
for k = 1: n-1
    a_max = abs(A(k, k));
    cnt = k;
    for i = k: n
        if (abs(A(i, k)) > a_max)
            a_max = abs(A(i, k));
            cnt = i; % 确定下标i
        end
    end
    if (A(cnt, k) == 0)
        fprintf('Gauss_lie: no unique solution\n');
        return;
    end
    % 换行
    if (cnt ~= k)
        t = 0; s = 0;
        for j = k: n
            t = A(k, j);
            A(k, j) = A(cnt, j);
            A(cnt, j) = t;
            s = b(cnt);
            b(cnt) = b(k);
            b(k) = s;
        end
    end
    % step 5
    for i = k+1: n
        L(i) = A(i, k) / A(k, k);
        for j = k+1: n
            A(i, j) = A(i, j) - L(i)*A(k, j);
        end
        b(i) = b(i) - L(i)*b(k);
    end
end
for i = 1: n
    if (A(i, i) == 0)
        fprintf('Gauss_lie no unique solution\n');
        return;
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
