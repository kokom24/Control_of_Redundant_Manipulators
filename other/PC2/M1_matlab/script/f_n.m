function [dx, angle] = f_n(L, K, D, n, x_vec, x_full, y_full)
    x = x_vec(1:n-1);
    y = x_vec(n:2*(n-1));
    vx = [0, x_vec(2*(n-1)+1:3*(n-1)), 0];
    vy = [0, x_vec(3*(n-1)+1:end), 0];

    x_full = [x_full(1), x, x_full(end)];
    y_full = [y_full(1), y, y_full(end)];

    angle = zeros(1, n);
    for i = 1:n
        dx_i = x_full(i+1) - x_full(i);
        dy_i = y_full(i+1) - y_full(i);
        angle(i) = atan2(dy_i, dx_i);
    end

    dL = zeros(1, n);
    for i = 1:n
        dL(i) = hypot(x_full(i+1)-x_full(i), y_full(i+1)-y_full(i)) - L;
    end

    ax = zeros(1, n-1);
    ay = zeros(1, n-1);
    for i = 1:n-1
        ax(i) = K*(dL(i+1)*cos(angle(i+1)) - dL(i)*cos(angle(i))) + ...
                D*(vx(i+2) - 2*vx(i+1) + vx(i));
        ay(i) = K*(dL(i+1)*sin(angle(i+1)) - dL(i)*sin(angle(i))) + ...
                D*(vy(i+2) - 2*vy(i+1) + vy(i));
    end

    dx = [vx(2:end-1), vy(2:end-1), ax, ay];
end
