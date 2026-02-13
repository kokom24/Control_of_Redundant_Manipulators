function [x_new, angle] = runge_kutta_n(L, K, D, n, x_vec, dt, x_full, y_full)
    [k1, angle] = f_n(L, K, D, n, x_vec, x_full, y_full);
    [k2, ~] = f_n(L, K, D, n, x_vec + 0.5 * dt * k1, x_full, y_full);
    [k3, ~] = f_n(L, K, D, n, x_vec + 0.5 * dt * k2, x_full, y_full);
    [k4, ~] = f_n(L, K, D, n, x_vec + dt * k3, x_full, y_full);

    x_new = x_vec + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end
