clear; clc;

% パラメータ設定
n = 5;              % リンク数
L = 1;              % リンクの長さ
K = 10;             % ばね定数
D = 5;              % 減衰定数
dt = 0.03;          % タイムステップ
tmax = 10;          % シミュレーション時間（秒）

% 初期位置と角度
initial_pos = [0, 0];
theta0 = pi/4;  % 初期角度45度

% 初期リンク座標計算
angle = theta0 * ones(1, n);
x = zeros(1, n+1);
y = zeros(1, n+1);
for i = 1:n
    x(i+1) = x(i) + L * cos(angle(i));
    y(i+1) = y(i) + L * sin(angle(i));
end

% 初期状態ベクトル（x, y, vx, vy）
x_state = [x(2:end-1), y(2:end-1), zeros(1, 2*(n-1))];

% 目標位置（固定）
goal = [n*L*0.9, n*L*0.5];

% 描画準備
figure;
axis equal;
xlim([-n*L, n*L]);
ylim([-n*L, n*L]);

t = 0;
while t < tmax
    t = t + dt;

    % 状態更新（ルンゲ・クッタ）
    [x_state, angle] = runge_kutta_n(L, K, D, n, x_state, dt, x, y);

    % 各リンクの位置を更新
    link_x = zeros(1, n+1);
    link_y = zeros(1, n+1);
    link_x(1) = initial_pos(1);
    link_y(1) = initial_pos(2);
    for i = 1:n
        link_x(i+1) = link_x(i) + L * cos(angle(i));
        link_y(i+1) = link_y(i) + L * sin(angle(i));
    end

    % 描画
    cla;
    hold on;
    for i = 1:n
        plot([link_x(i), link_x(i+1)], [link_y(i), link_y(i+1)], 'bo-', 'LineWidth', 2);
    end
    plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    title(sprintf("t = %.2f", t));
    axis equal;
    xlim([-n*L, n*L]);
    ylim([-n*L, n*L]);
    drawnow;
    pause(0.01);
end
