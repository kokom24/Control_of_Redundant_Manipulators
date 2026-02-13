% Kinova Gen3のロボットモデルを読み込む
gen3 = loadrobot("kinovaGen3");

% ロボットのホームポジションを定義する（適宜変更してください）
homeConfig = homeConfiguration(gen3);

% ロボットのアニメーションを作成するための目標位置を設定する
targetPosition = [0.5, 0.2, 0.4]; % [x, y, z]

% 目標位置を姿勢情報に変換する
targetPose = trvec2tform(targetPosition);

% ロボットのホームポジションからIKを使用して目標位置への関節角度を計算する
ik = inverseKinematics('RigidBodyTree', gen3);
ik.SolverParameters.AllowRandomRestart = false; % ランダムリスタートを無効にする
weights = [0.25, 0.25, 0.25, 1, 1, 1]; % 重み付け
initialGuess = homeConfig; % 初期推測
configSoln = ik('EndEffector_Link', targetPose, weights, initialGuess); % 修正

% アニメーションのフレーム数と速度を設定する
numFrames = 50;
motionPath = linspace(0, 1, numFrames);

% ロボットのアニメーションを作成する
figure;
for i = 1:numFrames
    % 目標位置への関節角度を補間する
    interpConfig = interpolateJointPositions(configSoln, homeConfig, motionPath(i));
    
    % ロボットをプロットする
    show(gen3, interpConfig, 'PreservePlot', false);
    hold on;
    
    % 目標位置を赤い球体としてプロットする
    scatter3(targetPosition(1), targetPosition(2), targetPosition(3), 100, 'r', 'filled');
    
    % グラフの範囲を設定する（適宜変更してください）
    axis([-0.2 0.7 -0.7 0.7 0 1.2]);
    title("Kinova Gen3のアニメーション");
    hold off;
    drawnow;
end

function interpConfig = interpolateJointPositions(endConfig, startConfig, t)
    numJoints = numel(startConfig);
    interpConfig = startConfig; % 初期構造体をコピー
    for j = 1:numJoints
        % 各関節角度を補間
        interpConfig(j).JointPosition = (1-t) * startConfig(j).JointPosition + t * endConfig(j).JointPosition;
    end
end
