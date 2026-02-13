import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import colormaps

# =============================================================================
# 1. データの読み込み
# =============================================================================

# jointごとのデータを格納
joint_data = {}

# 解析したいデータが含まれるフォルダのパス
base_path = r'C:\Users\rsait\OneDrive\ドキュメント\git\M1_styudy\result\2025\10\28\012909\trial_004\dis_range_17'

print(f"--- データ読み込み開始 ---")
print(f"対象フォルダ: {base_path}")

for joint_n in range(10):
    filename = f'{base_path}\\joint_data{joint_n}.txt'
    
    dL_list, theta_list, v_self_list, pos_self_list, pos_positive_list = [], [], [], [], []

    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"エラー: ファイルが見つかりません: {filename}")
        continue

    for line in lines:
        m_dl = re.match(r"Joint \d+ : dL_self: ([\d\.\-eE]+), angle: ([\d\.\-eE]+)", line)
        if m_dl:
            dL_list.append(float(m_dl.group(1)))
            continue
        m_v = re.match(r"v_self: \(([\d\.\-eE]+), ([\d\.\-eE]+)\)", line)
        if m_v:
            v_self_list.append((float(m_v.group(1)), float(m_v.group(2))))
            continue
        m_pos = re.match(r"pos_self: \(([\d\.\-eE]+), ([\d\.\-eE]+)\)", line)
        if m_pos:
            pos_self_list.append((float(m_pos.group(1)), float(m_pos.group(2))))
            continue
        # ▼▼▼▼▼ 修正点1: pos_positive の行を正規表現で探す処理を追加 ▼▼▼▼▼
        m_pos_p = re.match(r"pos_positive: \(([\d\.\-eE]+), ([\d\.\-eE]+)\)", line)
        if m_pos_p:
            pos_positive_list.append((float(m_pos_p.group(1)), float(m_pos_p.group(2))))
            continue
        # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    joint_data[joint_n] = {
        "dL_self": np.array(dL_list),
        "theta_self": np.array(theta_list),
        "v_self": np.array(v_self_list),
        "pos_self": np.array(pos_self_list),
    }
    # pos_positiveのデータがあれば辞書に追加
    if pos_positive_list:
        joint_data[joint_n]["pos_positive"] = np.array(pos_positive_list)


if not joint_data:
    print("データが読み込めませんでした。処理を終了します。")
    exit()

print("--- データ読み込み完了 ---")

# =============================================================================
# 2. 座標設定（目標位置のみ）
# =============================================================================

target_pos = (100,100)
print(f"目標位置は固定: {target_pos}")

# =============================================================================
# 3. 最終形状をPNG画像で保存
# =============================================================================
print("--- 最終形状の画像を生成中 ---")
max_len = min(len(data["pos_self"]) for data in joint_data.values() if len(data["pos_self"]) > 0)
fig_final, ax_final = plt.subplots(figsize=(8, 8))
final_frame = max_len - 1

# ▼▼▼▼▼ 修正点2: 描画する座標リストの作成方法を変更 ▼▼▼▼▼
# 1. 始点(0,0)を追加
final_xs, final_ys = [0.], [0.]

# 2. joint 0 から 9 までのジョイント位置(pos_self)を追加
for joint_n in range(10):
    pos = joint_data[joint_n]["pos_self"]
    if final_frame < len(pos):
        x, y = pos[final_frame]
        final_xs.append(x)
        final_ys.append(y)

# 3. joint 9 から手先位置(pos_positive)を追加
if 9 in joint_data and "pos_positive" in joint_data[9]:
    hand_pos_frames = joint_data[9]["pos_positive"]
    if final_frame < len(hand_pos_frames):
        hand_x, hand_y = hand_pos_frames[final_frame]
        final_xs.append(hand_x)
        final_ys.append(hand_y)
# ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

# プロット処理
ax_final.plot(target_pos[0], target_pos[1], 'o', color='red', markersize=10, label=f'Target {target_pos}')
ax_final.plot(final_xs, final_ys, 'o-', color='black', linewidth=3, markersize=7, label=f'Final Pose (t={final_frame})')

# グラフの体裁と保存
ax_final.set_title("Final Shape of Robot Arm")
ax_final.set_xlabel("x position")
ax_final.set_ylabel("y position")
ax_final.set_aspect('equal', 'box')
ax_final.legend()
ax_final.set_xlim(-10, 110)
ax_final.set_ylim(-10, 110)
ax_final.tick_params(direction='in')

output_filename = 'final_shape.png'
plt.savefig(output_filename, dpi=300)
print(f"最終形状を {output_filename} として保存しました。")
plt.close(fig_final)

# =============================================================================
# 4. アニメーションの生成と表示
# =============================================================================
# print("--- アニメーションを生成中 ---")
# # ... (アニメーション部分も同様の考え方で修正が必要です) ...
# # アニメーションの update 関数も、描画リストを作成する部分を上記と同様に
# # 「始点(0,0) -> joint0-9のpos_self -> joint9のpos_positive」の順で
# # データを追加するよう修正することで正しく動作します。