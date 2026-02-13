
# データセット (修正版)
dataset = [
    (0, "None", 100.0, 100.0), (1, "1", 94.9, 95.3), (1, "8", 95.6, 96.3), (1, "5", 97.3, 98.7),
    (1, "1,2", 92.1, 94.4), (2, "4,5", 94.2, 95.3), (2, "7,8", 89.9, 91.3), (2, "2,5", 93.5, 94.4),
    (3, "1,2,3", 89.8, 89.8), (3, "4,5,6", 90.2, 91.5), (3, "3,4,6", 89.5, 90.6), (3, "2,5,8", 88.3, 89.6),
    (4, "3,4,5,6", 82.8, 82.5), (4, "2,4,6,8", 83.7, 83.1), (4, "1,2,3,5", 83.8, 84.1),
    (5, "2,3,4,5,6", 52.8, 52.0), (5, "1,3,5,6,7", 79.5, 79.0),  
]




import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker  # 目盛り調整用に追加

def plot_equivalence_fixed_legend(plot_data, delta=5.0):
    # データを逆転（No.0が一番上に来るように）
    plot_data_rev = plot_data[::-1]
    plt.rcParams['font.family'] = 'MS Gothic' # Windowsの場合など
    
    labels = [f"故障{d[0]}個: [{d[1]}]" for d in plot_data_rev]
    diffs = [d[2] for d in plot_data_rev]
    lowers = [d[3] for d in plot_data_rev]
    uppers = [d[4] for d in plot_data_rev]
    
    y_pos = np.arange(len(labels))

    plt.figure(figsize=(12, 8))
    
    # 1. 等価性マージン (±5%)
    plt.axvline(x=-delta, color='red', linestyle='--', linewidth=1.5, label=f'等価性マージン (±{delta}%)')
    plt.axvline(x=delta, color='red', linestyle='--')
    
    # 2. 中央線 (0%)
    plt.axvline(x=0, color='black', linewidth=0.8, alpha=0.5, label='性能差なし (0%)')

    # --- ダミープロットによる凡例の固定表示 ---
    # 実際のデータにオレンジがなくても、凡例に表示させるためのテクニックです
    plt.plot([], [], 'o', color='blue', label='等価 (90% 信頼区間)')
    plt.plot([], [], 'o', color='orange', label='非等価 (90% 信頼区間)')
    ax = plt.gca()
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1)) # ここで1刻みを指定

    # 3. 信頼区間のプロット
    for i in range(len(labels)):
        is_equivalent = (lowers[i] > -delta and uppers[i] < delta)
        color = 'blue' if is_equivalent else 'orange'
        
        plt.errorbar(diffs[i], y_pos[i], 
                     xerr=[[diffs[i]-lowers[i]], [uppers[i]-diffs[i]]], 
                     fmt='o', color=color, capsize=6, elinewidth=2.5, 
                     markeredgewidth=2)

    # 4. グラフの装飾
    plt.yticks(y_pos, labels)
    plt.xlabel('成功率の差 (%) [分散知能 vs 集中制御]')
    plt.ylabel('関節故障状況 (故障数: [故障箇所ID])')
    plt.title(f'TOST等価性検定の集計結果 (マージン: $\pm${delta}%)')
    
    # 凡例を「右上 (upper right)」に配置
    plt.legend(loc='upper right', frameon=True, shadow=True)
    
    plt.grid(axis='x', linestyle=':', alpha=0.6)
    
    # x軸の範囲を調整（すべてのヒゲとマージンが見えるように）
    all_vals = np.concatenate([lowers, uppers, [-delta, delta]])
    plt.xlim(min(all_vals)-1, max(all_vals)+1)
    
    plt.tight_layout()
    plt.show()

# --- データ計算と実行 ---
# あなたのdatasetを使用してfinal_plot_dataを作成
final_plot_data = []
n_trials = 1000
alpha = 0.05
z_crit = stats.norm.ppf(1 - alpha)

for num_f, detail, p_dist_val, p_cent_val in dataset:
    p_dist = p_dist_val / 100.0
    p_cent = p_cent_val / 100.0
    diff = (p_dist - p_cent) * 100
    
    se_dist = np.sqrt(max(p_dist * (1 - p_dist), 1e-9) / n_trials)
    se_cent = np.sqrt(max(p_cent * (1 - p_cent), 1e-9) / n_trials)
    se_total = np.sqrt(se_dist**2 + se_cent**2)
    
    margin = z_crit * se_total * 100
    # (故障数, 詳細, 差, 下限, 上限)
    final_plot_data.append((num_f, detail, diff, diff - margin, diff + margin))

# グラフを表示
plot_equivalence_fixed_legend(final_plot_data)