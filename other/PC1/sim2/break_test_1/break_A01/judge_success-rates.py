from collections import defaultdict
import matplotlib.pyplot as plt

# テキストファイルのパス
file_path = 'success_rate.txt'

#  データを格納するリスト
data = []

# ファイルの読み込み
with open(file_path, 'r') as file:
    lines = file.readlines()
    i = 0
    while i < len(lines):
        # パラメータ部分を取得
        theta_0_line = lines[i + 1].strip().split(':')
        theta_0 = int(theta_0_line[1].strip())
        
        theta_n_line = lines[i + 2].strip().split(':')
        theta_n = int(theta_n_line[1].strip())
        
        # 成功率を取得。空の場合はデフォルトで0.0を設定
        success_rate_str = lines[i + 3].strip().split(':')[1].strip()
        success_rate = float(success_rate_str) if success_rate_str else 0.0

        # データを辞書として格納
        entry = {"theta_0": theta_0, "theta_n": theta_n, "success_rate": success_rate}
        data.append(entry)
        # print(entry)
        

        i += 5  # 新しい形式では7行おきにデータが現れるため

# 成功率ごとの出現回数を計算
success_rates_count = defaultdict(int)

for entry in data:
    success_rate = entry["success_rate"]
    success_rates_count[success_rate] += 1

# 結果を出力
for rate, count in sorted(success_rates_count.items(), reverse=True):
    print(f"{rate}%: {count}")
    
# ヒストグラムを作成
rates = list(success_rates_count.keys())
counts = list(success_rates_count.values())

plt.bar(rates, counts)
plt.xlabel('Success Rate[%]')
plt.ylabel('Count')
plt.title('Success Rate Histogram')
plt.show()
