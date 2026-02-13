from collections import defaultdict
import matplotlib.pyplot as plt

# テキストファイルのパス
file_path = 'K100vsD10.txt'

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
        
        # 成功率を取得。空の場合はデフォルトで0.0を設定
        success_rate_str = lines[i + 2].strip().split(':')[1].strip()
        success_rate = float(success_rate_str) if success_rate_str else 0.0

        # データを辞書として格納
        entry = {"link": theta_0, "success_rate": success_rate}
        data.append(entry)

        i += 4  # 新しい形式では7行おきにデータが現れるため

# データをlinkを基準にソート
data.sort(key=lambda x: x["link"])

# データの表示
links = [entry["link"] for entry in data]
success_rates = [entry["success_rate"] for entry in data]

# データを折れ線グラフとしてプロット
plt.plot(links, success_rates, marker='o', linestyle='-')

# 縦軸の範囲を0から110まで、10ごとにグリッドを表示
plt.yticks(range(0, 111, 10))
# 横軸のグリッドを1ごとに表示
plt.xticks(range(min(links), max(links)+1, 1))

plt.xlabel('Link')
plt.ylabel('Success Rate')
plt.title('Success Rate vs Link')
plt.grid(True)
plt.savefig('success_rate.png')

plt.show()
