# failed_simulation.py
import re
import random

# ----------------------------
# 1. ログファイルから失敗座標を抽出
# ----------------------------
failed_coords = []

log_file = r"C:\Users\rsait\OneDrive\ドキュメント\git\M1_styudy\result\2025\10\28\result.txt"  # ログファイル名を適宜変更
import re

failed_coords = []

with open(r"C:\Users\rsait\OneDrive\ドキュメント\git\M1_styudy\result\2025\10\28\result.txt", "r") as f:
    for line in f:
        # "-> 0" の行だけ対象
        if "-> 0" in line:
            # Goal (x, y) の部分を正規表現で抽出
            match = re.search(r"Goal \(([^,]+), ([^)]+)\)", line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                failed_coords.append((x, y))

if not failed_coords:
    raise ValueError("失敗座標が見つかりません。ログの形式を確認してください。")

print(f"失敗座標: {failed_coords}")
print("成功" if success else "失敗")
