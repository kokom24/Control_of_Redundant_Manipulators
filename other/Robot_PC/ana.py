import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import os

# ===============================
# 設定
# ===============================
CSV_PATH = "アルゴリズム検証-20250103-1.csv"
OUTPUT_PATH = "overlay.png"

# ===============================
# CSV 読み込み（OptiTrack特殊形式）
# ===============================
df_raw = pd.read_csv(CSV_PATH, sep="\t", header=None)
df_split = df_raw[0].str.split(",", expand=True)

HEADER_ROW = 6
DATA_START = 7

df_split.columns = df_split.iloc[HEADER_ROW]
df = df_split.iloc[DATA_START:].reset_index(drop=True)
df = df.apply(pd.to_numeric, errors="coerce")

# ===============================
# Marker列番号（XZ平面）
# ===============================
markers = [
    [27, 29],  # Marker 001 (X, Z)
    [30, 32],  # Marker 002
    [33, 35],  # Marker 003
    [36, 38],  # Marker 004
]

# ===============================
# figure
# ===============================
plt.figure(figsize=(6, 6))

skipped = 0

# ===============================
# 重ね描き
# ===============================
for _, row in tqdm(df.iterrows(), total=len(df)):
    xs, zs = [], []

    for m in markers:
        x = row.iloc[m[0]]
        z = row.iloc[m[1]]

        if not (np.isnan(x) or np.isnan(z)):
            xs.append(x)
            zs.append(z)

    if len(xs) < 4:
        skipped += 1
        continue

    # ---- 四角形化 ----
    points = np.column_stack([xs, zs])
    center = points.mean(axis=0)

    angles = np.arctan2(
        points[:, 1] - center[1],
        points[:, 0] - center[0]
    )

    points = points[np.argsort(angles)]
    points = np.vstack([points, points[0]])

    # ---- 描画 ----
    plt.plot(points[:, 0], points[:, 1], color="blue", alpha=0.05)
    plt.fill(points[:, 0], points[:, 1], color="blue", alpha=0.02)

# ===============================
# 仕上げ
# ===============================
# ---- 軸設定 ----
plt.xlabel("X [mm]")
plt.ylabel("Z [mm]")
plt.title("EE marker quadrilateral (XZ plane, zoomed)")
plt.grid(True)

# 等倍固定
ax = plt.gca()
ax.set_aspect("equal", adjustable="box")

# Z をズーム
Z_MIN, Z_MAX = 50, 100
ax.set_ylim(Z_MIN, Z_MAX)

# Z幅に合わせて X もズーム
z_range = Z_MAX - Z_MIN
x_center = (df.iloc[:, 27].min() + df.iloc[:, 27].max()) / 2
ax.set_xlim(x_center - z_range / 2,
            x_center + z_range / 2)


plt.savefig(OUTPUT_PATH, dpi=300)
plt.close()

print(f"完了：{len(df)} フレーム中 {skipped} フレームをスキップ")
print(f"出力画像: {OUTPUT_PATH}")
