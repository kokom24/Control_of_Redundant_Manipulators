import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import detrend, butter, filtfilt, get_window, find_peaks

# === 設定 ===
csv_path = "optitrack.csv"
fs = 120.0
velocity_threshold = 0.005 # m/s単位に合わせて少し調整
highcut = 10.0

plt.rcParams["font.size"] = 14

def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# === データ読み込み ===
df = pd.read_csv(csv_path)
x = pd.to_numeric(df.iloc[:, 5], errors="coerce").values
z = pd.to_numeric(df.iloc[:, 7], errors="coerce").values

# NaN 除去
valid = ~np.isnan(x) & ~np.isnan(z)
x, z = x[valid], z[valid]

# 1. 座標のフィルタリング
x_filt = butter_lowpass_filter(x, highcut, fs)
z_filt = butter_lowpass_filter(z, highcut, fs)

# 合成位置 (mm) -> (m) へ変換
r_m = np.sqrt(x_filt**2 + z_filt**2) / 1000.0

# 速度計算 (m/s)
v_m_s = np.gradient(r_m, 1/fs)
v_det = detrend(v_m_s)

# --- 解析区間の抽出 ---
moving = np.abs(v_det) > velocity_threshold
start = np.argmax(moving)
end = len(moving) - np.argmax(moving[::-1])
v_valid = v_det[start:end]

# --- 2. 減衰比 (zeta) の計算 ---
# 速度の最大値（最後の動き）からピークを探す
max_idx_in_v = np.argmax(v_valid)
peaks, _ = find_peaks(v_valid[max_idx_in_v:], distance=fs*0.2, height=velocity_threshold)

zeta = None
if len(peaks) >= 2:
    A1 = v_valid[max_idx_in_v + peaks[0]]
    A2 = v_valid[max_idx_in_v + peaks[1]]
    delta = np.log(A1 / A2)
    zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)

# --- 3. FFT (ピーク周波数) の計算 ---
N = len(v_valid)
v_windowed = v_valid * get_window('hann', N)
fft_vals = np.fft.rfft(v_windowed)
freqs = np.fft.rfftfreq(N, 1/fs)
amp = np.abs(fft_vals) * (2.0 / N) * 2.0 
peak_freq = freqs[np.argmax(amp[1:]) + 1]

# --- 4. 結果のプリント ---
print("-" * 30)
print(f"最大速度: {np.max(np.abs(v_det)):.4f} m/s")
print(f"ピーク周波数: {peak_freq:.3f} Hz")
if zeta is not None:
    print(f"減衰比 (ζ): {zeta:.4f}")
else:
    print("減衰比: 十分なピークが検出されませんでした")
print("-" * 30)

# --- 5. グラフ化 ---
fig, ax = plt.subplots(2, 1, figsize=(12, 10))

# 上段：時間波形
time_full = np.arange(len(v_det)) / fs
ax[0].plot(time_full, v_det, color='tab:blue', label="Velocity [m/s]")
ax[0].axhline(y=0, color='black', linestyle='-', alpha=0.3)

# --- ここでY軸の上下幅を設定 ---
v_max = np.max(np.abs(v_det))
ax[0].set_ylim(-v_max * 1.5, v_max * 1.5) # 最大値の1.5倍の幅を上下に持たせる

ax[0].set_title(f"Velocity Time Series (ζ = {zeta:.4f} if detected)")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("Velocity [m/s]")
ax[0].grid(True, linestyle='--')

# 下段：FFT
ax[1].plot(freqs[1:], amp[1:], color='tab:orange')
ax[1].axvline(peak_freq, color='red', linestyle=':', label=f"Peak: {peak_freq:.2f}Hz")

# --- 下段も少し上に余裕を持たせる ---
ax[1].set_ylim(0, np.max(amp[1:]) * 1.3) 

ax[1].set_xlim(0, 10)
ax[1].set_title("Frequency Analysis (FFT)")
ax[1].set_xlabel("Frequency [Hz]")
ax[1].set_ylabel("Amplitude (m/s)")
ax[1].legend()
ax[1].grid(True, linestyle='--')

plt.tight_layout() # レイアウトの自動調整
#画像を保存
plt.savefig("optitrack_analysis.png", dpi=300) # dpiを指定すると発表資料で綺麗になります

plt.show()