import pandas as pd
import matplotlib.pyplot as plt
import japanize_matplotlib # 日本語ラベルを使用するためにインポート

# --- 設定項目 ---
# ★★★ この行でCSVファイルのパスを直接指定します ★★★
# 例 (Windows): r'C:\Users\YourUser\Documents\simulation_history.csv'
# 例 (Mac): '/Users/YourUser/Documents/simulation_history.csv'
CSV_FILE_PATH = r'C:\Users\rsait\OneDrive\ドキュメント\git\M1_styudy\result\2025\08\19\042005\simulation_history.csv'

# 形状を表示したいステップ番号を指定
STEP_TO_PLOT = 1 # この数値を変更して、見たいステップを指定


def plot_manipulator_shape(df: pd.DataFrame, step_number: int):
    """
    指定されたステップ番号のマニピュレータ形状をプロットする関数
    """
    # ... (この関数の中身は変更ありません) ...
    # 指定されたステップのデータのみを抽出
    arm_data = df[df['dis_range_step'] == step_number]

    if arm_data.empty:
        print(f"エラー: ステップ {step_number} のデータが見つかりません。")
        return

    # X座標とY座標を取得
    x_coords = arm_data['Position X']
    y_coords = arm_data['Position Y']

    # グラフの作成
    plt.figure(figsize=(8, 8))
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', label=f'アーム形状 (ステップ {step_number})')
    
    # 関節番号をプロット上に表示
    for i, joint_num in enumerate(arm_data['Joint']):
        plt.text(x_coords.iloc[i], y_coords.iloc[i] + 0.5, str(joint_num), ha='center')

    # グラフの装飾
    plt.title(f'マニピュレータの形状 (dis_range_step = {step_number})')
    plt.xlabel('X座標')
    plt.ylabel('Y座標')
    plt.grid(True)
    plt.axis('equal')  # X軸とY軸のスケールを合わせる
    plt.legend()
    plt.show()


def plot_joint_angles(df: pd.DataFrame):
    """
    全関節の角度の変動をプロットする関数
    """
    # ... (この関数の中身は変更ありません) ...
    # グラフの作成
    plt.figure(figsize=(12, 7))

    # 関節ごとにループして角度の変動をプロット
    # df['Joint'].unique() で存在する関節番号のリストを取得
    for joint_id in sorted(df['Joint'].unique()):
        joint_data = df[df['Joint'] == joint_id]
        plt.plot(joint_data['dis_range_step'], joint_data['Angle'], label=f'関節 {joint_id}')

    # グラフの装飾
    plt.title('各関節の角度の変動')
    plt.xlabel('シミュレーションステップ (dis_range_step)')
    plt.ylabel('角度 (°)')
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left') # 凡例をグラフの右側に表示
    plt.tight_layout() # レイアウトを自動調整
    plt.show()


def main():
    """
    メイン処理
    """
    try:
        # CSVファイルを指定されたパスから読み込む
        data = pd.read_csv(CSV_FILE_PATH)
        print(f"'{CSV_FILE_PATH}' を正常に読み込みました。")
    except FileNotFoundError:
        print(f"エラー: '{CSV_FILE_PATH}' が見つかりません。パスが正しいか確認してください。")
        return

    # 1. 指定したステップのマニピュレータ形状を表示
    plot_manipulator_shape(data, STEP_TO_PLOT)

    # # 2. 関節角度の変動グラフを表示
    # plot_joint_angles(data)


if __name__ == '__main__':
    main()