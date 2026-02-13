import numpy as np
import matplotlib.pyplot as plt
import math
import time
import os
from datetime import datetime
from tqdm import tqdm
import optuna
from multiprocessing import Lock, cpu_count

# グローバルロック（ログ書き込み用）
log_lock = Lock()

def new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, end_eff_x, end_eff_y,n):
    dt = 0.05
    new_end_eff_x = end_eff_x + (gole_position[0] - first_pos_x[n]) * (dt / 5)
    new_end_eff_y = end_eff_y + (gole_position[1] - first_pos_y[n]) * (dt / 5)
    return new_end_eff_x, new_end_eff_y

def f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t, fail_count, angle,count):
    fail_time = 5

    re = np.zeros(4 *(n-1)) 
    x = x_vec[0:n-1]
    y = x_vec[n-1:2*(n-1)]
    vx = np.concatenate([np.zeros(1), x_vec[2*(n-1):3*(n-1)],np.zeros(1)])
    vy = np.concatenate([np.zeros(1), x_vec[3*(n-1):4*(n-1)],np.zeros(1)])
    #print("x:",x)
    #print("y:",y)
    #print("vx:",vx)
    #print("vy:",vy)
    # time.sleep(100)
    
    #Lの長さ
    dL = np.zeros(n)
    for i in range(n):
        dL[i] = np.sqrt((end_eff_x[i+1]-end_eff_x[i])**2 + (end_eff_y[i+1]-end_eff_y[i])**2) - L
    # #print("dL:",dL)
        
    #関節の故障を計算   
    if judge == 1 and t > fail_time:
        fail_angle = np.zeros(5)
        fail_angle[0] = angle[1] *0.9
        fail_angle[1] = angle[2]*0.9
        fail_angle[2] = angle[3]*0.9
        fail_angle[3] = angle[4]*0.9
        fail_angle[4] = angle[5]*0.9
        fail_count = 1
    else:
        fail_angle = np.zeros(5)

    if fail_count == 1:
        fail_angle[0] = angle[1]*0.9
        fail_angle[1] = angle[2]*0.9
        fail_angle[2] = angle[3]*0.9
        fail_angle[3] = angle[4]*0.9
        fail_angle[4] = angle[5]*0.9
        #print("fail_angle:",fail_angle)
        # time.sleep(100)

            
    #関節角度の計算算
    angle = np.zeros(n)
    sin_angle = np.zeros(n)
    cos_angle = np.zeros(n)
    for i in range(n):  
        # #print("i:",i)
        # #print("end_eff_x:",end_eff_x[i+1])
        # #print("end_eff_x:",end_eff_x[i])
        # #print("end_eff_y:",end_eff_y[i+1])
        # #print("end_eff_y:",end_eff_y[i])
        if end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                angle[1] = fail_angle[0]
                angle[2]= fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            else:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = (np.pi) - math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                angle[1] = fail_angle[0]
                angle[2]= fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = -1 * math.cos(angle[i])
            else:
                angle[i] = (np.pi) - math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = -1*math.cos(angle[i])
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            else:
                angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            # angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
            # sin_angle[i] = -1*math.sin(angle[i])
            # cos_angle[i] = math.cos(angle[i])
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i])) + np.pi
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = -1 *math.cos(angle[i])
            else:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i])) + np.pi
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = -1* math.cos(angle[i])
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = 1
                cos_angle[i] = 0
            else:
                angle[i] = 0
                sin_angle[i] = 1 
                cos_angle[i] = 0
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                sin_angle[i] = -1
                cos_angle[i] = 0
            else:
                angle[i] = 0
                sin_angle[i] = -1
                cos_angle[i] = 0
        else:
            print("error")
            # time.sleep(100)

    #print("angle:",angle)
    #print("angle (degrees):", np.rad2deg(angle))
    
    #関節角度の制限
    P_R = np.zeros(n-1)
    dL_data = np.zeros(n-1)
    if count % 50 == 0:
        KKD = L *2
    else:
        KKD = 0
    

    #Fの計算
    DD = L
    ax = np.zeros(n-1)
    for i in range(n-1):
        ax[i] = K*(dL[i+1] * cos_angle[i+1]) - K *(dL[i] * cos_angle[i]) + D * (vx[i+2] - vx[i+1]) - D*(vx[i+1]-vx[i])

    ay = np.zeros(n-1)
    for i in range(n-1):
        ay[i] = K*(dL[i+1] * sin_angle[i+1]) - K *(dL[i] * sin_angle[i]) + D * (vy[i+2] - vy[i+1]) - D*(vy[i+1]-vy[i])
        #+ (0.001*np.sqrt(((1/(R[i]-DD) )** 2) ))

    #vxのデータの数をprint
    # print("len(vx):",len(vx))
    # print("len(ax):",len(ax))
    # print("len(dL):",len(dL))
    # time.sleep(100)

    #print("vx:",vx)
    #print("vy:",vy)
    #reの各座標に代入
    re[0:n-1] = vx[1:-1]
    re[(n-1):2*(n-1)] = vy[1:-1]
    re[2*(n-1):3*(n-1)] = ax
    re[3*(n-1):4*(n-1)] = ay
    # #print("vx:",vx)
    # #print("vy:",vy)
    #print("re:",re)
    return re, dL, angle, fail_count

def runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,end_eff_x,end_eff_y, judge, t, fail_count, angle, count):
    k1, dL, angle,fail_count = f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t,fail_count, angle,count)
    k2, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k1,end_eff_x,end_eff_y,judge,t,fail_count, angle, count)
    k3, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k2,end_eff_x,end_eff_y,judge,t,fail_count, angle, count)
    k4, dL , angle,fail_count = f_n(L, K, D, n, theta_0, x_vec + dt * k3,end_eff_x,end_eff_y,judge ,t,fail_count, angle, count)
    return x_vec + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4) , dL, angle, fail_count

def first_position(n , L ,initial_position, theta_0, theta_n):
    x = np.zeros(n+1)
    y = np.zeros(n+1)
    x[0] = initial_position[0]
    y[0] = initial_position[1]
    angle = np.zeros(n)
    angle[0] = np.deg2rad(theta_0)
    theta = theta_n / n
    for i in range(n-1):
        angle[i+1] = angle[i] + np.deg2rad(-theta)
    for i in range(n):
        x[i+1] = x[i] + L * math.cos(angle[i])
        y[i+1] = y[i] + L * math.sin(angle[i])
    return x,y

def move_posi(x,y,n):
    mov_x = np.zeros(n-1)
    mov_y = np.zeros(n-1)
    for i in range(n-1):
        mov_x[i] = x[i+1]
        mov_y[i] = y[i+1]
    return mov_x, mov_y

def new_posi_spring_n(tmax, dt, t, x_vec, theta_0, gole_position, n, L, initial_position, first_pos_x, first_pos_y, spring_len_judg, ran, theta_n, Kp, Ki, Kd):
    plt_x = np.zeros(n + 1)
    plt_y = np.zeros(n + 1)
    plt_x[0] = initial_position[0]  # 初期位置のx座標
    plt_y[0] = initial_position[1]  # 初期位置のy座標
    angle = np.zeros(n)
    angle[0] = np.deg2rad(theta_0)
    theta = theta_n / n
    for i in range(n - 1):
        angle[i + 1] = angle[i] + np.deg2rad(-theta)
    for i in range(n):
        plt_x[i + 1] = plt_x[i] + L * math.cos(angle[i])
        plt_y[i + 1] = plt_y[i] + L * math.sin(angle[i])
    count = 0
    judge = 1
    fail_count = 1
    error_integral = 0.0
    last_error = 0.0
    while t < tmax:
        t += dt
        count += 1
        error = np.linalg.norm([gole_position[0] - plt_x[n], gole_position[1] - plt_y[n]])
        error_integral += error * dt
        error_derivative = (error - last_error) / dt
        last_error = error
        K = max(10.0, Kp * error + Ki * error_integral + Kd * error_derivative)
        D = max(1.0, 0.1 * np.sqrt(K))
        x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt, plt_x, plt_y, judge, t, fail_count, angle, count)
        x = x_vec[0:n - 1]
        y = x_vec[(n - 1):2 * (n - 1)]
        if judge == 1:
            end_eff_x, end_eff_y = new_pos_eff(dt, first_pos_x, first_pos_y, gole_position, plt_x[n], plt_y[n], n)
        plt_x = np.concatenate([np.array([initial_position[0]]), x, np.array([end_eff_x])])
        plt_y = np.concatenate([np.array([initial_position[1]]), y, np.array([end_eff_y])])
        max_dL = np.max(np.abs(dL))
        if count > 1:
            if max_dL < spring_len_judg:
                judge = 1
                if np.all(gole_position[0] - ran <= plt_x[n]) and np.all(plt_x[n] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= plt_y[n]) and np.all(plt_y[n] <= gole_position[1] + ran):
                    return 1
            elif max_dL >= L * 0.4:
                return 0
            else:
                judge = 0
    return 0

def run(Kp, Ki, Kd):
    # ロボット設定
    n = 10                     # 関節数
    L = 10                     # 各リンクの長さ
    ran = 0.1                  # ゴールの判定範囲
    theta_0 = 80               # 最初の角度（deg）
    theta_n = 60               # 総角度（deg）
    initial_position = [0.0, 0.0]  # アームの基点位置
    spring_len_judg = 0.023    # ばねの長さ変化の閾値

    # シミュレーション設定
    tmin = 0.0
    tmax = 500.0
    dt = 0.05
    t = tmin
    loop = 500               # 試行回数
    all_success = 0

    for _ in tqdm(range(loop), desc="Loop Progress"):
        # 初期配置を計算
        first_pos_x, first_pos_y = first_position(n, L, initial_position, theta_0, theta_n)
        mov_x, mov_y = move_posi(first_pos_x, first_pos_y, n)
        
        # 状態ベクトルの初期化（x, y, vx, vy）
        x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(n-1), np.zeros(n-1)])

        # ゴール位置をランダムに設定（初期位置と遠すぎる位置を除外）
        gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]
        while gole_position == initial_position or np.linalg.norm(np.array(gole_position) - np.array(initial_position)) > L * n:
            gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]

        # シミュレーション実行（成功すれば 1 が返る）
        success = new_posi_spring_n(
            tmax, dt, t, x_vec_n, theta_0, gole_position, n, L,
            initial_position, first_pos_x, first_pos_y,
            spring_len_judg, ran, theta_n,
            Kp, Ki, Kd  # ←★順番修正済み！
        )

        if success == 1:
            all_success += 1

    # 成功率（0〜1）
    return all_success / loop
def objective(trial):
    Kp = trial.suggest_float("Kp", 4.5, 10.0)
    Ki = trial.suggest_float("Ki", 0.08, 2.0)
    Kd = trial.suggest_float("Kd", 3.0, 10.0)

    print(f"Trial with Kp={Kp:.3f}, Ki={Ki:.4f}, Kd={Kd:.3f}")
    success_rate = run(Kp, Ki, Kd)
    print(f"Success rate: {success_rate * 100:.2f}%")

    # ログを排他制御付きで出力
    with log_lock:
        with open("result_log.txt", "a") as f:
            f.write(f"Kp={Kp:.3f}, Ki={Ki:.4f}, Kd={Kd:.3f} -> Success rate: {success_rate * 100:.2f}%\n")

    return success_rate


def main():
    # 📁 保存ディレクトリ作成（日時ベース）
    now = datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    day = now.strftime("%d")
    time_str = now.strftime("%H%M%S")

    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../../result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)

    # 🧠 Optunaで最適化（並列数 = コア数 - 2）
    study = optuna.create_study(direction="maximize")
    n_jobs = max(cpu_count() - 2, 1)
    study.optimize(objective, n_trials=200, n_jobs=n_jobs)

    # 💾 最適結果を保存
    with open("best_result.txt", "w") as f:
        f.write("Best parameters:\n")
        for key, value in study.best_params.items():
            f.write(f"{key}: {value}\n")
        f.write(f"Best success rate: {study.best_value * 100:.2f}%\n")

    print("Best parameters:")
    print(study.best_params)
    print(f"Best success rate: {study.best_value * 100:.2f}%")

if __name__ == "__main__":
    print("Starting optimization...")
    time.sleep(1)
    main()