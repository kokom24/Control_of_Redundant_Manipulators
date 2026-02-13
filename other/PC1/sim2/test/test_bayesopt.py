#ひとまず完成用
#完成！！

import numpy as np
import matplotlib.pyplot as plt
import math
import time
import matplotlib.patches as pat
from matplotlib import cm
import os
from datetime import datetime
from tqdm import tqdm
# import optuna

#キー入力でグラフを閉じる
def on_key(event):
    if event.key == ' ':
        plt.close()
        
#手先位置の更新
def new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, end_eff_x, end_eff_y,n):
    dt = 0.05
    new_end_eff_x = end_eff_x + (gole_position[0] - first_pos_x[n]) * (dt / 5)
    new_end_eff_y = end_eff_y + (gole_position[1] - first_pos_y[n]) * (dt / 5)
    return new_end_eff_x, new_end_eff_y

#nリンクの時の連立微分方程式の右辺
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
        KD = L *2
    else:
        KD = 0
    

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

#nリンクの時のルンゲ・クッタ法
def runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,end_eff_x,end_eff_y, judge, t, fail_count, angle, count):
    #print("x_vec:",x_vec)
    k1, dL, angle,fail_count= f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t,fail_count, angle,count)
    #print("k1:",k1)
    k2, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k1,end_eff_x,end_eff_y,judge,t,fail_count, angle,  count)
    #print("k2:",k2)
    k3, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k2,end_eff_x,end_eff_y,judge,t,fail_count, angle,  count)
    #print("k3:",k3)
    k4, dL , angle,fail_count = f_n(L, K, D, n, theta_0, x_vec + dt * k3,end_eff_x,end_eff_y,judge ,t,fail_count,   angle,  count)
    # #print("k4:",k4)
    return x_vec + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4) , dL, angle, fail_count

#nリンクの時のグラフの更新
def new_posi_spring_n(tmax, dt, t, x_vec, theta_0, gole_position, n, L, K, D, initial_position, first_pos_x, first_pos_y, spring_len_judg, ran, theta_n, Kp, Ki, Kd):
    # グラフの準備
    fig, ax = plt.subplots()
    plt_x = np.zeros(n + 1)
    plt_y = np.zeros(n + 1)
    plt_x[0] = initial_position[0]
    plt_y[0] = initial_position[1]
    
    # 初期角度を設定
    angle = np.zeros(n)
    angle[0] = np.deg2rad(theta_0)
    theta = theta_n / n
    for i in range(n - 1):
        angle[i + 1] = angle[i] + np.deg2rad(-theta)
    
    # 初期リンク位置の計算
    for i in range(n):
        plt_x[i + 1] = plt_x[i] + L * math.cos(angle[i])
        plt_y[i + 1] = plt_y[i] + L * math.sin(angle[i])
    
    # 初期描画
    for i in range(n):
        ax.plot([plt_x[i], plt_x[i + 1]], [plt_y[i], plt_y[i + 1]], color="black", marker="o", linestyle="-")
        plt.gca().set_aspect('equal', adjustable='box')
    ax.plot(gole_position[0], gole_position[1], color="red", marker="o")  # ゴール描画
    ax.set_xlim(-L * (n * 0.1), L * (n * 1.1))
    ax.set_ylim(-L * (n * 0.1), L * (n * 1.1))
    ax.grid(True)
    plt.text(-L * (n * 0.1), L * (n * 1.1) + 2, "start", fontsize=20, color="red")
    
    # 初期化
    gole_x = gole_position[0]
    gole_y = gole_position[1]
    count = 0
    judge = 1
    fail_count = 1

    # --- PID制御の初期化 --- #
    # Kp, Ki, Kd are now passed as parameters
    error_integral = 0.0
    last_error = 0.0

    # シミュレーションループ
    while t < tmax:
        t += dt
        count += 1

        # --- PID制御によるKとDの更新 --- #
        error = np.linalg.norm([gole_position[0] - plt_x[n], gole_position[1] - plt_y[n]])
        error_integral += error * dt
        error_derivative = (error - last_error) / dt
        last_error = error

        K_pid = Kp * error + Ki * error_integral + Kd * error_derivative
        K = max(10.0, K_pid)
        D = max(1.0, 0.5 * np.sqrt(K))

        # リンク位置と角度更新
        x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt, plt_x, plt_y, judge, t, fail_count, angle, count)

        x = x_vec[0:n - 1]
        y = x_vec[(n - 1):2 * (n - 1)]

        if judge == 1:
            end_eff_x, end_eff_y = new_pos_eff(dt, first_pos_x, first_pos_y, gole_position, plt_x[n], plt_y[n], n)
            plt.text(-L * (n * 0.1), L * (n * 1.1) + 2, "judg", fontsize=20, color="red")
        elif judge == 0:
            pass

        if fail_count == 1:
            plt.text(L * (n * 0.7), L * (n * 1.1) + 2, "angle_fixed", fontsize=18, color="blue")

        plt_x = np.concatenate([np.array([initial_position[0]]), x, np.array([end_eff_x])])
        plt_y = np.concatenate([np.array([initial_position[1]]), y, np.array([end_eff_y])])

        for i in range(n):
            if np.all(plt_x[i] - ran <= plt_x[i + 1]) and np.all(plt_x[i + 1] <= plt_x[i] + ran) and np.all(plt_y[i] - ran <= plt_y[i + 1]) and np.all(plt_y[i + 1] <= plt_y[i] + ran):
                print("点が重なりました。")
                return "cross"

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
      


#初期条件の計算
def first_position(n , L ,initial_position, theta_0, theta_n):
    #n:リンク数
    #L:リンクの長さ
    #initial_angle:初期角度
    #initial_position:初期位置
    #初期位置を設定
    x = np.zeros(n+1)
    y = np.zeros(n+1)
    x[0] = initial_position[0]
    y[0] = initial_position[1]
    #初期角度を設定
    angle = np.zeros(n)
    angle[0] = np.deg2rad(theta_0)
    theta = theta_n / n
    #リンクの角度を計算
    for i in range(n-1):
        angle[i+1] = angle[i] + np.deg2rad(-theta)
    #リンクの位置を計算
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



def run():
    # パラメータ設定
    n = 10 # リンク数
    L = 10 # リンクの長さ
    ran = 0.1 # 目標位置の許容範囲
    theta_0 = 80 # 初期角度
    theta_n = 60 # 目標角度
    initial_position = [0, 0] # 初期位置
    spring_len_judge = 0.023 # ばねの収束判定範囲
    success = 0 # 成功判定
    test = 0 # テスト回数
    all_success = 0 # 成功数

    # 物理定数
    m_pa = 1.0
    k_pa = 100
    D_pa = 10

    # 時間変数
    tmin = 0.0
    tmax = 1000.0
    dt = 0.05
    t = tmin

    # ファイルパス
    file_path = os.path.join(os.getcwd(), "result.txt")

    # 初期化
    success_positions = []  # 成功した座標のリスト
    failure_positions = []  # 失敗した座標のリスト
    timeover_positions = []  # タイムオーバーした座標のリスト
    cross_positions = []  # 点が重なった座標のリスト
    loop = 1000  # テスト回数

    # 成功率の計算と座標の取得
    all_success = 0  # 成功数
    print("theta_0:", theta_0, "theta_n:", theta_n)

    # tqdmで進捗バーを作成
    for i in tqdm(range(loop), desc="Simulation Progress"):
        test += 1
        first_pos_x, first_pos_y = first_position(n, L, initial_position, theta_0, theta_n)
        mov_x, mov_y = move_posi(first_pos_x, first_pos_y, n)
        x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(n-1), np.zeros(n-1)])

        gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]
        while gole_position == initial_position or np.sqrt((gole_position[0] - initial_position[0])**2 + (gole_position[1] - initial_position[1])**2) > L * n:
            gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]

        # PID初期値（任意で調整可能）
        Kp = 2.5
        Ki = 0.02
        Kd = 1.0

        success = new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, gole_position, n, L, k_pa, D_pa,
                                    initial_position, first_pos_x, first_pos_y, spring_len_judge, ran, theta_n,
                                    Kp, Ki, Kd)

        if success == 1:
            all_success += 1
            success_positions.append(gole_position)
        elif success in [0, None, "cross"]:
            failure_positions.append(gole_position)

    success_rate = all_success / test
    print("success_rate:", success_rate * 100, "%")

    success_positions = np.array(success_positions)
    failure_positions = np.array(failure_positions)
    timeover_positions = np.array(timeover_positions)
    cross_positions = np.array(cross_positions)

    fig, ax = plt.subplots()
    ax.set_xlim(-L * (n * 0.1), L * (n * 1.1))
    ax.set_ylim(-L * (n * 0.1), L * (n * 1.1))

    plt_x = np.zeros(n+1)
    plt_y = np.zeros(n+1)
    plt_x[0] = initial_position[0]
    plt_y[0] = initial_position[1]

    angle = np.zeros(n)
    angle[0] = np.deg2rad(theta_0)
    theta = theta_n / n

    for i in range(n-1):
        angle[i+1] = angle[i] + np.deg2rad(-theta)
    for i in range(n):
        plt_x[i+1] = plt_x[i] + L * math.cos(angle[i])
        plt_y[i+1] = plt_y[i] + L * math.sin(angle[i])
    for i in range(n):
        ax.plot([plt_x[i], plt_x[i+1]], [plt_y[i], plt_y[i+1]], color="black", marker="o", linestyle="-", linewidth=2)
    plt.scatter([pos[0] for pos in success_positions], [pos[1] for pos in success_positions], color='blue', label='Success', marker="x")
    plt.scatter([pos[0] for pos in failure_positions], [pos[1] for pos in failure_positions], color='red', label='Failure', marker="x")
    plt.scatter([pos[0] for pos in timeover_positions], [pos[1] for pos in timeover_positions], color='green', label='Timeover', marker="x")
    plt.scatter([pos[0] for pos in cross_positions], [pos[1] for pos in cross_positions], color='yellow', label='Cross', marker="x")
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Success and Failure Positions')
    fig_path = os.path.join(os.getcwd(), "result.png")
    plt.savefig(fig_path)

    para_to_write = f"n: {n}\nL: {L}\nloop: {loop}\n"
    data_to_write = f"theta_0: {theta_0}\ntheta_n: {theta_n}\nsuccess_rate: {success_rate * 100}%\n"

    with open(file_path, 'a') as file:
        file.write(para_to_write)
        file.write(data_to_write)

    print("-----------------------------finish-----------------------------")
    plt.clf()
            

if __name__ == '__main__':
    # Set file path based on the current date and time
    now = datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    day = now.strftime("%d")
    time_str = now.strftime("%H%M%S")

    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../../result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)

    # Set broken joint
    broken_joint = [2,3,4,5,6]

    #write file
    porpose_write = f"{len(broken_joint)} joints are broken.\n {broken_joint} is fixed\n"
    date_time_str = now.strftime("%Y/%m/%d %H:%M:%S")
    date_write = f"date: {date_time_str}\n"
    with open("result.txt", mode='w') as f:
        f.write(porpose_write)
        f.write(date_write)

    # Run simulation
    run()


from bayes_opt import BayesianOptimization

def optimize_pid(Kp, Ki, Kd):
    try:
        n = 10
        L = 10
        ran = 0.1
        theta_0 = 80
        theta_n = 60
        initial_position = [0, 0]
        spring_len_judge = 0.023
        tmin = 0.0
        tmax = 100.0
        dt = 0.05
        t = tmin
        k_pa = 100
        D_pa = 10

        gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]
        while gole_position == initial_position or np.linalg.norm(np.array(gole_position) - np.array(initial_position)) > L * n:
            gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]

        first_pos_x, first_pos_y = first_position(n, L, initial_position, theta_0, theta_n)
        mov_x, mov_y = move_posi(first_pos_x, first_pos_y, n)
        x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(n-1), np.zeros(n-1)])

        result = new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, gole_position, n, L, k_pa, D_pa,
                                   initial_position, first_pos_x, first_pos_y, spring_len_judge, ran,
                                   theta_n, Kp, Ki, Kd)

        if result == 1:
            return 1.0
        elif result == "cross":
            return 0.2
        else:
            return 0.0
    except Exception as e:
        print("Error in optimize_pid:", e)
        return 0.0


def optimize_pid_with_bayesopt():
    pbounds = {
        'Kp': (0.1, 10.0),
        'Ki': (0.0, 2.0),
        'Kd': (0.1, 5.0),
    }

    optimizer = BayesianOptimization(
        f=optimize_pid,
        pbounds=pbounds,
        random_state=42,
        verbose=2
    )

    optimizer.maximize(init_points=5, n_iter=15)

    print("最適パラメータ：", optimizer.max)


if __name__ == '__main__':
    optimize_pid_with_bayesopt()
