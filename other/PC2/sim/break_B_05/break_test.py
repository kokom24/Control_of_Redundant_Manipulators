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
from multiprocessing import Pool

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
def f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t, fail_count, angle,count, broken_joint ):
    fail_time = 5

    re = np.zeros(4 *(n-1)) 
    x = x_vec[0:n-1]
    y = x_vec[n-1:2*(n-1)]
    vx = np.concatenate([np.zeros(1), x_vec[2*(n-1):3*(n-1)],np.zeros(1)])
    vy = np.concatenate([np.zeros(1), x_vec[3*(n-1):4*(n-1)],np.zeros(1)])
    
    #Lの長さ
    dL = np.zeros(n)
    for i in range(n):
        dL[i] = np.sqrt((end_eff_x[i+1]-end_eff_x[i])**2 + (end_eff_y[i+1]-end_eff_y[i])**2) - L
        
    #関節の故障を計算   
    if judge == 1 and t > fail_time:
        fail_angle = np.zeros(len(broken_joint))
        for i in range(len(broken_joint)):
            fail_angle[i] = angle[broken_joint[i]]
        fail_count = 1
    else:
        fail_angle = [0] * len(broken_joint)

    if fail_count == 1:
        for i in range(len(broken_joint)):
            angle[broken_joint[i]] = fail_angle[i]

            
    #関節角度の計算算
    angle = np.zeros(n)
    sin_angle = np.zeros(n)
    cos_angle = np.zeros(n)
    for i in range(n):  
        if end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            else:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = (np.pi) - math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = -1 * math.cos(angle[i])
            else:
                angle[i] = (np.pi) - math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = -1*math.cos(angle[i])
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            else:
                angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i])) + np.pi
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = -1 *math.cos(angle[i])
            else:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i])) + np.pi
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = -1* math.cos(angle[i])
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = 1
                cos_angle[i] = 0
            else:
                angle[i] = 0
                sin_angle[i] = 1 
                cos_angle[i] = 0
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                for j in range(len(broken_joint)):
                    angle[broken_joint[j]] = fail_angle[j]
                sin_angle[i] = -1
                cos_angle[i] = 0
            else:
                angle[i] = 0
                sin_angle[i] = -1
                cos_angle[i] = 0
        else:
            print("error")
            break

    #関節角度の制限
    P_R = np.zeros(n-1)
    dL_data = np.zeros(n-1)
    if count % 50 == 0:
        KD = L *2
    else:
        KD = 0

    for i in range(n-1):
        dL_data = np.sqrt((end_eff_x[i+2]-end_eff_x[i])**2 + (end_eff_y[i+2]-end_eff_y[i])**2)
        P_R[i] =  KD *(1/dL_data)
    

    #Fの計算
    ax = np.zeros(n-1)
    ay = np.zeros(n-1)
    for i in range(n-1):
        ax[i] = K*(dL[i+1] * cos_angle[i+1]) - K *(dL[i] * cos_angle[i]) + D * (vx[i+2] - vx[i+1]) - D*(vx[i+1]-vx[i]) - P_R[i] * (cos_angle[i] ) + P_R[i] * (cos_angle[i+1] )
    for i in range(n-1):
        ay[i] = K*(dL[i+1] * sin_angle[i+1]) - K *(dL[i] * sin_angle[i]) + D * (vy[i+2] - vy[i+1]) - D*(vy[i+1]-vy[i]) - P_R[i] * (sin_angle[i] ) + P_R[i] * (sin_angle[i+1] )

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
def runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,end_eff_x,end_eff_y, judge, t, fail_count, angle, count, broken_joint):
    #print("x_vec:",x_vec)
    k1, dL, angle,fail_count= f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t,fail_count, angle,count, broken_joint)
    #print("k1:",k1)
    k2, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k1,end_eff_x,end_eff_y,judge,t,fail_count, angle,  count, broken_joint)
    #print("k2:",k2)
    k3, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k2,end_eff_x,end_eff_y,judge,t,fail_count, angle,  count, broken_joint)
    #print("k3:",k3)
    k4, dL , angle,fail_count = f_n(L, K, D, n, theta_0, x_vec + dt * k3,end_eff_x,end_eff_y,judge ,t,fail_count,   angle,  count, broken_joint)
    # #print("k4:",k4)
    return x_vec + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4) , dL, angle, fail_count

#nリンクの時のグラフの更新
def new_posi_spring_n(tmax, dt, t, x_vec, theta_0, gole_position,n, L, K, D, initial_position,first_pos_x,first_pos_y, spring_len_judg,ran,theta_n,broken_joint):
    # グラフの準備
    fig, ax = plt.subplots()
    plt_x = np.zeros(n+1)
    plt_y = np.zeros(n+1)
    plt_x[0] = initial_position[0]
    plt_y[0] = initial_position[1]
    #初期角度を設定
    angle = np.zeros(n)
    angle[0] = np.deg2rad(theta_0)
    theta = theta_n / n
    #リンクの角度を計算
    for i in range(n-1):
        angle[i+1] = angle[i] + np.deg2rad(-theta)
    #リンクの位置を計算
    for i in range(n):
        plt_x[i+1] = plt_x[i] + L * math.cos(angle[i])
        plt_y[i+1] = plt_y[i] + L * math.sin(angle[i])
    #リンクの描画
    for i in range(n):
        ax.plot([plt_x[i],plt_x[i+1]],[plt_y[i],plt_y[i+1]],color="black",marker="o",linestyle="-")
        plt.gca().set_aspect('equal', adjustable='box')
    #目標位置の描画
    ax.plot(gole_position[0],gole_position[1],color="red",marker="o")
    #グラフの設定
    ax.set_xlim(-L *(n * 0.1),L *(n * 1.1))  # x軸の範囲を設定
    ax.set_ylim(-L *(n * 0.1),L *(n * 1.1))  # y軸の範囲を設定
    ax.grid(True)  # グリッドを表示
    plt.connect('key_press_event', on_key)
    #グラフの左上に”spoe”と文字を表示
    plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "start", fontsize=20, color="red")
    # #print("plt_x:",plt_x)
    # #print("plt_y:",plt_y)
    # plt.show()
    # time.sleep(100)
    
    #変数の初期化
    gole_x = gole_position[0]
    gole_y = gole_position[1]
    count = 0
    judge = 1
    fail_count = 1
    
    
    #シミュレーションの実行
    while t < tmax:
        #print("/////////////////////////////count:",count,"/////////////////////////////")
        # print("fail_count:",fail_count)
        # print("t:",t)
        t += dt
        count += 1
         
        # if count > 3:
        #     time.sleep(100)
         
        # グラフの初期化
        # ax.clear()
        # ax.set_xlim(-L *(n * 0.1),L *(n * 1.1))  # x軸の範囲を設定
        # ax.set_ylim(-L *(n * 0.1),L *(n * 1.1))  # y軸の範囲を設定
        #print("re_x_vec:",x_vec)
        #print("re_plt_x:",plt_x)
        #print("re_plt_y:",plt_y)

        #アニメーションの初期化
        x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,plt_x,plt_y, judge, t, fail_count,angle, count,broken_joint)
        # 各要素の更新
        x = x_vec[0:n-1]
        y = x_vec[(n-1):2*(n-1)]
        # v_x = x_vec[2*(n-1):3*(n-1)]
        # v_y = x_vec[3*(n-1):4*(n-1)]
        #print("x:",x)
        #print("y:",y)
        #print("v_x:",v_x)
        #print("v_y:",v_y)
        
        #print("plt_x[n]",plt_x[n])
        #print("plt_y[n]",plt_y[n])
        
        
        #手先位置の更新
        if judge == 1:
            end_eff_x,end_eff_y = new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, plt_x[n], plt_y[n],n)
            plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "judg", fontsize=20, color="red")
            # print("-----------------------------judge:-----------------------------")
            #print("end_eff_x:",end_eff_x)
            #print("end_eff_y:",end_eff_y)
        elif judge == 0:
            pass

        if fail_count == 1:
            plt.text(L *(n * 0.7), L *(n * 1.1)+2, "angle_fixed", fontsize=18, color="blue")
        
        
        # リンクの更新
        plt_x = np.concatenate([np.array([initial_position[0]]), x, np.array([end_eff_x])])
        plt_y = np.concatenate([np.array([initial_position[1]]), y, np.array([end_eff_y])])

        for i in range(n):
            if np.all(plt_x[i] - ran <= plt_x[i+1] ) and np.all(plt_x[i+1] <= plt_x[i] + ran) and np.all(plt_y[i] - ran <= plt_y[i+1]) and np.all(plt_y[i+1] <= plt_y[i] + ran):
                print("点が重なりました。")
                success = "cross"
                return success

        max_dL = np.max(np.abs(dL))
        # print("max_dL:",max_dL)
        if count > 1:  # count が 1 以上の場合に条件を満たすように変更
            # ばねの運動が収束したら、ループを抜ける
            if max_dL <spring_len_judg:
                #print("ばねの運動が収束しました。")
                # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                judge = 1
                        #ゴールの判定
                if np.all(gole_position[0] - ran <= plt_x[n]) and np.all(plt_x[n] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= plt_y[n]) and np.all(plt_y[n] <= gole_position[1] + ran):
                           # グラフの初期化
                    # ax.clear()
                    # plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "fin", fontsize=20, color="red")                    
                    # print("ゴールしました。")
                    success = 1
                    return success
                    
                    # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                    # break
                # break
            elif max_dL >= L * 0.4:
                # print("ばねの運動が収束していません。")
                success = 0
                return success
                # break
            else:
                judge = 0
            

    # print("time over")            


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



def run(broken_joint):
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
    for i in tqdm(range(loop), desc=f"broken_joint: {broken_joint}"):
        test += 1
        first_pos_x, first_pos_y = first_position(n, L, initial_position, theta_0, theta_n)
        mov_x, mov_y = move_posi(first_pos_x, first_pos_y, n)
        x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(n-1), np.zeros(n-1)])

        gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]
        while gole_position == initial_position or np.sqrt((gole_position[0] - initial_position[0])**2 + (gole_position[1] - initial_position[1])**2) > L * n:
            gole_position = [np.random.randint(0, 100), np.random.randint(0, 100)]

        success = new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, gole_position, n, L, k_pa, D_pa, initial_position, first_pos_x, first_pos_y, spring_len_judge, ran, theta_n,broken_joint)

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
    fig_path = os.path.join(os.getcwd(), f"{broken_joint}_result.png")
    plt.savefig(fig_path)

    para_to_write = f"n: {n}\nL: {L}\nloop: {loop}\n"
    data_to_write = f"theta_0: {theta_0}\ntheta_n: {theta_n}\nsuccess_rate: {success_rate * 100}%\n"

    with open(file_path, 'a') as file:
        file.write(para_to_write)
        file.write(data_to_write)

    print("-----------------------------finish-----------------------------")
    plt.clf()
    return all_success, test
            

def process_joint(params):
    indices = params
    broken_joint = tuple(sorted(indices))

    # Start simulation for the given joint
    success_count, try_count = run(broken_joint)
    
    # Calculate the success rate for the current broken_joint
    return success_count, try_count, broken_joint, indices


def main():
    # Set file path based on the current date and time
    now = datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    day = now.strftime("%d")
    time_str = now.strftime("%H%M%S")

    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)

    seen_combinations = set()
    adjacent_results = []
    adjacent_success_count = 0
    adjacent_try_count = 0
    three_adjacent_results = []
    three_adjacent_success_count = 0
    three_adjacent_try_count = 0
    four_adjacent_results = []
    four_adjacent_success_count = 0
    four_adjacent_try_count = 0
    five_adjacent_results = []
    five_adjacent_success_count = 0
    five_adjacent_try_count = 0
    all_success = 0
    all_try = 0

    # Prepare the list of parameters to process
    parameters = [(i, j, k, l, m) for i in range(10) for j in range(10) for k in range(10) for l in range(10) for m in range(10)
                  if len(set([i, j, k, l, m])) == 5]

    # Specify the number of processes
    num_processes = 5  # Adjust this number based on your machine's capabilities

    # Process parameters in batches to limit memory usage
    batch_size = 100  # Adjust based on your system's capabilities
    with Pool(processes=num_processes) as pool:
        for i in tqdm(range(0, len(parameters), batch_size)):
            batch = parameters[i:i+batch_size]
            results = pool.map(process_joint, batch)
            
            # Collect results and calculate the overall success rate
            for success_count, try_count, broken_joint, indices in results:
                if broken_joint in seen_combinations:
                    continue

                seen_combinations.add(broken_joint)

                porpose_write = f"{len(broken_joint)} angle break.\n angle {broken_joint} is fixed\n\n"
                with open("result.txt", mode='a') as f:
                    f.write(porpose_write)

                all_success += success_count
                all_try += try_count

                # Calculate the success rate
                success_rate = all_success / all_try
                print(f"broken_joint: {broken_joint}")
                print("success_rate:", success_rate * 100, "%")

                # Check for adjacent conditions
                if len(broken_joint) == 5:
                    adjacents = [abs(indices[x] - indices[x+1]) for x in range(4)]
                    if any(adj == 1 for adj in adjacents):
                        adjacent_success_count += success_count
                        adjacent_try_count += try_count
                        adjacent_success_rate = adjacent_success_count / adjacent_try_count if adjacent_try_count > 0 else 0
                        adjacent_results.append(f"broken_joint: {broken_joint}, success_rate: {adjacent_success_rate * 100}%\n")

                    if all(adj == 1 for adj in adjacents):
                        three_adjacent_success_count += success_count
                        three_adjacent_try_count += try_count
                        three_adjacent_success_rate = three_adjacent_success_count / three_adjacent_try_count if three_adjacent_try_count > 0 else 0
                        three_adjacent_results.append(f"broken_joint: {broken_joint}, success_rate: {three_adjacent_success_rate * 100}%\n")

                    if len(set(adjacents)) == 1 and adjacents[0] == 1:
                        four_adjacent_success_count += success_count
                        four_adjacent_try_count += try_count
                        four_adjacent_success_rate = four_adjacent_success_count / four_adjacent_try_count if four_adjacent_try_count > 0 else 0
                        four_adjacent_results.append(f"broken_joint: {broken_joint}, success_rate: {four_adjacent_success_rate * 100}%\n")

                    if len(set(adjacents)) == 1 and adjacents[0] == 1:
                        five_adjacent_success_count += success_count
                        five_adjacent_try_count += try_count
                        five_adjacent_success_rate = five_adjacent_success_count / five_adjacent_try_count if five_adjacent_try_count > 0 else 0
                        five_adjacent_results.append(f"broken_joint: {broken_joint}, success_rate: {five_adjacent_success_rate * 100}%\n")

    # Write the total success rate to the file
    success_rate_write = f"total success_rate: {success_rate * 100}%\n"
    with open("result.txt", mode='a') as f:
        f.write(success_rate_write)

    # Write the 2-adjacent results and success rate to a separate file
    adjacent_success_rate = adjacent_success_count / adjacent_try_count if adjacent_try_count > 0 else 0
    with open("adjacent_results.txt", mode='w') as f:
        f.writelines(adjacent_results)
        f.write(f"\nadjacent success_rate: {adjacent_success_rate * 100}%\n")

    # Write the 3-adjacent results and success rate to a separate file
    three_adjacent_success_rate = three_adjacent_success_count / three_adjacent_try_count if three_adjacent_try_count > 0 else 0
    with open("three_adjacent_results.txt", mode='w') as f:
        f.writelines(three_adjacent_results)
        f.write(f"\nthree_adjacent success_rate: {three_adjacent_success_rate * 100}%\n")

    # Write the 4-adjacent results and success rate to a separate file
    four_adjacent_success_rate = four_adjacent_success_count / four_adjacent_try_count if four_adjacent_try_count > 0 else 0
    with open("four_adjacent_results.txt", mode='w') as f:
        f.writelines(four_adjacent_results)
        f.write(f"\nfour_adjacent success_rate: {four_adjacent_success_rate * 100}%\n")

    # Write the 5-adjacent results and success rate to a separate file
    five_adjacent_success_rate = five_adjacent_success_count / five_adjacent_try_count if five_adjacent_try_count > 0 else 0
    with open("five_adjacent_results.txt", mode='w') as f:
        f.writelines(five_adjacent_results)
        f.write(f"\nfive_adjacent success_rate: {five_adjacent_success_rate * 100}%\n")

    print("\ntotal success_rate:", success_rate * 100, "%")
    print(f"adjacent success_rate: {adjacent_success_rate * 100}%")
    print(f"three_adjacent success_rate: {three_adjacent_success_rate * 100}%")
    print(f"four_adjacent success_rate: {four_adjacent_success_rate * 100}%")
    print(f"five_adjacent success_rate: {five_adjacent_success_rate * 100}%")
    print("-----------------------------finish-----------------------------")

if __name__ == '__main__':
    main()