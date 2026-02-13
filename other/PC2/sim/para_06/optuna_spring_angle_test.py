#ひとまず完成用
#完成！！

import numpy as np
import matplotlib.pyplot as plt
import math
import time
import matplotlib.patches as pat
from matplotlib import cm
import csv
import datetime
from collections import defaultdict
from tqdm import tqdm
from multiprocessing import Pool, cpu_count
import random
import os
from datetime import datetime
import optuna

#キー入力でグラフを閉じる
def on_key(event):
    if event.key == ' ':
        plt.close()


def judge_success_rate_fig(file_path,file_fig_path):
    # データを格納するリスト
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
    plt.savefig(file_fig_path)
    # plt.show() 
    plt.clf()
        
#手先位置の更新
def new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, end_eff_x, end_eff_y,n):
    dt = 0.05
    # new_end_eff_x = end_eff_x + (gole_position[0] - first_pos_x[n]) * (dt /4 )
    # new_end_eff_y = end_eff_y + (gole_position[1] - first_pos_y[n]) * (dt /4 )
    new_end_eff_x = end_eff_x + (gole_position[0] - end_eff_x) * (dt)
    new_end_eff_y = end_eff_y + (gole_position[1] - end_eff_y) * (dt)
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
    # if judge == 1 and t > fail_time:
    #     fail_angle = np.zeros(3)
    #     fail_angle[0] = angle[2]
    #     fail_angle[1] = angle[3]
    #     fail_angle[2] = angle[4]
    #     fail_count = 1
    # else:
    #     fail_angle = np.zeros(3)

    # if fail_count == 1:
    #     fail_angle[0] = angle[2]
    #     fail_angle[1] = angle[3]
    #     fail_angle[2] = angle[4]

            
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
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            else:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = (np.pi) - math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = -1 * math.cos(angle[i])
            else:
                angle[i] = (np.pi) - math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = math.sin(angle[i])
                cos_angle[i] = -1*math.cos(angle[i])
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
            else:
                angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = math.cos(angle[i])
                angle[i] =  math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))

            # angle[i] = -1* math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
            # sin_angle[i] = -1*math.sin(angle[i])
            # cos_angle[i] = math.cos(angle[i])
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i])) + np.pi
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = -1*math.cos(angle[i])
            else:
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i])) + np.pi
                sin_angle[i] = -1*math.sin(angle[i])
                cos_angle[i] = -1*math.cos(angle[i])
                angle[i] = math.atan2((end_eff_y[i+1] - end_eff_y[i]), (end_eff_x[i+1] - end_eff_x[i]))
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] > end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = 0
                cos_angle[i] = 1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = 1
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = np.pi
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = np.pi
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = np.pi/2
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = 1
                cos_angle[i] = 0
            else:
                angle[i] = np.pi/2
                sin_angle[i] = 1 
                cos_angle[i] = 0
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = -1 * np.pi/2
                # angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = -1
                cos_angle[i] = 0
            else:
                angle[i] = -1 * np.pi/2
                sin_angle[i] = -1
                cos_angle[i] = 0
        else:
            print("error")
            time.sleep(100)
        

    #print("angle:",angle)
    #print("angle (degrees):", np.rad2deg(angle))
    
    #関節角度の制限
    P_R = np.zeros(n-1)
    dL_data = np.zeros(n-1)
    if count % 50 == 0:
        KD = 0
    else:
        KD =L *2
    
    # KD = (L * 2 * 1/(n-1))

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
def runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,end_eff_x,end_eff_y, judge, t, fail_count, angle, count):
    #print("x_vec:",x_vec)
    k1, dL, angle,fail_count= f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t,fail_count, angle,count)
    #print("k1:",k1)
    k2, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k1,end_eff_x,end_eff_y,judge,t,fail_count, angle,count)
    #print("k2:",k2)
    k3, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k2,end_eff_x,end_eff_y,judge,t,fail_count, angle,count)
    #print("k3:",k3)
    k4, dL , angle,fail_count = f_n(L, K, D, n, theta_0, x_vec + dt * k3,end_eff_x,end_eff_y,judge ,t,fail_count,   angle,count)
    # #print("k4:",k4)
    return x_vec + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4) , dL, angle, fail_count

#nリンクの時のグラフの更新
def new_posi_spring_n(tmax, dt, t, x_vec, theta_0, gole_position,n, L, K, D, initial_position,first_pos_x,first_pos_y, spring_len_judg,theta_n,ran):
    # グラフの準備
    # fig, ax = plt.subplots()
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
    initial_angle = angle
    #リンクの描画
    # for i in range(n):
    #     ax.plot([plt_x[i],plt_x[i+1]],[plt_y[i],plt_y[i+1]],color="black",marker="o",linestyle="-")
    #     plt.gca().set_aspect('equal', adjustable='box')
    # #目標位置の描画
    # ax.plot(gole_position[0],gole_position[1],color="red",marker="o")
    # #グラフの設定
    # ax.set_xlim(-L *(n * 0.1),L *(n * 1.1))  # x軸の範囲を設定
    # ax.set_ylim(-L *(n * 0.1),L *(n * 1.1))  # y軸の範囲を設定
    # ax.grid(True)  # グリッドを表示
    # plt.connect('key_press_event', on_key)
    # #グラフの左上に”spoe”と文字を表示
    # plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "start", fontsize=20, color="red")
    # #print("plt_x:",plt_x)
    # #print("plt_y:",plt_y)
    # plt.show()
    # time.sleep(100)
    
    #変数の初期化
    gole_x = gole_position[0]
    gole_y = gole_position[1]
    count = 0
    judge = 1
    fail_count = 0
    fail_time = 0
    fail_angle = np.zeros(n)
    mani_x = np.zeros(n)
    mani_y = np.zeros(n)
    normal_angle = np.zeros(n)
    final_x = np.zeros(n)
    final_y = np.zeros(n)
    z =0
    sp_judge = 0
    fin_judge = 0
    
    
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
        x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,plt_x,plt_y, judge, t, fail_count,angle,count)


        # 各要素の更新
        x = x_vec[0:n-1]
        y = x_vec[(n-1):2*(n-1)]
        # v_x = x_vec[2*(n-1):3*(n-1)]
        # v_y = x_vec[3*(n-1):4*(n-1)]
        #print("x:",x)
        #print("y:",y)
        #print("v_x:",v_x)
        #print("v_y:",v_y)
        
        # print("plt_x[n]",plt_x[n])
        # print("plt_y[n]",plt_y[n])
        
        
        #手先位置の更新
        if judge == 1:
            judge = 0
            end_eff_x,end_eff_y = new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, plt_x[n], plt_y[n],n)
            # plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "judge", fontsize=20, color="red")
            # print("-----------------------------judge:-----------------------------")
            #関節の故障を計算   
            for i in range(n):
                fail_angle[i] = angle[i]
                normal_angle[i] = angle[i]
            # print("angle:",angle)
                # print("fail_angle:",fail_angle)
            
            #計算されたマニピュレータのリンクの描画
            for i in range(n): 
                if i ==0:
                    final_x[i] = L * math.cos(normal_angle[i])
                    final_y[i] = L * math.sin(normal_angle[i])
                elif i == n-1:
                    final_x[i] = final_x[i-1] + L * math.cos(normal_angle[i])
                    final_y[i] = final_y[i-1] + L * math.sin(normal_angle[i])
                else:
                    final_x[i] = final_x[i-1] + L * math.cos(normal_angle[i])
                    final_y[i] = final_y[i-1] + L * math.sin(normal_angle[i])
            
            # fail_angle[2] = angle[2]*0.95
            # fail_angle[3] = angle[3]*0.95
            # fail_angle[4] = angle[4]*0.95

            #実際のマニピュレータのリンクの描画
            for i in range(n):
                if i == 0:
                    mani_x[i] = L * math.cos(fail_angle[i])
                    mani_y[i] = L * math.sin(fail_angle[i])
                    x[i] =mani_x[i]
                    y[i] = mani_y[i]
                elif i == n-1:
                    mani_x[i] = mani_x[i-1] + L * math.cos(fail_angle[i])
                    mani_y[i] = mani_y[i-1] + L * math.sin(fail_angle[i])
                else:
                    mani_x[i] = mani_x[i-1] + L * math.cos(fail_angle[i])
                    mani_y[i] = mani_y[i-1] + L * math.sin(fail_angle[i])
                    x[i] = mani_x[i]
                    y[i] = mani_y[i]

            #print("end_eff_x:",end_eff_x)
            #print("end_eff_y:",end_eff_y)
            # リンクの更新
            # mani_plt_x = np.concatenate([np.array([initial_position[0]]), mani_x])
            # mani_plt_y = np.concatenate([np.array([initial_position[1]]), mani_y])
            # final_plt_x = np.concatenate([np.array([initial_position[0]]), final_x])
            # final_plt_y = np.concatenate([np.array([initial_position[1]]), final_y])

            #計算されたマニピュレータのリンクの描画
            # for i in range(n):
            #     ax.plot([final_plt_x[i],final_plt_x[i+1]],[final_plt_y[i],final_plt_y[i+1]],color="red",marker="o",linestyle="-")
            #     plt.gca().set_aspect('equal', adjustable='box')

            #実際のマニピュレータのリンクの描画
            # for i in range(n):
            #     ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
            #     plt.gca().set_aspect('equal', adjustable='box')    
            
            # plt.draw()
            # plt.pause(dt)  # グラフの更新を表示するための待機時間
            # time.sleep(1)
            z = 1
        elif judge == 0:
            angle = angle
            # time.sleep(100)
            pass

        
        # リンクの更新
        plt_x = np.concatenate([np.array([initial_position[0]]), x, np.array([end_eff_x])])
        plt_y = np.concatenate([np.array([initial_position[1]]), y, np.array([end_eff_y])])
        # print("plt_x:",plt_x)
        # print("plt_y:",plt_y)
        # for i in range(n):
        #     ax.plot([plt_x[i],plt_x[i+1]],[plt_y[i],plt_y[i+1]],color="blue",marker="o",linestyle="-")
        #     plt.gca().set_aspect('equal', adjustable='box')
        # # ゴール座標の表示
        # ax.plot(gole_position[0],gole_position[1],color="red",marker="o")

        # #計算されたマニピュレータのリンクの描画
        # for i in range(n):
        #     ax.plot([final_plt_x[i],final_plt_x[i+1]],[final_plt_y[i],final_plt_y[i+1]],color="red",marker="o",linestyle="-")
        #     plt.gca().set_aspect('equal', adjustable='box')

        # #実際のマニピュレータのリンクの描画
        # for i in range(n):
        #     ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
        #     plt.gca().set_aspect('equal', adjustable='box')

        # plt.draw()
        # plt.pause(dt)  # グラフの更新を表示するための待機時間

        if z == 1 and count > 1:
            x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,plt_x,plt_y, judge, t, fail_count,angle,count)
            z =0
        
        
        # plt.draw()
        # plt.pause(dt)  # グラフの更新を表示するための待機時間
        for i in range(n):
            if np.all(plt_x[i] - ran <= plt_x[i+1] ) and np.all(plt_x[i+1] <= plt_x[i] + ran) and np.all(plt_y[i] - ran <= plt_y[i+1]) and np.all(plt_y[i+1] <= plt_y[i] + ran):
                # print("点が重なりました。")
                success = "cross"
                return success
        # 角度がすべて50度になったら、ばねの運動を収束させる
        # for i in range (n-1):
        #     if np.all(np.rad2deg(angle[i]) -np.rad2deg(angle[i+1]) <= 0.1) and np.all(np.rad2deg(angle[i]) -np.rad2deg(angle[i+1]) >= -0.1):
        #         sp_judge += 1
        #         # print("sp_judge:",sp_judge)
                
        #     if sp_judge == 4:
        #         fin_judge = 1
        #         #print("judge:",judge)
        
        
        # if fin_judge == 1:
        #     print("特異点です。")
        #     success = 0
        #     return success
        
        max_dL = np.max(np.abs(dL))
        # print("max_dL:",max_dL)
        if count > 1:  # count が 1 以上の場合に条件を満たすように変更
            # ばねの運動が収束したら、ループを抜ける
            if max_dL <spring_len_judg:
                #print("ばねの運動が収束しました。")
                # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                judge = 1
                        #ゴールの判定
                if np.all(gole_position[0] - ran <= mani_x[n-1]) and np.all(mani_x[n-1] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= mani_y[n-1]) and np.all(mani_y[n-1] <= gole_position[1] + ran):
                           # グラフの初期化
                    # ax.clear()
                    # plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "fin", fontsize=20, color="red")                    
                    # print("ゴールしました。")
                    success = 1
                    return success
                    
                    # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                    # break
                # break
            elif max_dL >= L :
                # print("ばねの運動が収束していません。")
                success = 0
                return success
                # break
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


    
def run1(k,file_path,k_pa,D_pa):
    # print("-----link:",k,"start-----")            
    #パラメータ設定
    L = 10 #リンクの長さ
    ran = 0.1 #目標位置の許容範囲
    theta_0 = 80 #初期角度
    theta_n = 60 #目標角度
    initial_position = [0,0] #初期位置
    # gole_position = [10,15] #目標位置
    # gole_position = [50,50] #目標位置 n=6
    # gole_position = [35,35] #目標位置
    # gole_position = [30,20 ] #目標位置
    # gole_position = [80,40] #目標位置 n=10
    spring_len_judge = 0.025 #ばねの収束判定範囲
    success = 0 #成功判定
    test = 0
    failure_count =0 #テスト回数
    all_success = 0 #成功数
    
    file_mode = 0  #mode0: 関節固定、mode1：出力変化

    # 物理定数
    m_pa = 1.0
    # k_pa = 100
    # D_pa = 10

    # 時間変数
    tmin = 0.0
    tmax = 500
    dt = 0.05
    t = tmin  

    #csvファイル
    # file_dL = 'spring-dL.csv'
    # file_angle = 'spring-angle.csv'
    # file_vec = 'spring-vec.csv'
    # file_path = "K100vsD10.txt"
    
    # dL_data = []
    # vce_data = []
    # angle_data = []

    # #csvファイルの初期化
    # with open(file_dL, mode='w') as f:
    #     f.write("")
    # with open(file_angle, mode='w') as f:
    #     f.write("")
    # with open(file_vec, mode='w') as f:
    #     f.write("")
    # with open(file_path, mode='w') as f:
    #     f.write("")
        # print("link:", k)
        
    test = 0
    failure_count =0
    failure_count = 0
    #初期化
    success_positions = []  # 成功した座標のリスト
    failure_positions = []  # 失敗した座標のリスト
    timeover_positions = []  # タイムオーバーした座標のリスト
    cross_positions = []  # 点が重なった座標のリスト

    
    # 成功率の計算と座標の取得
    test = 0
    failure_count =0  # テスト回数
    all_success = 0  # 成功数
    #初期化
    success_positions = [] # 成功した座標のリスト
    failure_positions = [] # 失敗した座標のリスト
    timeover_positions = [] # タイムオーバーした座標のリスト
    cross_positions = [] # 点が重なった座標のリスト

    loop = 500
    # for i in tqdm(range(loop),desc="link"+str(k),leave=False):
    for i in range(loop):
        test += 1
        # print("link:",k,", test:", test)
        first_pos_x, first_pos_y = first_position(k , L ,initial_position, theta_0, theta_n)
        #print("first_pos_x:",first_pos_x)
        #print("first_pos_y:",first_pos_y)
        
        mov_x, mov_y = move_posi(first_pos_x,first_pos_y,k)
        #print("mov_x:",mov_x)
        
        # 数値解用変数
        x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(k-1),np.zeros(k-1)])  #[xの各座標、yの各座標、vx,vy]
        #print("x_vec0:",x_vec_n)

        # ランダムな目標点を設定
        gole_position = [np.random.randint(0, k*10), np.random.randint(0, k*10)]
        # 目標位置が初期位置と同じか到達不可能な場合は再設定
        while gole_position == initial_position or np.sqrt((gole_position[0] - initial_position[0])**2 + (gole_position[1] - initial_position[1])**2) > L * k:
            gole_position = [np.random.randint(0, k*10), np.random.randint(0, k*10)]

        # print("gole_position:", gole_position)

        # シミュレーション
        success = new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, gole_position, k, L, k_pa, D_pa, initial_position, first_pos_x, first_pos_y, spring_len_judge, theta_n, ran)
        # print("success:", success)

        # 成功数を計算
        if success == 1:
            all_success += 1
            success_positions.append(gole_position)
        elif success == 0:
            failure_count += 1
            failure_positions.append(gole_position)
        elif success == None:
            failure_positions.append(gole_position)
        elif success == "cross":
            failure_positions.append(gole_position)

            
        #失敗が多い場合は終了
        if failure_count > loop * (1-0.85):
            break
        # 成功率を計算
        success_rate = all_success / test
        # print("success_rate:", success_rate * 100, "%", "\n")
                

    # 成功率を計算
    success_rate = all_success / test

        
    # 成功した座標を青い点で、失敗した座標を赤い点でプロット
    # success_positions = np.array(success_positions)
    # failure_positions = np.array(failure_positions)
    # timeover_positions = np.array(timeover_positions)
    # cross_positions = np.array(cross_positions)

    # グラフの設定
    # fig, ax = plt.subplots()
    # ax.set_xlim(-L * (k * 0.1), L * (k * 1.1))  # x軸の範囲を設定
    # ax.set_ylim(-L * (k * 0.1), L * (k * 1.1))  # y軸の範囲を設定
    # #初期状態を描画
    # plt_x = np.zeros(k+1)
    # plt_y = np.zeros(k+1)
    # plt_x[0] = initial_position[0]
    # plt_y[0] = initial_position[1]
    # #初期角度を設定
    # angle = np.zeros(k)
    # angle[0] = np.deg2rad(theta_0)
    # theta = theta_n / k
    # #リンクの角度を計算
    # for i in range(k-1):
    #     angle[i+1] = angle[i] + np.deg2rad(-theta)
    # #リンクの位置を計算
    # for i in range(k):
    #     plt_x[i+1] = plt_x[i] + L * math.cos(angle[i])
    #     plt_y[i+1] = plt_y[i] + L * math.sin(angle[i])
    # # リンクの描画
    # for i in range(k):
    #     ax.plot([plt_x[i],plt_x[i+1]],[plt_y[i],plt_y[i+1]],color="black",marker="o",linestyle="-", linewidth=2)
    #     plt.gca().set_aspect('equal', adjustable='box')

    # # NumPy配列ではなくPythonリストなので、インデックス指定は [:][0] のように書きます
    # plt.scatter([pos[0] for pos in success_positions], [pos[1] for pos in success_positions], color='blue', label='Success',marker="x")
    # plt.scatter([pos[0] for pos in failure_positions], [pos[1] for pos in failure_positions], color='red', label='Failure',marker="x")
    # plt.scatter([pos[0] for pos in timeover_positions], [pos[1] for pos in timeover_positions], color='green', label='Timeover',marker="x")
    # plt.scatter([pos[0] for pos in cross_positions], [pos[1] for pos in cross_positions], color='yellow', label='Cross',marker="x")
    # plt.xlabel('X-axis')
    # plt.ylabel('Y-axis')
    # plt.title('Success and Failure Positions')
    # 画像として保存
    #デスクトップに保存
    # plt.savefig(fr'C:\Users\rsait\OneDrive - 東京理科大学\デスクトップ\git\study_B4\test_all\012_n_change\data\link_{k}.png')
    #ノートパソコンに保存
    # plt.savefig(fr"C:\Users\81808\Documents\GitHub\study_B4\test_all\013_multi\data\link_{k}.png")
    #相対パスで保存
    # plt.savefig(fr'./data/K100vsD10/link_{k}.png')
            
    # ファイルに書き込むデータを作成
    data_to_write = f"parameter:\n link:{k} \n success_rate:{success_rate * 100} \n\n"

    # ファイルに書き込み
    with open(file_path, 'a') as file:
        file.write(data_to_write)
    # plt.close('all')
    # print(f"link:{k} finished")
    # print("-----link:",k,"finished-----", "\n")

    #結果の出力
    # print("link_",k,"_success_rate_all:", success_rate * 100, "%")
    return success_rate,k



def calculate_total_success_rate(file_path):
    total_success_rate = 0.0
    
    try:
        with open(file_path, 'r') as file:
            for line in file:
                # テキストファイルの各行を処理する
                if line.strip().startswith("success_rate:"):
                    # success_rate: の後に続く数値を取得する
                    try:
                        success_rate = float(line.split(":")[1].strip())
                        total_success_rate += success_rate
                    except ValueError:
                        print(f"Warning: Invalid success_rate value in line: {line}")
    
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
    return total_success_rate

def process_task(args):
    return run1(*args)

def judge_success_rate(file_path, k_pa, D_pa):
    # 成功率の計算
    min_value = 2
    max_value = 26
    p = Pool(processes=cpu_count()-1)
    print(f"cpu_count:{cpu_count()-1}")
    print("-----------------------------start-----------------------------")
    print("k_pa:",k_pa,", D_pa:",D_pa)

    # # 並列処理
    # for i_ in range(min_value, max_value):
    #     p.apply_async(run1, args=(i_, file_path,k_pa,D_pa))
 



    # プロセスプールの作成
    with Pool(processes=cpu_count()-1) as p:
        # 引数を含むタプルのリストを作成
        tasks = [(i, file_path, k_pa, D_pa) for i in range(min_value, max_value)]
        # tqdm で進行状況バーを表示
        with tqdm(total=max_value - min_value) as pbar:
            # タスクを実行
            for _ in p.imap_unordered(process_task, tasks):
                pbar.update()
                pbar.write(f"link:{_[1]} success_rate:{_[0] * 100} %")    

    # for i in range(min_value, max_value):
    #     print(f"----------link:{i} start----------")
    #     run1(i)
    #     print(f"----------link:{i} finished----------", "\n")
          
            
    # print("-----------------------------finish-----------------------------")
    p.close()
    p.join()
    total = calculate_total_success_rate(file_path)
    if total == 2400:
        file_fig_path =  os.path.join(all_success_path, f"K{k_pa}_D{D_pa}.png")
        print("all_success")
    else:
        file_fig_path =  os.path.join(dir_path, f"K{k_pa}_D{D_pa}.png")
    judge_success_rate_fig(file_path,file_fig_path)
    print("-----------------------------finish-----------------------------")
    print(f"Total success rate: {total}")
    return total
    
    # judge_success_rate(file_path)

#最適化処理
def objective(trial):
    # 目標値
    target_success_rate = 2400 
    # パラメータの設定
    k_pa = trial.suggest_int("k_pa", 10, 100)
    D_pa = trial.suggest_int("D_pa", 5, 10)
    
    # ファイルパスの設定
    file_path = os.path.join(dir_path, f"K{k_pa}_D{D_pa}.txt")

    # ファイルの初期化
    with open(file_path, mode='w') as f:
        f.write("")
    
    # 成功率の計算
    success_rate = judge_success_rate(file_path, k_pa, D_pa)
    
    # スコアの計算
    score = (target_success_rate - abs(target_success_rate - success_rate)) * 100 / target_success_rate
    # score = (target_success_rate - abs(target_success_rate - success_rate)) * 100 / target_success_rate

    # 結果の表示
    print(f"K: {k_pa}, D: {D_pa}, success_rate: {score}")
    print("\n")

    return score

def main():
    #最適化の設定
    study = optuna.create_study(direction='maximize')
    study.optimize(objective, n_trials=100)

    best_params = study.best_params
    best_success_rate = study.best_value

    #最適化結果の表示
    print("Best params:", best_params)
    print("Best success rate:", best_success_rate)

if __name__ == "__main__":
    #ファイルパスの設定
    now = datetime.now()
    date_time_str = now.strftime("%Y%m%d_%H%M%S")
    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../result"))
    dir_path = os.path.join(base_dir, date_time_str)
    all_success_path = os.path.join(dir_path, "all_success")
    os.makedirs(dir_path, exist_ok=True)
    os.makedirs(all_success_path, exist_ok=True)
    
    main()