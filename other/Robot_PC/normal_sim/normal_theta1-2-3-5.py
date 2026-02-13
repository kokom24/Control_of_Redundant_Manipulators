#ひとまず完成用
#完成！！

from collections import defaultdict
import csv
import matplotlib.pyplot as plt
import os
from datetime import datetime
import time
import itertools
import numpy as np
import math
import multiprocessing
from multiprocessing import Process, Queue, Pipe
import math
from tqdm import tqdm
import re



#キー入力でグラフを閉じる
def on_key(event):
    if event.key == ' ':
        plt.close()
        
#手先位置の更新
def new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, end_eff_x, end_eff_y,n):
    dt = 0.05
    # new_end_eff_x = end_eff_x + (gole_position[0] - first_pos_x[n]) * (dt /4 )
    # new_end_eff_y = end_eff_y + (gole_position[1] - first_pos_y[n]) * (dt /4 )
    new_end_eff_x = end_eff_x + (gole_position[0] - end_eff_x) * (dt/(n/20))
    new_end_eff_y = end_eff_y + (gole_position[1] - end_eff_y) * (dt/(n/20))
    return new_end_eff_x, new_end_eff_y


#nリンクの時の連立微分方程式の右辺
def f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t, fail_count, angle):
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

    fail_angle = np.zeros(10)
    fail_angle[0] = angle[1]
    fail_angle[1] = angle[2]
    fail_angle[2] = angle[3]
    # fail_angle[3] = angle[4]
    fail_angle[4] = angle[5]
    # fail_angle[5] = angle[6]
    # fail_angle[6] = angle[7]
    # fail_angle[7] = angle[8]
    # fail_angle[8] = angle[9]
    # fail_angle[9] = angle[10]
        
    # #関節の故障を計算   
    # if judge == 1 and t > fail_time:
    #     fail_angle = np.zeros(3)
    #     fail_angle[0] = angle[2]
    #     # fail_angle[1] = angle[3]
    #     # fail_angle[2] = angle[4]
    #     fail_count = 1
    #     print("-----------------------------fail_count:-----------------------------")
    # else:
    #     fail_angle = np.zeros(3)

    # if fail_count == 1:
    #     fail_angle[0] = angle[2]
    #     # fail_angle[1] = angle[3]
    #     # fail_angle[2] = angle[4]

            
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
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                angle[1] = fail_angle[0]
                angle[2] = fail_angle[1]
                angle[3] = fail_angle[2]
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
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
                # angle[4] = fail_angle[3]
                angle[5] = fail_angle[4]
                # angle[6] = fail_angle[5]
                # angle[7] = fail_angle[6]
                # angle[8] = fail_angle[7]
                # angle[9] = fail_angle[8]
                # angle[10] = fail_angle[9]
                sin_angle[i] = -1
                cos_angle[i] = 0
            else:
                angle[i] = 0
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
    KD =0 
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
def runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,end_eff_x,end_eff_y, judge, t, fail_count, angle):
    #print("x_vec:",x_vec)
    k1, dL, angle,fail_count= f_n(L, K, D, n, theta_0, x_vec,end_eff_x,end_eff_y,judge,t,fail_count, angle)
    #print("k1:",k1)
    k2, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k1,end_eff_x,end_eff_y,judge,t,fail_count, angle)
    #print("k2:",k2)
    k3, dL , angle, fail_count = f_n(L, K, D, n, theta_0, x_vec + (dt / 2) * k2,end_eff_x,end_eff_y,judge,t,fail_count, angle)
    #print("k3:",k3)
    k4, dL , angle,fail_count = f_n(L, K, D, n, theta_0, x_vec + dt * k3,end_eff_x,end_eff_y,judge ,t,fail_count,   angle)
    # #print("k4:",k4)
    return x_vec + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4) , dL, angle, fail_count

#nリンクの時のグラフの更新
def new_posi_spring_n(tmax, dt, t, x_vec, theta_0, gole_position,n, L, K, D, initial_position,first_pos_x,first_pos_y, spring_len_judg):
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
    #目標位置の描画
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
    success = 0
    
    
    #シミュレーションの実行
    while t < tmax:
        #print("/////////////////////////////count:",count,"/////////////////////////////")
        # print("fail_count:",fail_count)
        # print("t:",t)
        t += dt
        count += 1

        # if t >tmax *0.8:
        #     print(f"time:{t}")
         
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
        x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,plt_x,plt_y, judge, t, fail_count,angle)


        # 各要素の更新
        x = x_vec[0:n-1]
        y = x_vec[(n-1):2*(n-1)]
        v_x = x_vec[2*(n-1):3*(n-1)]
        v_y = x_vec[3*(n-1):4*(n-1)]
        #print("x:",x)
        #print("y:",y)
        #print("v_x:",v_x)
        #print("v_y:",v_y)
        
        # print("plt_x[n]",plt_x[n])
        # print("plt_y[n]",plt_y[n])
        
        
        #手先位置の更新
        if judge == 1:
            judge = 0
            
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
            
            fail_angle[1] = initial_angle[1]
            fail_angle[2] = initial_angle[2]
            fail_angle[3] = initial_angle[3]
            # fail_angle[4] = initial_angle[4]
            fail_angle[5] = initial_angle[5]
            # fail_angle[6] = initial_angle[6]
            # fail_angle[7] = initial_angle[7]
            # fail_angle[8] = initial_angle[8]

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
            mani_plt_x = np.concatenate([np.array([initial_position[0]]), mani_x])
            mani_plt_y = np.concatenate([np.array([initial_position[1]]), mani_y])
            final_plt_x = np.concatenate([np.array([initial_position[0]]), final_x])
            final_plt_y = np.concatenate([np.array([initial_position[1]]), final_y])

            #計算されたマニピュレータのリンクの描画
            # for i in range(n):
            #     ax.plot([final_plt_x[i],final_plt_x[i+1]],[final_plt_y[i],final_plt_y[i+1]],color="red",marker="o",linestyle="-")
            #     plt.gca().set_aspect('equal', adjustable='box')

            #実際のマニピュレータのリンクの描画
            # for i in range(n):
            #     ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
            #     plt.gca().set_aspect('equal', adjustable='box')    
                
                
            end_eff_x,end_eff_y = new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, mani_plt_x[n], mani_plt_y[n],n)
            
            # plt.draw()
            # plt.pause(dt)  # グラフの更新を表示するための待機時間



            z = 1
        elif judge == 0:
            angle = angle
            # time.sleep(100)
            pass

        # if fail_count == 1:
        #     plt.text(L *(n * 0.7), L *(n * 1.1)+2, "angle_fixed", fontsize=18, color="blue")
        
    
        
        # リンクの更新
        plt_x = np.concatenate([np.array([initial_position[0]]), x, np.array([end_eff_x])])
        plt_y = np.concatenate([np.array([initial_position[1]]), y, np.array([end_eff_y])])
        # print("plt_x:",plt_x)
        # print("plt_y:",plt_y)
        # for i in range(n):
        #     ax.plot([plt_x[i],plt_x[i+1]],[plt_y[i],plt_y[i+1]],color="blue",marker="o",linestyle="-")
        #     plt.gca().set_aspect('equal', adjustable='box')
        # ゴール座標の表示
        # ax.plot(gole_position[0],gole_position[1],color="red",marker="o")

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

        if z == 1 and count > 1:
            x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,plt_x,plt_y, judge, t, fail_count,angle)
            z =0
        
        # データの保存
        # dL_data.append([count * dt] + [dL[i] for i in range(n)])
        # vce_data.append([count * dt] + [x_vec[i] for i in range(4*(n-1))])
        # angle_data.append([count * dt] + [np.rad2deg(fail_angle[i]) for i in range(n)])
        # # 角度のデータをcsvファイルに保存
        # with open(file_angle, 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['time']+[f'angle{i+1}' for i in range(n)])
        #     writer.writerows(angle_data)
        #     # time.sleep(1)
        # # 面積のデータをcsvファイルに保存
        # with open(file_dL, 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(["time"]+[f'dL{i+1}' for i in range(n)])  # ヘッダーを書き込む
        #     writer.writerows(dL_data)
        # # 手先位置のデータをcsvファイルに保存
        # with open(file_vec, 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['time']+[f'x{i+1}' for i in range(n-1)]+[f'y{i+1}' for i in range((n-1))]+[f'vx{i+1}' for i in range((n-1))]+[f'vy{i+1}' for i in range((n-1))])
        #     writer.writerows(vce_data)


        max_dL = np.max(np.abs(dL))
        # print("max_dL:",max_dL)
        if count > 1:  # count が 1 以上の場合に条件を満たすように変更
            # ばねの運動が収束したら、ループを抜ける
            
            if max_dL <spring_len_judg:
                # print("ばねの運動が収束しました。")
                # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                judge =1

                        #ゴールの判定
                if np.all(gole_position[0] - ran <= mani_x[n-1]) and np.all(mani_x[n-1] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= mani_y[n-1]) and np.all(mani_y[n-1] <= gole_position[1] + ran):
                     # グラフの初期化
                    # ax.clear()
                    # plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "fin", fontsize=20, color="red")                    
                    # print("ゴールしました。")
                    # #リンクの描画
                    # for i in range(n):
                    #     ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
                    #     plt.gca().set_aspect('equal', adjustable='box')
                    # ax.set_xlim(-L *(n * 0.1),L *(n * 1.1))  # x軸の範囲を設定
                    # ax.set_ylim(-L *(n * 0.1),L *(n * 1.1))  # y軸の範囲を設定
                    # plt.draw()
                    # plt.pause(dt)  # グラフの更新を表示するための待機時間
                    # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                    success = 1
                    return success
                # break
            elif max_dL >= L * 10:
                # print("ばねの運動が収束していません。")
                success = 0
                return success
            else:
                judge = 0
            
                # if np.all(gole_position[0] - ran <= plt_x[n-1]) and np.all(plt_x[n-1] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= plt_y[n-1]) and np.all(plt_y[n-1] <= gole_position[1] + ran):
                #     judge = 1
                #     return plt_x[n-1], plt_y[n-1], judge
                # else:
                #     judge = 0
        # plt.draw()
        # plt.pause(dt)  # グラフの更新を表示するための待機時間
    success = 0
    return success


#初期条件の計算
def first_position(n , L ,initial_position, theta_0):
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

def move_posi(x,y):
    mov_x = np.zeros(joint_num-1)
    mov_y = np.zeros(joint_num-1)
    for i in range(joint_num-1):
        mov_x[i] = x[i+1]
        mov_y[i] = y[i+1]
    
    return mov_x, mov_y

    
def load_completed_trials_by_lines(results_file_path):
    completed_goals = []     # [[x, y], ...]
    completed_results = []   # [1, 0, 1, ...]

    pattern = re.compile(
        r"Goal\s+\(([-\d.]+),\s*([-\d.]+)\)\s*->\s*(\d+)"
    )

    with open(results_file_path, 'r', encoding='utf-8') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                result = int(match.group(3))

                completed_goals.append([x, y])
                completed_results.append(result)

    return completed_goals, completed_results


if __name__ == '__main__':
    #################### Set directory path ####################
    now = datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    day = now.strftime("%d")
    time_str = now.strftime("%H%M%S")
    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    # csv_file_path = os.path.join(dir_path, "angle.csv")
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)
    main_run_dir = dir_path
    
    # 結果を保存するファイルのパス
    results_file_path = os.path.join(dir_path, "simulation_results.txt")
    # 条件を保存するファイルのパス
    conditions_file_path = os.path.join(dir_path, "conditions.txt")
    # 目標位置を保存するファイルのパス
    goal_positions_file_path = os.path.join(dir_path, "goal_positions.txt")

    #################### 2. シミュレーションのパラメータ設定 ####################
    m = 1
    L = 10
    g = 0
    k = 100
    d = 10
    dt = 0.05
    joint_num = 10
    success_count = 0
    initial_position = [0,0] #初期位置
    goal_position = [60,60] #目標位置
    num_trials = 1000 # 試行回数
    first = 1  # 最初の1回だけ固定ゴールでテストするフラグ
    ran = 1 #目標位置の許容範囲
    theta_0 = 80 #初期角度
    theta_n = 80 #目標角度
    spring_len_judge = 0.03 #ばねの収束判定範囲

    # 物理定数
    m_pa = 1.0
    k_pa = 100
    D_pa = 10


    # 時間変数
    tmin = 0.0
    tmax = 200.0
    dt = 0.03
    t = tmin

    print("ロボットアームの到達成功率シミュレーションを開始します。")
    print(f"結果は {dir_path} に保存されます。")
    print(f"総試行回数: {num_trials}\n")
    start_time = time.time()

    # 初期条件
    initial_conditions = f"n={joint_num}, L={L}, theta_0={theta_0}, theta_n={theta_n}, initial_position={initial_position}, gole_position={goal_position}, spring_len_judge={spring_len_judge}, m_pa={m_pa}, k_pa={k_pa}, D_pa={D_pa}, tmin={tmin}, tmax={tmax}, dt={dt}"

    # ファイルに書き込むデータを作成
    # data_to_write = f"date:{formatted_datetime}\nParameter:{initial_conditions}\n\n"

    # # # ファイルに書き込み
    # # file_path = r"C:\Users\81808\Documents\GitHub\study_B4\spring\deta\data-fail-1.txt"  #ノートPC用
    # file_path = r"C:\Users\rsait\OneDrive - 東京理科大学\デスクトップ\git\study_B4\spring\deta\data-1.txt" #デスクトップPC用
    # with open(file_path, 'a') as file:
    #     file.write(data_to_write)
    # print("初期条件:",initial_conditions)
    first_pos_x, first_pos_y = first_position(joint_num, L, initial_position, theta_0)
    #print("first_pos_x:",first_pos_x)
    #print("first_pos_y:",first_pos_y)
    
    mov_x, mov_y = move_posi(first_pos_x,first_pos_y)
    #print("mov_x:",mov_x)
    
    # 数値解用変数
    x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(joint_num-1),np.zeros(joint_num-1)])  #[xの各座標、yの各座標、vx,vy]
    #print("x_vec0:",x_vec_n)


    # 各試行の結果を保存するリスト
    trial_results = []
    success_positions = []
    failure_positions = []
    timeover_positions = []

    original_cwd = os.getcwd() # 元の作業ディレクトリを保存


    # --- グラフ描画処理 ---
    fig, ax = plt.subplots(figsize=(10, 10))
    max_reach = L * joint_num
    ax.set_xlim(-max_reach * 0.2, max_reach * 1.1)
    ax.set_ylim(-max_reach * 0.2, max_reach * 1.1)
    ax.set_aspect('equal', adjustable='box')


    # ラベル、タイトルなどもすべて ax.set_... に変更
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # --- 4. グラフをメインディレクトリに画像ファイルとして保存 ---
    file_name = f'summary_graph_{num_trials}_trials.png'
    save_path = os.path.join(main_run_dir, file_name)

    #-----シミュレーション条件の保存-----
    with open(conditions_file_path, 'w', encoding='utf-8') as f:
        f.write(f"Simulation Conditions")
        f.write(f"file name: {__file__}\n")
        f.write(f"datetime: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"m: {m}\n")
        f.write(f"L: {L}\n")
        f.write(f"g: {g}\n")
        f.write(f"k: {k}\n")
        f.write(f"d: {d}\n")
        f.write(f"dt: {dt}\n")
        f.write(f"joint_num: {joint_num}\n")
        f.write(f"initial_position: {initial_position}\n")
        f.write(f"num_trials: {num_trials}\n\n")

    # ===== 途中再開用：既存結果の読み込み =====
    re_results_file_path = r"C:\Users\robot\Documents\GitHub\M1_styudy\result\summary_results_final_1000_trials.txt"  # 既存の結果ファイルパス
    completed_goals, completed_results = load_completed_trials_by_lines(re_results_file_path)
    skipped_count = 0
    for idx, (goal, result) in enumerate(zip(completed_goals, completed_results)):

        x, y = goal
        radius = np.sqrt(x**2 + y**2)

        # ===== 半径90〜100の円弧内はスキップ =====
        if 60 <= radius <= 100:
            skipped_count += 1
            continue

        if result == 1:
            success_positions.append(goal)
            success_count += 1
        elif result == 0:
            failure_positions.append(goal)
        goal_str = f"({goal[0]:.2f}, {goal[1]:.2f})"
        trial_results.append(
            f"Trial {idx+1:03d}: Goal {goal_str.ljust(18)} -> {result}"
        )

    start_trial_index = len(completed_goals) - skipped_count

    print(f"途中再開: {start_trial_index} 試行分を読み込みました")
    print(f"成功数: {success_count}")



    #シミュレーション
    # new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, gole_position,n, L, k_pa, D_pa, initial_position,first_pos_x,first_pos_y, spring_len_judge)

    
    #################### 3. 成功率計算ループ ####################
    
    for i in tqdm(range(start_trial_index, num_trials),desc="Simulating Trials", unit="trial"):

        # --- 2. 各試行ごとのサブディレクトリを作成し、そこに移動 ---
        trial_dir_name = f"trial_{i+1:03d}"
        current_trial_dir = os.path.join(main_run_dir, trial_dir_name)
        # os.makedirs(current_trial_dir, exist_ok=True)
        # os.chdir(current_trial_dir) # この試行のログファイル保存場所に移動
        random_goal = [np.random.randint(0, 100), np.random.randint(0, 100)]
        # 目標位置が初期位置と同じか到達不可能な場合は再設定
        while (random_goal == initial_position or np.sqrt(random_goal[0]**2 + random_goal[1]**2) < 0.6 * L * joint_num or np.sqrt(random_goal[0]**2 + random_goal[1]**2) > L * joint_num):
            random_goal = [np.random.randint(0,100), np.random.randint(0, 100)]
        if first == 1:
            first = 0
            # random_goal = [96.76, 8.76]  # ★固定ゴール座標でテストする場合はこちらを使用
            # random_goal = [1,1]  # ★固定ゴール座標でテストする場合はこちらを使用
            # random_goal = [12.46, 39.54]
            random_goal = [65,65]
            # random_goal = [80,60]
        
        # 目標位置をgoal_positions.txtに追記保存
        with open(goal_positions_file_path, 'a', encoding='utf-8') as f:
            f.write(f"Trial {i+1:03d}: Goal ({random_goal[0]:.2f}, {random_goal[1]:.2f})\n")

        result_status = new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, random_goal, joint_num, L, k_pa, D_pa, initial_position,first_pos_x,first_pos_y, spring_len_judge)

        os.chdir(original_cwd) # 元の作業ディレクトリに戻る

        # 結果に応じて座標を各リストに追加
        if result_status == 1:
            success_count += 1
            success_positions.append(random_goal)
        elif result_status == 0:
            failure_positions.append(random_goal)

        goal_str = f"({random_goal[0]:.2f}, {random_goal[1]:.2f})"
        result_line = f"Trial {i+1:03d}: Goal {goal_str.ljust(18)} -> {result_status}"
        trial_results.append(result_line)

         # --- 1. 現時点での統計情報を計算 ---
        current_trials = i + 1
        elapsed_time = time.time() - start_time
        success_rate = (success_count / current_trials) * 100 if current_trials > 0 else 0

        # --- 2. サマリーファイルの上書き保存 ---
        summary_header = f"--- Simulation Results (In Progress: {current_trials}/{num_trials}) ---"
        summary_body = [
            f"Total Trials Attempted: {current_trials}",
            f"Success: {success_count}",
            f"Failure: {len(failure_positions)}",
            f"Timeover: {len(timeover_positions)}",
            f"Success Rate: {success_rate:.2f}%",
            f"Elapsed Time: {elapsed_time:.2f}s"
        ]
        try:
            with open(results_file_path, 'w', encoding='utf-8') as f:
                f.write(summary_header.strip() + "\n")
                for line in summary_body:
                    f.write(line + "\n")
                
                f.write("\n--- Trial Details ---\n")
                for res_line in trial_results:
                    f.write(res_line + "\n")
        except Exception as e:
            print(f"\n[Error] Failed to write results to file: {e}")

        # --- 3. グラフの更新と上書き保存 ---
        ax.clear()  # 以前の描画内容をクリア

        # グラフの各種設定
        max_reach = L * joint_num
        ax.set_xlim(-max_reach * 0.2, max_reach * 1.1)
        ax.set_ylim(-max_reach * 0.2, max_reach * 1.1)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_title(f'Success Rate: {success_rate:.2f}% (Trials: {current_trials}/{num_trials})')
        ax.grid(True, linestyle='--', alpha=0.6)

        # アームの初期状態を描画
        ini_theta = 80
        plt_x = np.zeros(joint_num + 1)
        plt_y = np.zeros(joint_num + 1)
        angle = np.zeros(joint_num)
        angle[0] = np.deg2rad(ini_theta)
        for j in range(joint_num - 1):
            angle[j+1] = angle[j] + np.deg2rad(-ini_theta / joint_num)
        for j in range(joint_num):
            plt_x[j+1] = plt_x[j] + L * math.cos(angle[j])
            plt_y[j+1] = plt_y[j] + L * math.sin(angle[j])
        ax.plot(plt_x, plt_y, color="black", marker="o", linestyle="-", linewidth=2, label="Initial Position")

        # 結果の座標をプロット
        if success_positions:
            ax.scatter([pos[0] for pos in success_positions], [pos[1] for pos in success_positions], 
                       color='blue', label=f'Success ({len(success_positions)})', marker="o", alpha=0.7)
        if failure_positions:
            ax.scatter([pos[0] for pos in failure_positions], [pos[1] for pos in failure_positions], 
                       color='red', label=f'Failure ({len(failure_positions)})', marker="x")
        if timeover_positions:
            ax.scatter([pos[0] for pos in timeover_positions], [pos[1] for pos in timeover_positions], 
                       color='green', label=f'Timeover ({len(timeover_positions)})', marker="^")
        
        ax.legend(loc='upper right')
        ax.set_title(f'Success Rate: {success_rate:.2f}% (Trials: {num_trials})')

        # グラフを画像ファイルとして上書き保存
        try:
            fig.savefig(save_path)
        except Exception as e:
            print(f"\n[Error] Failed to save graph: {e}")

        # print(f"  -> Progress saved. Current success rate: {success_rate:.2f}%")
        
    end_time = time.time()
    total_time = end_time - start_time
    success_rate = (success_count / num_trials) * 100

    #################### 4. 結果の表示とファイルへの保存 ####################
    # コンソールへの最終結果表示
    summary_header = "\n--- シミュレーション結果 ---"
    summary_body = [
        f"総試行回数: {num_trials}",
        f"成功回数: {success_count}",
        f"失敗回数: {num_trials - success_count}",
        f"成功率: {success_rate:.2f}%",
        f"総計算時間: {total_time:.2f}秒"
    ]
    
    print(summary_header)
    for line in summary_body:
        print(line)

        # #################### 4. 最終処理 ####################
    end_time = time.time()
    total_time = end_time - start_time
    
    # 最終的なファイル名を変更（任意）
    final_results_path = os.path.join(main_run_dir, f'summary_results_final_{num_trials}_trials.txt')
    final_graph_path = os.path.join(main_run_dir, f'summary_graph_final_{num_trials}_trials.png')
    os.rename(results_file_path, final_results_path)
    os.rename(save_path, final_graph_path)

    # グラフウィンドウを閉じる
    plt.close(fig)

    print("\n" + "="*50)
    print("全てのシミュレーションが完了しました。")
    print(f"総計算時間: {total_time:.2f}秒")
    print(f"最終結果は以下のファイルに保存されています:\n- {final_results_path}\n- {final_graph_path}")
    print("="*50)