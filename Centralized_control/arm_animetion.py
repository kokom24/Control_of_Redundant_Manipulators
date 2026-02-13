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

    fail_angle = np.zeros(3)
    fail_angle[0] = angle[2]
    fail_angle[1] = angle[3]
    fail_angle[2] = angle[4]
        
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
                angle[2] = fail_angle[0]
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
                angle[2] = fail_angle[0]
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
                angle[2] = fail_angle[0]
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
                angle[2] = fail_angle[0]
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
                angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] == end_eff_y[i] and end_eff_x[i+1] < end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = 0
                cos_angle[i] = -1
            else:
                angle[i] = 0
                sin_angle[i] = 0
                cos_angle[i] = -1
        elif end_eff_y[i+1] > end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
                sin_angle[i] = 1
                cos_angle[i] = 0
            else:
                angle[i] = 0
                sin_angle[i] = 1 
                cos_angle[i] = 0
        elif end_eff_y[i+1] < end_eff_y[i] and end_eff_x[i+1] == end_eff_x[i]:
            if fail_count == 1:
                angle[i] = 0
                angle[2] = fail_angle[0]
                # angle[3] = fail_angle[1]
                # angle[4] = fail_angle[2]
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
    initial_angle = angle
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
    fail_count = 0
    fail_time = 0
    fail_angle = np.zeros(n)
    mani_x = np.zeros(n)
    mani_y = np.zeros(n)
    normal_angle = np.zeros(n)
    final_x = np.zeros(n)
    final_y = np.zeros(n)
    z =0
    
    
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
        ax.clear()
        ax.set_xlim(-L *(n * 0.1),L *(n * 1.1))  # x軸の範囲を設定
        ax.set_ylim(-L *(n * 0.1),L *(n * 1.1))  # y軸の範囲を設定
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
            
            plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "judge", fontsize=20, color="red")
            print("-----------------------------judge:-----------------------------")
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
            
            fail_angle[2] = initial_angle[2]
            fail_angle[3] = initial_angle[3]
            fail_angle[4] = initial_angle[4]

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
            for i in range(n):
                ax.plot([final_plt_x[i],final_plt_x[i+1]],[final_plt_y[i],final_plt_y[i+1]],color="red",marker="o",linestyle="-")
                plt.gca().set_aspect('equal', adjustable='box')

            #実際のマニピュレータのリンクの描画
            for i in range(n):
                ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
                plt.gca().set_aspect('equal', adjustable='box')    
                
                
            end_eff_x,end_eff_y = new_pos_eff(dt, first_pos_x,first_pos_y, gole_position, mani_plt_x[n], mani_plt_y[n],n)
            
            plt.draw()
            plt.pause(dt)  # グラフの更新を表示するための待機時間



            z = 1
        elif judge == 0:
            angle = angle
            # time.sleep(100)
            pass

        if fail_count == 1:
            plt.text(L *(n * 0.7), L *(n * 1.1)+2, "angle_fixed", fontsize=18, color="blue")
        
    
        
        # リンクの更新
        plt_x = np.concatenate([np.array([initial_position[0]]), x, np.array([end_eff_x])])
        plt_y = np.concatenate([np.array([initial_position[1]]), y, np.array([end_eff_y])])
        # print("plt_x:",plt_x)
        # print("plt_y:",plt_y)
        for i in range(n):
            ax.plot([plt_x[i],plt_x[i+1]],[plt_y[i],plt_y[i+1]],color="blue",marker="o",linestyle="-")
            plt.gca().set_aspect('equal', adjustable='box')
        # ゴール座標の表示
        ax.plot(gole_position[0],gole_position[1],color="red",marker="o")

        #計算されたマニピュレータのリンクの描画
        for i in range(n):
            ax.plot([final_plt_x[i],final_plt_x[i+1]],[final_plt_y[i],final_plt_y[i+1]],color="red",marker="o",linestyle="-")
            plt.gca().set_aspect('equal', adjustable='box')

        #実際のマニピュレータのリンクの描画
        for i in range(n):
            ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
            plt.gca().set_aspect('equal', adjustable='box')

        plt.draw()
        plt.pause(dt)  # グラフの更新を表示するための待機時間

        if z == 1 and count > 1:
            x_vec, dL, angle, fail_count = runge_kutta_n(L, K, D, n, theta_0, x_vec, dt,plt_x,plt_y, judge, t, fail_count,angle)
            z =0
        
        # データの保存
        dL_data.append([count * dt] + [dL[i] for i in range(n)])
        vce_data.append([count * dt] + [x_vec[i] for i in range(4*(n-1))])
        angle_data.append([count * dt] + [np.rad2deg(fail_angle[i]) for i in range(n)])
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
        print("max_dL:",max_dL)
        if count > 1:  # count が 1 以上の場合に条件を満たすように変更
            # ばねの運動が収束したら、ループを抜ける
            
            if max_dL <spring_len_judg:
                # print("ばねの運動が収束しました。")
                # plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                judge =1

                        #ゴールの判定
                if np.all(gole_position[0] - ran <= mani_x[n-1]) and np.all(mani_x[n-1] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= mani_y[n-1]) and np.all(mani_y[n-1] <= gole_position[1] + ran):
                     # グラフの初期化
                    ax.clear()
                    plt.text(-L *(n * 0.1), L *(n * 1.1)+2, "fin", fontsize=20, color="red")                    
                    print("ゴールしました。")
                    #リンクの描画
                    for i in range(n):
                        ax.plot([mani_plt_x[i],mani_plt_x[i+1]],[mani_plt_y[i],mani_plt_y[i+1]],color="black",marker="o",linestyle="-")
                        plt.gca().set_aspect('equal', adjustable='box')
                    ax.set_xlim(-L *(n * 0.1),L *(n * 1.1))  # x軸の範囲を設定
                    ax.set_ylim(-L *(n * 0.1),L *(n * 1.1))  # y軸の範囲を設定
                    plt.draw()
                    plt.pause(dt)  # グラフの更新を表示するための待機時間
                    plt.waitforbuttonpress()  # スペースキーが押されるまで待機
                    break
                # break
            elif max_dL >= L * 10:
                print("ばねの運動が収束していません。")
                break
            else:
                judge = 0
            
                # if np.all(gole_position[0] - ran <= plt_x[n-1]) and np.all(plt_x[n-1] <= gole_position[0] + ran) and np.all(gole_position[1] - ran <= plt_y[n-1]) and np.all(plt_y[n-1] <= gole_position[1] + ran):
                #     judge = 1
                #     return plt_x[n-1], plt_y[n-1], judge
                # else:
                #     judge = 0
        # plt.draw()
        # plt.pause(dt)  # グラフの更新を表示するための待機時間
                


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
    mov_x = np.zeros(n-1)
    mov_y = np.zeros(n-1)
    for i in range(n-1):
        mov_x[i] = x[i+1]
        mov_y[i] = y[i+1]
    
    return mov_x, mov_y

if __name__ == '__main__':
    #パラメータ設定
    n = 10 #リンク数
    L = 10 #リンクの長さ
    ran = 1 #目標位置の許容範囲
    theta_0 = 80 #初期角度
    theta_n = 60 #目標角度
    initial_position = [0,0] #初期位置
    # gole_position = [10,15] #目標位置
    # gole_position = [5,50] #目標位置 n=6
    # gole_position = [35,35] #目標位置
    # gole_position = [30,20 ] #目標位置
    gole_position = [60,50] #目標位置 n=10
    spring_len_judge = 0.03 #ばねの収束判定範囲

    # 物理定数
    m_pa = 1.0
    k_pa = 10
    D_pa = 10


    # 時間変数
    tmin = 0.0
    tmax = 1000.0
    dt = 0.03
    t = tmin

    # 現在の日付と時間を取得
    current_datetime = datetime.datetime.now()

    # フォーマットを指定して日付と時間を文字列に変換
    formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

    # ファイル名に日付と時間を追加
    # # ノートPC用
    # file_dL = fr"C:\Users\81808\Documents\GitHub\study_B4\spring\deta\dL\spring-fail-dL-{formatted_datetime}.csv"
    # file_angle = fr"C:\Users\81808\Documents\GitHub\study_B4\spring\deta\angle\spring-fail-angle-{formatted_datetime}.csv"
    # file_vec = fr"C:\Users\81808\Documents\GitHub\study_B4\spring\deta\vec\spring2-fail-vec-{formatted_datetime}.csv"
    # # デスクトップPC用
    # file_dL = fr"C:\Users\rsait\OneDrive - 東京理科大学\デスクトップ\git\study_B4\spring\deta\dL\spring-dL-{formatted_datetime}.csv"
    # file_angle = fr"C:\Users\rsait\OneDrive - 東京理科大学\デスクトップ\git\study_B4\spring\deta\angle\spring-angle-{formatted_datetime}.csv"
    # file_vec = fr"C:\Users\rsait\OneDrive - 東京理科大学\デスクトップ\git\study_B4\spring\deta\vec\spring-vec-{formatted_datetime}.csv"   
    # # (fr'./data/position/success_data_link{k}.txt')
    # データの保存用リスト    
    dL_data = []
    vce_data = []
    angle_data = []

    # 初期条件
    initial_conditions = f"n={n}, L={L}, theta_0={theta_0}, theta_n={theta_n}, initial_position={initial_position}, gole_position={gole_position}, spring_len_judge={spring_len_judge}, m_pa={m_pa}, k_pa={k_pa}, D_pa={D_pa}, tmin={tmin}, tmax={tmax}, dt={dt}"

    # ファイルに書き込むデータを作成
    data_to_write = f"date:{formatted_datetime}\nParameter:{initial_conditions}\n\n"

    # # # ファイルに書き込み
    # # file_path = r"C:\Users\81808\Documents\GitHub\study_B4\spring\deta\data-fail-1.txt"  #ノートPC用
    # file_path = r"C:\Users\rsait\OneDrive - 東京理科大学\デスクトップ\git\study_B4\spring\deta\data-1.txt" #デスクトップPC用
    # with open(file_path, 'a') as file:
    #     file.write(data_to_write)
    # print("初期条件:",initial_conditions)
    first_pos_x, first_pos_y = first_position(n , L ,initial_position, theta_0)
    #print("first_pos_x:",first_pos_x)
    #print("first_pos_y:",first_pos_y)
    
    mov_x, mov_y = move_posi(first_pos_x,first_pos_y)
    #print("mov_x:",mov_x)
    
    # 数値解用変数
    x_vec_n = np.concatenate([mov_x, mov_y, np.zeros(n-1),np.zeros(n-1)])  #[xの各座標、yの各座標、vx,vy]
    #print("x_vec0:",x_vec_n)

    #シミュレーション
    new_posi_spring_n(tmax, dt, t, x_vec_n, theta_0, gole_position,n, L, k_pa, D_pa, initial_position,first_pos_x,first_pos_y, spring_len_judge)