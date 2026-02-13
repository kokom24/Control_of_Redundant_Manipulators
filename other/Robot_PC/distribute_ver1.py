from collections import defaultdict
import matplotlib.pyplot as plt
import os
from datetime import datetime
import time
import itertools
import numpy as np
import math

def new_target_pos(EE_x, EE_y, goal_x, goal_y, dis_range):
    # Calculate target position (distripute)
    new_pos_x = np.zeros(dis_range)
    new_pos_y = np.zeros(dis_range)
    for i in range(dis_range):
        new_pos_x[i] = EE_x + (goal_x - EE_x) * i / dis_range
        new_pos_y[i] = EE_y + (goal_y - EE_y) * i / dis_range

    return new_pos_x, new_pos_y

def joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self,re,joint_num):
    # Set parameters
    pos_self = [re[0], re[1]]
    v_self = [re[2], re[3]]

    # Calculate dL
    spring_leng = (math.sqrt((pos_positive[0]-pos_self[0])**2
                                + (pos_positive[1]-pos_self[1])**2))
    dL_self = spring_leng - L

    # Calculate dL_negative
    if joint_num == 1:
        dL_negative = 0
    else:
        spring_leng = (math.sqrt((pos_self[0]-pos_negative[0])**2 
                                    + (pos_self[1]-pos_negative[1])**2))
        dL_negative = spring_leng - L
        print("dL_negative", dL_negative)

    # Calculate angle
    theta_self = math.atan2(pos_positive[1]-pos_self[1],
                            pos_positive[0]-pos_self[0])
    angle = math.degrees(theta_self)

    # Calculate force with spring
    fx = k*(dL_self*math.cos(theta_self))
    fx_negative = k*(dL_negative*math.cos(theta_negative))
    fy = k*(dL_self*math.sin(theta_self))
    fy_negative = k*(dL_negative*math.sin(theta_negative))

    ax = fx - fx_negative + d*v_negative[0] - d*v_self[0]
    ay = fy - fy_negative + d*v_negative[1] - d*v_self[1]

    # Update result
    re = np.array([v_self[0], v_self[1], ax, ay]) 

    # Test print
    # print("theta_negative", theta_negative)
    # print("theta_positive", theta_positive)
    # print("pos_negative", pos_negative)
    # print("pos_positive", pos_positive)
    # print("spring_leng", spring_leng)
    # print("pos_self", pos_self)
    # print("dL", dL_self)
    # print("angle", angle)
    # print("v_negative", v_negative)
    # print("v_positive", v_positive)
    # print("v_self", v_self)
    # print("fx", fx)
    # print("fx_negative", fx_negative)
    # print("fy", fy)
    # print("fy_negative", fy_negative)
    print("ax", ax)
    print("ay", ay)
    print("re", re)
    # time.sleep(100)

    return re, dL_self, angle


def runge_kutta(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, theta_self, re, joint_num):
    #calculate runge kutta
    print("runge_kutta")
    k1, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self,re,joint_num)
    k2, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self, re + k1*dt/2, joint_num)
    k3, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self, re + k2*dt/2, joint_num)
    k4, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self, re + k3*dt, joint_num)
    return re + dt/6*(k1 + 2*k2 + 2*k3 + k4), dL_self, angle


def run(file_path, m, L, g, k, b, dt, joint_num, goal_pos):
    # Set parameters
    ran = 0.1 # Gole position range
    ini_theta = 70 # initial angle
    ini_pos = [0, 0] # initial position
    spring_range = 0.1 # spring range
    t = 0 # time
    tmax =5 # max time

    # Set glaph parameters
    fig, ax = plt.subplots()
    plt_x = np.zeros(joint_num + 2)
    plt_y = np.zeros(joint_num + 2)
    theta = np.zeros(joint_num + 2)
    plt_x[0] = ini_pos[0]
    plt_y[0] = ini_pos[1]
    ax.plot(goal_pos[0], goal_pos[1], color='red', marker='o')
    ax.set_xlim(-L*(joint_num*0.1), L*(joint_num*1.1))
    ax.set_ylim(-L*(joint_num*0.1), L*(joint_num*1.1))
    ax.grid(True)
    re = np.zeros(4)
        
   # Set joint parameters
    theta[0] = np.deg2rad(ini_theta)

    # Calculate each joint angle
    for i in range(joint_num-1):
        theta[i+1] = theta[i] + np.deg2rad(-ini_theta/joint_num)
    # theta[joint_num] = 0

    # Calculate each joint position
    for i in range(joint_num):
        plt_x[i+1] = plt_x[i] + L*math.cos(theta[i])
        plt_y[i+1] = plt_y[i] + L*math.sin(theta[i])
        

    # Calculate initail EE position
    EE_x = plt_x[joint_num]
    EE_y = plt_y[joint_num]

    # Calculate target position (distripute)
    dis_range = 2
    target_pos_x,target_pos_y = new_target_pos(EE_x, EE_y, goal_pos[0], goal_pos[1], dis_range)

    # Variable initialization
    goal_x = goal_pos[0]
    goal_y = goal_pos[1]
    count = 0
    judge = 1
    fail_count = 0

    # Test print
    # print("Calculate each joint position", plt_x, plt_y)
    # print("Calculate initail EE position", EE_x, EE_y)
    # print("goal_pos", goal_pos)
    # print("target_pos_x", target_pos_x)
    # print("target_pos_y", target_pos_y)
    # print("theta", theta)

    # # Test plot
    # ax.plot(plt_x, plt_y, marker='o')
    # plt.show()

    # time.sleep(1000)

    # Simulation
    for i in range(dis_range-1): 
        # Set target position
        t = 0
        EE_x = target_pos_x[i+1]
        EE_y = target_pos_y[i+1]
        plt_x[joint_num] = EE_x
        plt_y[joint_num] = EE_y

        # Initialize result lists
        re_x_list = []  
        re_y_list = []  
        time_list = []
        re_vx_list = []
        re_vy_list = []
        re_ax_list = []
        re_ay_list = []

        print("\n")
        print("dis_range", dis_range)

        while t < tmax:
            print("\n")
            print("t",t)

            print("target_pos_x", target_pos_x[i+1])
            print("target_pos_y", target_pos_y[i+1])
            # time.sleep(100)

            # Set parameters
            test_joint_num = 10 - 1
            theta_negative = theta[test_joint_num - 1]
            theta_self = theta[test_joint_num]
            pos_negative = [plt_x[test_joint_num - 1], plt_y[test_joint_num -1]]
            v_positive = [0, 0]
            pos_positive = [plt_x[test_joint_num + 1], plt_y[test_joint_num + 1]]
            # print("plt_x", plt_x)
            # print("plt_y", plt_y)
            v_negative = [0, 0]

            if t == 0:
                # Calculate each joint position(test)
                pos_self = [plt_x[test_joint_num], plt_y[test_joint_num]]
                v_self = [0, 0]
                # set re
                re = [pos_self[0], pos_self[1], v_self[0], v_self[1]]     
                # print("re", re)  
                # print("pos_positive", pos_positive)

            else:
                pass              

            # joint_processor
            joint_num = test_joint_num + 1
            re, dL_self, angle = runge_kutta(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, theta_self, re, joint_num)

            # Set result
            x = re[0]
            y = re[1]
            vx = re[2]
            vy = re[3]
            re_vx_list.append(vx)
            re_vy_list.append(vy)

            re_x_list.append(x)
            re_y_list.append(y)
            time_list.append(t)

            # Update time
            t += dt

    # X軸 vs 時間
    plt.figure()
    plt.plot(time_list, re_x_list, marker='o', linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('X position')
    plt.title('X position vs Time')
    plt.grid(True)
    plt.savefig(os.path.join(dir_path, "x_vs_time.png"), dpi=300)
    # plt.show()

    # Y軸 vs 時間
    plt.figure()
    plt.plot(time_list, re_y_list, marker='o', linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('Y position')
    plt.title('Y position vs Time')
    plt.grid(True)
    plt.savefig(os.path.join(dir_path, "y_vs_time.png"), dpi=300)
    # plt.show()

    plt.figure()
    plt.plot(time_list, re_vx_list, marker='o', linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('Vx [m/s]')
    plt.title('Vx vs Time')
    plt.grid(True)
    plt.savefig(os.path.join(dir_path, "vx_vs_time.png"), dpi=300)

    plt.figure()
    plt.plot(time_list, re_vy_list, marker='o', linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('Vy [m/s]')
    plt.title('Vy vs Time')
    plt.grid(True)
    plt.savefig(os.path.join(dir_path, "vy_vs_time.png"), dpi=300)



    print("x_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "x_vs_time.png")))
    print("y_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "y_vs_time.png")))
    print("vx_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "vx_vs_time.png")))
    print("vy_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "vy_vs_time.png")))
    print("Saved image directory:", dir_path)




        # Calculate each joint angle
        # x_vec, dL_self, angle, fail_count = runge_kutta(x_vec,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t ,count,fail_count)




if __name__ == '__main__':
    #################### Set directory path ####################
    now = datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    day = now.strftime("%d")
    time_str = now.strftime("%H%M%S")
    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    # csv_file_path = os.path.join(dir_path, "angle.csv")
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)




    #################### Set joint parameters ####################
    m = 1
    L = 10
    g = 0
    k = 100
    d = 10
    dt = 0.01
    joint_num = 10
    goal_pos = [60,40]


    #################### run simulation ####################
    print("run simulation")
    run(dir_path, m, L, g, k, d, dt, joint_num, goal_pos)




    # # Set broken joint
    # broken_joint = [3,5]

    # #write file
    # porpose_write = f"{len(broken_joint)} joints are broken.\n {broken_joint} is fixed\n"
    # date_time_str = now.strftime("%Y/%m/%d %H:%M:%S")
    # now_dir_path = os.getcwd()
    # run_file_name = os.path.basename(__file__)
    # date_write = f"date: {date_time_str}\n"

    