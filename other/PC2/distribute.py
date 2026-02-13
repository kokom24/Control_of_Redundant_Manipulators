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

def joint_processor(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re):
    # Extract current state
    pos_self = np.array([re[0], re[1]])
    v_self = np.array([re[2], re[3]])

    # Compute spring vector to positive joint
    vec_self_to_pos = np.array(pos_positive) - pos_self
    length_self_to_pos = np.linalg.norm(vec_self_to_pos)
    direction_self_to_pos = vec_self_to_pos / length_self_to_pos if length_self_to_pos != 0 else np.array([0.0, 0.0])
    dL_self = length_self_to_pos - L

    # Compute spring vector to negative joint
    vec_neg_to_self = pos_self - np.array(pos_negative)
    length_neg_to_self = np.linalg.norm(vec_neg_to_self)
    direction_neg_to_self = vec_neg_to_self / length_neg_to_self if length_neg_to_self != 0 else np.array([0.0, 0.0])
    dL_negative = length_neg_to_self - L

    # Spring force
    F_spring_self = k * dL_self * direction_self_to_pos
    F_spring_negative = k * dL_negative * direction_neg_to_self

    # Damping force (projected relative velocity along the spring direction)
    v_rel_self = v_self - np.array(v_positive)
    v_rel_negative = v_self - np.array(v_negative)
    F_damp_self = -d * np.dot(v_rel_self, direction_self_to_pos) * direction_self_to_pos
    F_damp_negative = -d * np.dot(v_rel_negative, direction_neg_to_self) * direction_neg_to_self

    # Net force and acceleration
    F_total = F_spring_self - F_spring_negative + F_damp_self - F_damp_negative
    ax, ay = F_total[0], F_total[1]

    # Return state derivative: [vx, vy, ax, ay]
    result = np.array([v_self[0], v_self[1], ax, ay])

    # Compute angle for visualization/debug
    theta_self = math.atan2(direction_self_to_pos[1], direction_self_to_pos[0])
    angle = math.degrees(theta_self)

    # Print debug information
    print(f"pos_self: {pos_self}, v_self: {v_self}, dL_self: {dL_self}, F_spring_self: {F_spring_self}, F_damp_self: {F_damp_self}")
    print(f"F_total: {F_total}, ax: {ax}, ay: {ay}, angle: {angle}")
    print(f"result: {result}")

    return result, dL_self, angle


def runge_kutta(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re):
    # 4th-order Runge-Kutta integration
    print("Runge-Kutta integration")
    k1, dL1, angle1 = joint_processor(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re)
    k2, dL2, angle2 = joint_processor(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re + 0.5 * dt * k1)
    k3, dL3, angle3 = joint_processor(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re + 0.5 * dt * k2)
    k4, dL4, angle4 = joint_processor(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re + dt * k3)

    # Weighted average of slopes
    next_state = re + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    return next_state, dL1, angle1



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
    dis_range = 100
    target_pos_x,target_pos_y = new_target_pos(EE_x, EE_y, goal_pos[0], goal_pos[1], dis_range)

    # Variable initialization
    goal_x = goal_pos[0]
    goal_y = goal_pos[1]
    count = 0
    judge = 1
    fail_count = 0
    x_vec = EE_x

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
    for i in range(dis_range): 
        # Set target position
        t = 0
        EE_x = target_pos_x[i]
        EE_y = target_pos_y[i]
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

            print("target_pos_x", target_pos_x[i])
            print("target_pos_y", target_pos_y[i])
            # time.sleep(100)

            # Set parameters
            test_joint_num = 10
            theta_negative = theta[test_joint_num - 1]
            theta_self = theta[test_joint_num]
            pos_negative = [plt_x[test_joint_num - 1], plt_y[test_joint_num -1]]
            v_positive = [0, 0]
            pos_positive = [plt_x[test_joint_num + 1], plt_y[test_joint_num + 1]]
            v_negative = [0, 0]

            if t == 0:
                # Calculate each joint position(test)
                pos_self = [plt_x[test_joint_num], plt_y[test_joint_num]]
                v_self = [0, 0]
                # set re
                re = [pos_self[0], pos_self[1], v_self[0], v_self[1]]       

            else:
                pass              

            # joint_processor
            re, dL_self, angle = runge_kutta(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, plt_x, plt_y, theta_self, re)

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
    g = 9.8
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

    