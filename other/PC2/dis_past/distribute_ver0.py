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

def joint_processor(theta_negative,theta_positive,pos_negative,pos_positive,v_negative,v_positive,L,plt_x,plt_y,theta_self,pos_self):
    # Calculate dL
    # spring_leng = (math.sqrt((pos_positive[0]-pos_negative[0])**2 
    #                          + (pos_positive[1]-pos_negative[1])**2))
    # dL = spring_leng - L

    # Calculate dL_negative
    spring_leng = (math.sqrt((pos_positive[0]-pos_negative[0])**2 
                             + (pos_positive[1]-pos_negative[1])**2))
    dL = spring_leng - L

    # Calculate angle
    angle_rad = math.atan2(pos_positive[1]-pos_negative[1],
                            pos_positive[0]-pos_negative[0])
    angle = math.degrees(angle_rad)

    # Calculate force with spring
    
    

    # Test print
    print("theta_negative", theta_negative)
    print("theta_positive", theta_positive)
    print("pos_negative", pos_negative)
    print("pos_positive", pos_positive)
    print("spring_leng", spring_leng)
    print("dL", dL)
    print("angle", angle)
    print("v_negative", v_negative)
    print("v_positive", v_positive)
    # time.sleep(100)




def f_n(x_vec,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t, count,fail_count):
    # Set parameters
    fail_time = 5 


    


def runge_kutta(x_vec,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t, count,fail_count):
    #calculate runge kutta
    print("runge_kutta")
    k1, dL, angle,fail_count= f_n(x_vec,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t, count,fail_count)
    k2, dL, angle,fail_count= f_n(x_vec + dt/2*k1,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t, count, fail_count)
    k3, dL, angle,fail_count= f_n(x_vec + dt/2*k2,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t, count, fail_count)
    k4, dL, angle,fail_count= f_n(x_vec + dt*k3,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t, count, fail_count)
    return x_vec + dt/6*(k1 + 2*k2 + 2*k3 + k4), dL, angle, fail_count



def run(file_path, m, L, g, k, b, dt, joint_num, goal_pos):
    # Set parameters
    ran = 0.1 # Gole position range
    ini_theta = 70 # initial angle
    ini_pos = [0, 0] # initial position
    spring_range = 0.1 # spring range
    t = 0 # time
    tmax = 10 # max time

    # Set glaph parameters
    fig, ax = plt.subplots()
    plt_x = np.zeros(joint_num+1)
    plt_y = np.zeros(joint_num+1)
    plt_x[0] = ini_pos[0]
    plt_y[0] = ini_pos[1]
    ax.plot(goal_pos[0], goal_pos[1], color='red', marker='o')
    ax.set_xlim(-L*(joint_num*0.1), L*(joint_num*1.1))
    ax.set_ylim(-L*(joint_num*0.1), L*(joint_num*1.1))
    ax.grid(True)
        
   # Set joint parameters
    theta = np.zeros(joint_num)
    theta[0] = np.deg2rad(ini_theta)

    # Calculate each joint angle
    for i in range(joint_num-1):
        theta[i+1] = theta[i] + np.deg2rad(-ini_theta/joint_num)

    # Calculate each joint position
    for i in range(joint_num):
        plt_x[i+1] = plt_x[i] + L*math.cos(theta[i])
        plt_y[i+1] = plt_y[i] + L*math.sin(theta[i])
        

    # Calculate initail EE position
    EE_x = plt_x[joint_num]
    EE_y = plt_y[joint_num]

    # Calculate target position (distripute)
    dis_range = 5
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

    # Test plot
    # ax.plot(plt_x, plt_y, marker='o')
    # plt.show()

    # time.sleep(1000)

    # Simulation
    for i in range(dis_range): 
        # Set target position
        EE_x = target_pos_x[i]
        EE_y = target_pos_y[i]

        while t < tmax:
            print("t",t)
            t += dt

            print(target_pos_x[count], target_pos_y[count])
            # time.sleep(100)

            # Calculate each joint position(test)
            test_joint_num = 4
            theta_negative = theta[test_joint_num - 1]
            theta_self = theta[test_joint_num]
            # theta_positive = theta[test_joint_num + 1]
            pos_negative = [plt_x[test_joint_num - 1], plt_y[test_joint_num -1]]
            pos_self = [plt_x[test_joint_num], plt_y[test_joint_num]]
            pos_positive = [plt_x[test_joint_num + 1], plt_y[test_joint_num + 1]]


            # Calculate each joint position
            theta_negative = theta[joint_num-2]
            theta_positive = theta[joint_num-1]
            pos_negative = [plt_x[joint_num-1], plt_y[joint_num-1]]
            pos_positive = [plt_x[joint_num], plt_y[joint_num]]
            v_negative = 0
            v_positive = 0

            joint_processor(theta_negative,theta_positive,pos_negative,
                            pos_positive,v_negative,v_positive,L,plt_x[1],plt_y[1],theta_self,pos_self)
            



        # Calculate each joint angle
        # x_vec, dL, angle, fail_count = runge_kutta(x_vec,theta, m, L, g, k, b, dt, joint_num, plt_x, plt_y, t ,count,fail_count)




if __name__ == '__main__':
    #################### Set directory path ####################
    now = datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    day = now.strftime("%d")
    time_str = now.strftime("%H%M%S")
    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../../result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    # csv_file_path = os.path.join(dir_path, "angle.csv")
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)


    #################### Set joint parameters ####################
    m = 1
    L = 10
    g = 9.8
    k = 100
    b = 10
    dt = 0.01
    joint_num = 10
    goal_pos = [0,70]


    #################### run simulation ####################
    print("run simulation")
    run(dir_path, m, L, g, k, b, dt, joint_num, goal_pos)




    # # Set broken joint
    # broken_joint = [3,5]

    # #write file
    # porpose_write = f"{len(broken_joint)} joints are broken.\n {broken_joint} is fixed\n"
    # date_time_str = now.strftime("%Y/%m/%d %H:%M:%S")
    # now_dir_path = os.getcwd()
    # run_file_name = os.path.basename(__file__)
    # date_write = f"date: {date_time_str}\n"

    