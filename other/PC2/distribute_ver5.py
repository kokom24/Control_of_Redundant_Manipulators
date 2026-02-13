from collections import defaultdict
import matplotlib.pyplot as plt
import os
from datetime import datetime
import time
import itertools
import numpy as np
import math
import multiprocessing
from multiprocessing import Process, Queue, Pipe

def new_target_pos(EE_x, EE_y, goal_x, goal_y, dis_range):
    # Calculate target position (distripute)
    new_pos_x = np.zeros(dis_range + 1)
    new_pos_y = np.zeros(dis_range + 1)
    for i in range(dis_range + 1):
        new_pos_x[i] = EE_x + (goal_x - EE_x) * i / dis_range
        new_pos_y[i] = EE_y + (goal_y - EE_y) * i / dis_range
    return new_pos_x[1:], new_pos_y[1:]

def joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self,re,joint_n, k, d,v_positive):
    # Set parameters
    pos_self = [re[0], re[1]]
    v_self = [re[2], re[3]]

    # Calculate dL
    dx = pos_positive[0]-pos_self[0]
    dy = pos_positive[1]-pos_self[1]
    spring_leng = math.hypot(dx, dy)
    
    if spring_leng < 1e-8:
        # 完全に同じ座標の場合
        theta_self = 0
        sin_angle = 0
        cos_angle = 1
        dL_self = -L
    else:
        dL_self = spring_leng - L

        if pos_positive[1] >= pos_self[1] and pos_positive[0] > pos_self[0]:
            theta_self = math.atan2(pos_positive[1]-pos_self[1],
                                    pos_positive[0]-pos_self[0])
            sin_angle = math.sin(theta_self)
            cos_angle = math.cos(theta_self)
        elif pos_positive[1] >= pos_self[1] and pos_positive[0] < pos_self[0]:
            theta_self = (np.pi) - math.atan2(pos_self[1]-pos_positive[1],
                                    pos_self[0]-pos_positive[0])
            sin_angle = math.sin(theta_self)
            cos_angle = -1 * math.cos(theta_self)
        elif pos_positive[1] <= pos_self[1] and pos_positive[0] > pos_self[0]:
            theta_self = -1 * math.atan2(pos_self[1]-pos_positive[1],
                                    pos_self[0]-pos_positive[0])
            sin_angle = -1 * math.sin(theta_self)
            cos_angle = math.cos(theta_self)
        elif pos_positive[1] <= pos_self[1] and pos_positive[0] < pos_self[0]:
            theta_self = (np.pi) + math.atan2(pos_positive[1]-pos_self[1],
                                    pos_positive[0]-pos_self[0])
            sin_angle = -1 * math.sin(theta_self)
            cos_angle = -1 * math.cos(theta_self)
        elif pos_positive[1] == pos_self[1] and pos_positive[0] > pos_self[0]:
            theta_self = 0
            sin_angle = 0
            cos_angle = 1
        elif pos_positive[1] == pos_self[1] and pos_positive[0] < pos_self[0]:
            theta_self = np.pi
            sin_angle = 0
            cos_angle = -1
        elif pos_positive[1] >= pos_self[1] and pos_positive[0] == pos_self[0]:
            theta_self = np.pi/2
            sin_angle = 1
            cos_angle = 0
        elif pos_positive[1] <= pos_self[1] and pos_positive[0] == pos_self[0]:
            theta_self = -np.pi/2
            sin_angle = -1
            cos_angle = 0
        else:
            # 最後の保険
            theta_self = 0
            sin_angle = 0
            cos_angle = 1
            print("Warning: pos_positive and pos_self are the same point. Using default angle.")
    
    # Calculate dL_negative
    if joint_n == 0:
        dL_negative = 0
    else:
        spring_leng_ne = math.hypot(pos_self[0]-pos_negative[0],
                                    pos_self[1]-pos_negative[1])
        dL_negative = spring_leng_ne - L

    angle = theta_self

    # Calculate force with spring
    fx = k*(dL_self*cos_angle)
    fx_negative = k*(dL_negative*math.cos(theta_negative))
    fy = k*(dL_self*sin_angle)
    fy_negative = k*(dL_negative*math.sin(theta_negative))

    # Clip forces to prevent overflow
    # fx = np.clip(fx, -30, 30)
    # fy = np.clip(fy, -30, 30)
    # fx_negative = np.clip(fx_negative, -30, 30)
    # fy_negative = np.clip(fy_negative, -30, 30)
    # v_self = np.clip(v_self, -30, 30)
    # v_positive = np.clip(v_positive, -30, 30)
    # v_negative = np.clip(v_negative, -30, 30)

    # Calculate acceleration
    ax = fx - fx_negative + d*v_positive[0] - d*v_self[0] - d*v_self[0] + d*v_negative[0]
    ay = fy - fy_negative + d*v_positive[1] - d*v_self[1] - d*v_self[1] + d*v_negative[1]


    re = np.array([v_self[0], v_self[1], ax, ay]) 

    return re, dL_self, angle



def runge_kutta(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, theta_self, re, joint_n , k, d, dt):
    #calculate runge kutta
    # print("runge_kutta")
    k1, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self,re,joint_n,k, d, v_positive)
    k2, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self, re + k1*dt/2, joint_n, k, d, v_positive)
    k3, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self, re + k2*dt/2, joint_n, k, d, v_positive)
    k4, dL_self, angle = joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self, re + k3*dt, joint_n ,k , d, v_positive)
    return re + dt/6*(k1 + 2*k2 + 2*k3 + k4), dL_self, angle

def joint_main(i,tmax, dt, L, theta, plt_x, plt_y, target_pos_x, target_pos_y, joint_n, ran, k, d,status, pipe_left, pipe_right, pipes, joint_num):
    # Set parameters
    t = 0 
    re_x_list = []  
    re_y_list = []  
    time_list = []
    re_vx_list = []
    re_vy_list = []
    re_ax_list = []
    re_ay_list = []
    dL_success = 0  # Initialize dL_success
    data_left = None  # Initialize data from left pipe
    data_right = None  # Initialize data from right pipe


    # Set parameters
    theta_negative = theta[joint_n - 1]
    theta_self = theta[joint_n]
    pos_negative = [plt_x[joint_n - 1], plt_y[joint_n -1]]
    v_positive = [0, 0]
    pos_positive = [plt_x[joint_n + 1 ], plt_y[joint_n + 1 ]]
    # print("plt_x", plt_x)
    # print("plt_y", plt_y)
    v_negative = [0, 0]
    re_pos_negative = pos_negative
    re_pos_positive = pos_positive
    re_theta_negative = theta_negative
    re_theta_positive = theta_self
    re_v_negative = v_negative
    re_v_positive = v_positive
    prev_data_left = None
    prev_data_right = None



    while t < tmax:
        # Set pipe parameters
        if joint_n > 0:
            pipe_left = pipes[joint_n - 1][1]  # Get the pipe to the left joint
        if joint_n < joint_num - 1:
            pipe_right = pipes[joint_n][0]     # Get the pipe to the right joint
  
        # Receive data from pipes
        if pipe_left is not None:
            if pipe_left.poll(timeout=dt/10):
                try:
                    data_left = pipe_left.recv()
                    prev_data_left = data_left
                except EOFError:
                    print(...)
            else:
                data_left = prev_data_left
        else:
            data_left = prev_data_left

        
        if pipe_right is not None:
            if pipe_right.poll(timeout=dt/10):
                try:
                    data_right = pipe_right.recv()
                    prev_data_right = data_right
                except EOFError:
                    print(...)
            else:
                data_right = prev_data_right
        else:
            data_right = prev_data_right


        # if pipe_right is not None and pipe_right.poll(timeout=dt/10):
        #     data_right = pipe_right.recv()

        # Update status
        status_from_left = data_left[5:] if data_left is not None else None
        status_from_right = data_right[5:] if data_right is not None else None
        status = status_from_left if status_from_left is not None else status
        if status_from_left is not None:
            status[joint_n-1:] = status_from_left[joint_n-1:]
        if status_from_right is not None:
            status[joint_n + 1:] = status_from_right[joint_n + 1:]  

        # Update joint position
        theta_negative = data_left[0] if data_left is not None else re_theta_negative
        theta_positive = data_right[0] if data_right is not None else re_theta_positive
        v_negative = [data_left[1], data_left[2]] if data_left is not None else re_v_negative
        v_positive = [data_right[1], data_right[2]] if data_right is not None else re_v_positive
        pos_negative = [data_left[3], data_left[4]] if data_left is not None else re_pos_negative
        pos_positive = [data_right[3], data_right[4]] if data_right is not None else re_pos_positive
        status[joint_n] = 0


        # write data to txt
        with open('joint_data' + str(joint_n) + '.txt', 'a') as f:
            if t == 0:
                f.write(f"Joint {joint_n + 1} : ({plt_x[joint_n]:.2f}, {plt_y[joint_n]:.2f})\n")
                f.write(f"status: {status}\n\n")
            elif t > 0:
                f.write(f"Joint {joint_n + 1} : dL_self: {dL_self:.3f}, angle: {angle:.2f}\n")
                f.write(f"pos_negative: ({pos_negative[0]:.2f}, {pos_negative[1]:.2f})\n")
                f.write(f"pos_self: ({x:.2f}, {y:.2f})\n")
                f.write(f"pos_positive: ({pos_positive[0]:.2f}, {pos_positive[1]:.2f})\n")
                f.write(f"theta_self: {theta_self:.2f}\n")
                f.write(f"v_self: ({vx:.2f}, {vy:.2f})\n")
                f.write(f"data_to_left: {data_to_left}\n")
                f.write(f"data_to_right: {data_to_right}\n")
                f.write(f"data_left: {data_left}\n")
                f.write(f"data_right: {data_right}\n")
                f.write(f"status: {status}\n\n")
            # f.write(f"Target Position: ({target_pos_x[i+1]:.2f}, {target_pos_y[i+1]:.2f})\n")


        if t == 0:
            # Calculate each joint position(test)
            pos_self = [plt_x[joint_n], plt_y[joint_n]]
            v_self = [0, 0]
            # set re
            re = [pos_self[0], pos_self[1], v_self[0], v_self[1]]     
            # print("re", re)  
            # print("pos_positive", pos_positive)

        else:
            pass              

        # joint_processo
        re, dL_self, angle = runge_kutta(theta_negative, pos_negative, pos_positive, v_negative, v_positive, L, theta_self, re, joint_n, k, d, dt)

        # Set result
        x = re[0]
        y = re[1]
        vx = re[2]
        vy = re[3]

        # Joint 1 
        if joint_n == 0:
            x =0
            y = 0
            re[0] = x
            re[1] = y


        # add result to list
        re_vx_list.append(vx)
        re_vy_list.append(vy)
        re_x_list.append(x)
        re_y_list.append(y)
        time_list.append(t)


        # if abs(vx) < ran and abs(vy) < ran and abs(dL_self) < ran:
        if abs(dL_self) < ran:
            dL_success = 1  # Set dL_success to 1 if joint is successful
            status[joint_n] = 1  # Set status to 1 if joint is successful

        if np.all(np.array(status) == 1):
            with open('joint_data' + str(joint_n) + '.txt', 'a') as f:
                f.write(f"Joint {joint_n + 1} is successful at time {t:.2f} seconds.\n")
                f.write(f"dL_self: {dL_self:.3f}, angle: {angle:.2f}\n")
                f.write(f"status: {status}\n")            
            
            dL_success = 2
            # else:
            #     pass

        # Update time
        t += dt
        # status[joint_n] = 0

        # set send data to parent process
        vector = [vx, vy]
        position = [x, y]
        give_parameters = [angle, vector[0], vector[1], position[0], position[1]]+ list(status)
        data_to_left = give_parameters
        data_to_right = give_parameters
        re_pos_negative = [pos_negative[0], pos_negative[1]]
        re_pos_positive = [pos_positive[0], pos_positive[1]]
        re_theta_negative = theta_negative
        re_theta_positive = theta_positive
        re_v_negative = v_negative
        re_v_positive = v_positive
        

        # send data to parent process
        if pipe_left is not None:
            try:
                pipe_left.send(data_to_left)
            except (BrokenPipeError, EOFError):
                pass
        
        if pipe_right is not None:
            try:
                pipe_right.send(data_to_right)
            except (BrokenPipeError, EOFError):
                pass 

        if dL_success == 2:
            break

    if t >= tmax:
        # print("-----------------joint_num", joint_n + 1, "is failed-----------------\n")
        with open('joint_data' + str(joint_n) + '.txt', 'a') as f:
            f.write(f"Joint {joint_n + 1} failed to converge within time limit.\n")
        status[joint_n] = -1
        print(f"Joint {joint_n + 1} failed to converge within time limit.")

    # Set list graph(re_x_list, re_y_list, time_list, re_vx_list, re_vy_list, re_ax_list, re_ay_list)
    list_graph = [re_x_list, re_y_list, time_list, re_vx_list, re_vy_list, re_ax_list, re_ay_list]

    # Set give parameters(theta, vector, position, status)
    vector = [vx, vy]
    position = [x, y]
    give_parameters = [angle, vector, position, status, pos_positive, dL_self]

    return_list = [list_graph, give_parameters]

    return return_list


def joint_main_process(queue, args, joint_id):
    list_graph, give_parameters = joint_main(*args)
    queue.put((joint_id, list_graph, give_parameters))

 

def run(file_path, m, L, g, k, b, dt, joint_num, goal_pos):
    # Set parameters
    ran = 0.05 # Gole position range
    ini_theta = 70 # initial angle
    ini_pos = [0, 0] # initial position
    spring_range = 0.1 # spring range
    status = np.zeros(joint_num) 
    t = 0 # time
    tmax =100 # max time

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
        
    print("Calculate each joint position", plt_x, plt_y)

    # Calculate initail EE position
    EE_x = plt_x[joint_num]
    EE_y = plt_y[joint_num]
    print("EE_x", EE_x, "EE_y", EE_y)

    # Calculate target position (distripute)
    dis_range = 3
    target_pos_x,target_pos_y = new_target_pos(EE_x, EE_y, goal_pos[0], goal_pos[1], dis_range)
    print("target_pos", target_pos_x, target_pos_y)

    # Variable initialization
    goal_x = goal_pos[0]
    goal_y = goal_pos[1]
    count = 0
    judge = 1
    fail_count = 0
    judge_success = 0

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
        print("dis_range", i+1)
        print("target_pos", EE_x, EE_y)
        # print("plt_x", plt_x)
        # print("plt_y", plt_y)

        # Set path for joint information of dis_range
        if (i+1) == 1:
            joint_data_path = os.path.join(file_path, f"dis_range_{i+1}")
            os.makedirs(joint_data_path, exist_ok=True)
            os.chdir(joint_data_path)
        else:
            os.path.abspath(os.path.join(os.getcwd(), "../"))
            joint_data_path = os.path.join(file_path, f"dis_range_{i+1}")
            os.makedirs(joint_data_path, exist_ok=True)
            os.chdir(joint_data_path)

        # Set multiprocessing Pipe and Process
        pipes = []
        start_times = {}  # Initialize start times for each joint

        # create a pipe and process for each joint
        for j in range(joint_num - 1):
            parent_conn, child_conn = Pipe()
            pipes.append((parent_conn, child_conn))
        
        # Create a queue for inter-process communication
        processes = []
        queue = Queue()
        for p in range(joint_num):
            joint_n = p
            pipe_left = pipes[p - 1][1] if p > 0 else None
            pipe_right = pipes[p][1] if p < joint_num - 1 else None
            args = (
                i, tmax, dt, L, theta, plt_x, plt_y,
                target_pos_x, target_pos_y, joint_n, ran, k, b,
                status, pipe_left, pipe_right, pipes, joint_num,
            )
            proc = Process(target=joint_main_process, args=(queue, args, joint_n))
            proc.start()
            processes.append(proc)

        # Wait for all processes to finish and collect results
        results = {}

        for _ in range(joint_num):
            joint_id, list_graph, give_parameters = queue.get()
            results[joint_id] = {
                "list_graph": list_graph,
                "give_parameters": give_parameters,
            }

        # Join all processes
        for proc in processes:
            proc.join()

        # Process results
        joint_positions = [results[joint_id]['give_parameters'][2] for joint_id in sorted(results.keys())]
        dL_list = [results[joint_id]['give_parameters'][5] for joint_id in sorted(results.keys())]

        # Print joint positions and distances
        for r, pos in enumerate(joint_positions):
            if r == 0:
                dL = 0.0
            else:
                prev = joint_positions[r - 1]
                dL = dL_list[r - 1]
                if dL <= ran:
                    judge_success += 1
                # If simulation is not successful, stop the simulation
                if pos[0] > 100 or pos[1] > 100 or pos[0] < -100 or pos[1] < -100 or dL > ran*10 or dL < -ran*10 :
                    print("\n************************************************************")
                    print(f"Joint {r + 1} is not successful. Stop simulation.")
                    print("************************************************************")
                    break
            print(f"Joint {r + 1} : {pos}, dL={dL:.3f}")

        ee = results[9]['give_parameters'][4]
        joint10 = joint_positions[-1]
        dx = ee[0] - EE_x
        dy = ee[1] - EE_y
        print(f"EE = {ee}, dx = {dx:.3f}, dy = {dy:.3f}")

        # Judge success or failure
        if judge_success == joint_num-1:
            print(f"All joints are successful.\n")
        else:
            print("\n************************************************************")
            print("simulation is not successful.")
            print("************************************************************")
            break

        # Rese parameters for the next iteration
        judge_success = 0
        status = np.zeros(joint_num)  # Reset status for the next iteration

        # set plt_x, plt_y
        for j in range(joint_num):
            plt_x[j] = results[j]['give_parameters'][2][0]
            plt_y[j] = results[j]['give_parameters'][2][1]
            # print(f"Joint {j + 1} position: ({plt_x[j]:.2f}, {plt_y[j]:.2f})")




        # Use multiprocessing to run joint_main in parallel for each joint]
        # for k in range(joint_num - 1):
        #     if pipes[i][0].poll():
        #         data = pipes[k][0].recv()

            # joint_id, list_graph, give_parameters = data

            # # Get the parameters from the queue
            # elapsed_time = time.time() - start_times[joint_id]
            # # print(f"Received from joint {joint_id + 1}: {give_parameters}")
            # print(f"Joint {joint_id + 1} finished in {elapsed_time:.2f} seconds")


    # re_x_list, re_y_list, time_list, re_vx_list, re_vy_list, re_ax_list, re_ay_list = list_graph=0




    # # X軸 vs 時間
    # plt.figure()
    # plt.plot(time_list, re_x_list, marker='o', linestyle='-')
    # plt.xlabel('Time [s]')
    # plt.ylabel('X position')
    # plt.title('X position vs Time')
    # plt.grid(True)
    # plt.savefig(os.path.join(dir_path, "x_vs_time.png"), dpi=300)
    # # plt.show()

    # # Y軸 vs 時間
    # plt.figure()
    # plt.plot(time_list, re_y_list, marker='o', linestyle='-')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Y position')
    # plt.title('Y position vs Time')
    # plt.grid(True)
    # plt.savefig(os.path.join(dir_path, "y_vs_time.png"), dpi=300)
    # # plt.show()

    # plt.figure()
    # plt.plot(time_list, re_vx_list, marker='o', linestyle='-')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Vx [m/s]')
    # plt.title('Vx vs Time')
    # plt.grid(True)
    # plt.savefig(os.path.join(dir_path, "vx_vs_time.png"), dpi=300)

    # plt.figure()
    # plt.plot(time_list, re_vy_list, marker='o', linestyle='-')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Vy [m/s]')
    # plt.title('Vy vs Time')
    # plt.grid(True)
    # plt.savefig(os.path.join(dir_path, "vy_vs_time.png"), dpi=300)



    # print("x_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "x_vs_time.png")))
    # print("y_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "y_vs_time.png")))
    # print("vx_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "vx_vs_time.png")))
    # print("vy_vs_time.png exists:", os.path.exists(os.path.join(dir_path, "vy_vs_time.png")))
    # print("Saved image directory:", dir_path)




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
    d = 8
    dt = 0.01
    joint_num = 10
    goal_pos = [40,60]


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

    
    

# def joint_processor(theta_negative,pos_negative,pos_positive,v_negative,L,theta_self,re,joint_n, k, d,v_positive):
#     # Set parameters
#     pos_self = [re[0], re[1]]
#     v_self = [re[2], re[3]]

#     # Calculate dL
#     spring_leng = (math.sqrt((pos_positive[0]-pos_self[0])**2
#                                 + (pos_positive[1]-pos_self[1])**2))
#     dL_self = spring_leng - L

#     # Calculate dL_negative
#     if joint_n == 0:
#         dL_negative = 0
#     else:
#         spring_leng_ne = (math.sqrt((pos_self[0]-pos_negative[0])**2 
#                                     + (pos_self[1]-pos_negative[1])**2))
#         dL_negative = spring_leng_ne - L
#         # print("dL_negative", dL_negative_ne)

#     # Calculate angle
#     if pos_positive[1] >=pos_self[1] and pos_positive[0] > pos_self[0]:
#         theta_self = math.atan2(pos_positive[1]-pos_self[1],
#                                 pos_positive[0]-pos_self[0])
#         sin_angle = math.sin(theta_self)
#         cos_angle = math.cos(theta_self)
#     elif pos_positive[1] >= pos_self[1] and pos_positive[0] < pos_self[0]:
#         theta_self = (np.pi) - math.atan2(pos_self[1]-pos_positive[1],
#                                 pos_self[0]-pos_positive[0])
#         sin_angle = math.sin(theta_self)
#         cos_angle = -1 * math.cos(theta_self)
#     elif pos_positive[1] <= pos_self[1] and pos_positive[0] > pos_self[0]:
#         theta_self = -1 * math.atan2(pos_self[1]-pos_positive[1],
#                                 pos_self[0]-pos_positive[0])
#         sin_angle = -1 * math.sin(theta_self)
#         cos_angle = math.cos(theta_self)
#     elif pos_positive[1] <= pos_self[1] and pos_positive[0] < pos_self[0]:
#         theta_self = (np.pi) + math.atan2(pos_positive[1]-pos_self[1],
#                                 pos_positive[0]-pos_self[0])
#         sin_angle = -1 * math.sin(theta_self)
#         cos_angle = -1 * math.cos(theta_self)
#     elif pos_positive[1] == pos_self[1] and pos_positive[0] > pos_self[0]:
#         theta_self = 0
#         sin_angle = 0
#         cos_angle = 1
#     elif pos_positive[1] == pos_self[1] and pos_positive[0] < pos_self[0]:
#         theta_self = np.pi
#         sin_angle = 0
#         cos_angle = -1
#     elif pos_positive[1] >= pos_self[1] and pos_positive[0] == pos_self[0]:
#         theta_self = np.pi/2
#         sin_angle = 1
#         cos_angle = 0
#     elif pos_positive[1] <= pos_self[1] and pos_positive[0] == pos_negative[0]:
#         theta_self = -np.pi/2
#         sin_angle = -1
#         cos_angle = 0
#     # else:
#     #     # This case should not happen, but just in case
#     #     theta_self = 0
#     #     sin_angle = -1
#     #     cos_angle = 1
#     #     print("Error: pos_positive and pos_self are the same point.")
#     # try:
#     #     cos_angle = cos_angle
#     #     sin_angle = sin_angle
#     #     theta_self = theta_self
#     # except:
#     #     cos_angle = 0.5
#     #     sin_angle = 0.5
#     #     theta_self = np.pi/2
#         # print("Error: pos_positive and pos_self are the same point. Using default angle.")


#     angle = theta_self

#     # theta_self = math.atan2(pos_positive[1]-pos_self[1],
#     #                         pos_positive[0]-pos_self[0])
#     # angle = math.degrees(theta_self)

#     # Exception Handling


#     # Calculate force with spring
#     fx = k*(dL_self*cos_angle)
#     fx_negative = k*(dL_negative*math.cos(theta_negative))
#     fy = k*(dL_self*sin_angle)
#     fy_negative = k*(dL_negative*math.sin(theta_negative))

#     ax = fx - fx_negative + d*v_positive[0] - d*v_self[0] - d*v_self[0] + d*v_negative[0]
#     ay = fy - fy_negative + d*v_positive[1] - d*v_self[1] - d*v_self[1] + d*v_negative[1]

#     # Update result
#     re = np.array([v_self[0], v_self[1], ax, ay]) 

#     # Test print
#     # print("theta_negative", theta_negative)
#     # print("pos_negative", pos_negative)
#     # print("pos_positive", pos_positive)
#     # print("spring_leng", spring_leng)
#     # print("pos_self", pos_self)
#     # print("v_negative", v_negative)
#     # print("v_self", v_self)
#     # print("fx", fx)
#     # print("fx_negative", fx_negative)
#     # print("fy", fy)
#     # print("fy_negative", fy_negative)
#     # print("ax", ax)
#     # print("ay", ay)
#     # print("dL", dL_self)
#     # print("re", re)
#     # time.sleep(100)

#     return re, dL_self, angle