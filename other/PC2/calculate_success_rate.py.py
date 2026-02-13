from collections import defaultdict
import matplotlib.pyplot as plt
import os
from datetime import datetime
import time
import itertools

def one_broken_joint():
    #  データを格納するリスト
    data = []
    all_success_rate = 0
    count = 0
    joint_combinations = list(itertools.combinations(range(10), 1))
    for i in range(len(joint_combinations)):
        # Set broken joint
        broken_joint = list(joint_combinations[i])
        # Set dire path
        one_dir_path = os.path.join(dir_path, "1_joint_broken", f"joint_{broken_joint[0]}")
        # os.makedirs(one_dir_path, exist_ok=True)
        os.chdir(one_dir_path)
        file_path = os.path.join(one_dir_path, 'success_rate.txt')
        print(f'File path: {file_path}')
        # ファイルの読み込み
        with open(file_path, 'r') as file:
            for line in file:
                    if 'success_rate' in line:
                        count += 1
                        success_rate = line.split(':')[1].strip()
                        all_success_rate += float(success_rate)
                        print(f'Success rate: {success_rate}')
                        print(f'Count: {count}')
                        print(f'All success rate: {all_success_rate}')
    total_success_rate = all_success_rate / count
    return total_success_rate

def two_broken_joint():
    #  データを格納するリスト
    data = []
    all_success_rate = 0
    count = 0
    joint_combinations = list(itertools.combinations(range(10), 2))
    for i in range(len(joint_combinations)):
        # Set broken joint
        broken_joint = list(joint_combinations[i])
        # Set dire path
        two_dir_path = os.path.join(dir_path, "2_joint_broken", f"joint_{broken_joint[0]}_{broken_joint[1]}")
        # os.makedirs(two_dir_path, exist_ok=True)
        os.chdir(two_dir_path)
        file_path = os.path.join(two_dir_path, 'success_rate.txt')
        print(f'File path: {file_path}')
        # ファイルの読み込み
        with open(file_path, 'r') as file:
            for line in file:
                    if 'success_rate' in line:
                        count += 1
                        success_rate = line.split(':')[1].strip()
                        all_success_rate += float(success_rate)
                        print(f'Success rate: {success_rate}')
                        print(f'Count: {count}')
                        print(f'All success rate: {all_success_rate}')
    total_success_rate = all_success_rate / count
    return total_success_rate

def three_broken_joint():
    #  データを格納するリスト
    data = []
    all_success_rate = 0
    count = 0
    joint_combinations = list(itertools.combinations(range(10), 3))
    for i in range(len(joint_combinations)):
        # Set broken joint
        broken_joint = list(joint_combinations[i])
        # Set dire path
        three_dir_path = os.path.join(dir_path, "3_joint_broken", f"joint_{broken_joint[0]}_{broken_joint[1]}_{broken_joint[2]}")
        # os.makedirs(three_dir_path, exist_ok=True)
        os.chdir(three_dir_path)
        file_path = os.path.join(three_dir_path, 'success_rate.txt')
        print(f'File path: {file_path}')
        # ファイルの読み込み
        with open(file_path, 'r') as file:
            for line in file:
                    if 'success_rate' in line:
                        count += 1
                        success_rate = line.split(':')[1].strip()
                        all_success_rate += float(success_rate)
                        print(f'Success rate: {success_rate}')
                        print(f'Count: {count}')
                        print(f'All success rate: {all_success_rate}')
    total_success_rate = all_success_rate / count
    return total_success_rate


def four_broken_joint():
    #  データを格納するリスト
    data = []
    all_success_rate = 0
    count = 0
    joint_combinations = list(itertools.combinations(range(10), 4))
    for i in range(len(joint_combinations)):
        # Set broken joint
        broken_joint = list(joint_combinations[i])
        # Set dire path
        four_dir_path = os.path.join(dir_path, "4_joint_broken", f"joint_{broken_joint[0]}_{broken_joint[1]}_{broken_joint[2]}_{broken_joint[3]}")
        # os.makedirs(four_dir_path, exist_ok=True)
        os.chdir(four_dir_path)
        file_path = os.path.join(four_dir_path, 'success_rate.txt')
        # print(f'File path: {file_path}')
        # ファイルの読み込み
        with open(file_path, 'r') as file:
            for line in file:
                    if 'success_rate' in line:
                        count += 1
                        success_rate = line.split(':')[1].strip()
                        all_success_rate += float(success_rate)
                        print(f'Success rate: {success_rate}')
                        print(f'Count: {count}')
                        print(f'All success rate: {all_success_rate}')
    total_success_rate = all_success_rate / count
    return total_success_rate


def five_broken_joint():
    #  データを格納するリスト
    data = []
    all_success_rate = 0
    count = 0
    joint_combinations = list(itertools.combinations(range(10), 5))
    for i in range(len(joint_combinations)):
        # Set broken joint
        broken_joint = list(joint_combinations[i])
        # Set dire path
        five_dir_path = os.path.join(dir_path, "5_joint_broken", f"joint_{broken_joint[0]}_{broken_joint[1]}_{broken_joint[2]}_{broken_joint[3]}_{broken_joint[4]}")
        # os.makedirs(five_dir_path, exist_ok=True)
        os.chdir(five_dir_path)
        file_path = os.path.join(five_dir_path, 'success_rate.txt')
        print(f'File path: {file_path}')
        # ファイルの読み込み
        with open(file_path, 'r') as file:
            for line in file:
                    if 'success_rate' in line:
                        count += 1
                        success_rate = line.split(':')[1].strip()
                        all_success_rate += float(success_rate)
                        print(f'Success rate: {success_rate}')
                        print(f'Count: {count}')
                        print(f'All success rate: {all_success_rate}')
    total_success_rate = all_success_rate / count
    return total_success_rate

        

if __name__ == '__main__':
    # Set matplotlib warning
    plt.rcParams['figure.max_open_warning'] = 0

    # Set the path to the result directory
    now = datetime.now()
    year = now.strftime("2024")
    month = now.strftime("09")
    day = now.strftime("18")
    time_str = now.strftime("132750")

    base_dir = os.path.abspath(os.path.join(os.getcwd(), "result"))
    dir_path = os.path.join(base_dir, year, month, day, time_str)
    os.chdir(dir_path) 

    print(f"Directory path: {dir_path}")

    # total_success_rate = one_broken_joint()
    # total_success_rate = two_broken_joint()
    # total_success_rate = three_broken_joint()
    total_success_rate = four_broken_joint()
    # total_success_rate = five_broken_joint()
    print(f'Total success rate: {total_success_rate}')
