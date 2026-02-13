import spring_angle_test as sim_test
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm
from multiprocessing import Pool, cpu_count


#パラメータ設定
file_path = "success_rate.txt"
#csvファイルの初期化
with open(file_path, mode='w') as f:
    f.write("")

min = 2
max = 26
p = Pool(processes=cpu_count()-1)
print(f"cpu_count:{cpu_count()-1}")
print("-----------------------------start-----------------------------")

with tqdm(total=max - min) as pbar:
    for i, _ in enumerate(p.imap_unordered(sim_test.run, ((range(min, max))))):
        pbar.update()