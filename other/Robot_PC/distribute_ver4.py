import multiprocessing as mp
import numpy as np
from tqdm import tqdm

# パラメータ
num_links = 10
dt = 0.01
k = 10
c = 1
m = 1
max_steps = 10000
ran = 1e-3  # 終了判定のしきい値

def compute_next_state(index, conn):
    x = 0.1 * index
    v = 0.0
    a = 0.0
    x_prev = x
    history = []

    for t in range(max_steps):
        # 初期ステップ
        if t == 0:
            conn.send((index, x, v, False))  # 収束していない
        else:
            left_x, left_v, right_x, right_v, terminate_flag = conn.recv()

            if terminate_flag:
                break  # 終了シグナルを受信

            # バネ・ダンパーの力計算
            force = 0
            if left_x is not None:
                force += -k * (x - left_x) - c * (v - left_v)
            if right_x is not None:
                force += -k * (x - right_x) - c * (v - right_v)

            a = force / m
            v += a * dt
            x += v * dt

            # 収束条件チェック
            dx = abs(x - x_prev)
            converged = abs(v) < ran and dx < ran
            x_prev = x

            # 状態と収束情報を送信
            conn.send((index, x, v, converged))

        history.append((x, v))

    conn.close()

def main():
    processes = []
    parent_conns = []

    for i in range(num_links):
        parent_conn, child_conn = mp.Pipe()
        process = mp.Process(target=compute_next_state, args=(i, child_conn))
        processes.append(process)
        parent_conns.append(parent_conn)

    for p in processes:
        p.start()

    all_results = [[] for _ in range(num_links)]
    terminate_all = False
    step = 0

    pbar = tqdm(total=max_steps)
    while not terminate_all and step < max_steps:
        # 状態を取得
        states = [conn.recv() for conn in parent_conns]  # (index, x, v, converged)
        states.sort()

        # 結果保存
        for i, (idx, x, v, _) in enumerate(states):
            all_results[i].append((x, v))

        # 収束チェック
        convergences = [item[3] for item in states]
        terminate_all = all(convergences)

        # 隣の情報を各プロセスに送信（+ 終了フラグ）
        for i in range(num_links):
            left_x, left_v = (states[i - 1][1], states[i - 1][2]) if i > 0 else (None, None)
            right_x, right_v = (states[i + 1][1], states[i + 1][2]) if i < num_links - 1 else (None, None)
            parent_conns[i].send((left_x, left_v, right_x, right_v, terminate_all))

        step += 1
        pbar.update(1)

    pbar.close()

    for p in processes:
        p.join()

    # 最終出力
    for i, result in enumerate(all_results):
        print(f"Link {i}: Final x = {result[-1][0]:.4f}, Final v = {result[-1][1]:.4f}, steps = {step}")

if __name__ == "__main__":
    mp.set_start_method("spawn")  # Windows/macOS安全用
    main()
