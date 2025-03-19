import numpy as np
import matplotlib.pyplot as plt

def read_curvature_data(filename):
    """读取曲率数据和时间戳"""
    time_data = []
    curvature_data = []
    with open(filename, 'r') as file:
        for line in file:
            try:
                curvature, time = map(float, line.strip().split())
                time_data.append(time)
                curvature_data.append(curvature)
            except ValueError:
                print(f"Skipping invalid line: {line.strip()}")
    return time_data, curvature_data

def plot_curvatures(filenames, labels, output_filename='acc_plot.png'):
    """绘制多条曲率曲线"""
    plt.figure(figsize=(10, 6))

    for filename, label in zip(filenames, labels):
        time_data, curvature_data = read_curvature_data(filename)
        plt.plot(time_data, curvature_data, label=label)

    plt.title("Acc of the B-Spline")
    plt.xlabel("Time (s)")
    plt.ylabel("Acc (s*s/m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = ['data/acc_init.txt', 'data/acc_opt.txt', 'data/acc_yaw_constr.txt', 'data/acc_reall.txt']
labels = ['Acc init', 'Acc opt', 'Acc yaw constr',  'Acc reall']
plot_curvatures(filenames, labels)

