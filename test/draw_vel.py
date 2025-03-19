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

def plot_curvatures(filenames, labels, output_filename='vel_plot.png'):
    """绘制多条曲率曲线"""
    plt.figure(figsize=(10, 6))

    for filename, label in zip(filenames, labels):
        time_data, curvature_data = read_curvature_data(filename)
        plt.plot(time_data, curvature_data, label=label)

    plt.title("Vel of the B-Spline")
    plt.xlabel("Time (s)")
    plt.ylabel("Vel (s/m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = ['data/vel_init.txt', 'data/vel_opt.txt', 'data/vel_reall.txt', 'data/vel_geo.txt', 'data/vel_yaw_constr.txt']
labels = ['Vel init', 'Vel opt', 'Vel reall', 'Vel geo', 'Vel yaw constr']
plot_curvatures(filenames, labels)

