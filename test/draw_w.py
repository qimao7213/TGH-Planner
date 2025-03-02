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

def plot_curvatures(filenames, labels, output_filename='w_plot.png'):
    """绘制多条角速度曲线"""
    plt.figure(figsize=(10, 6))

    for filename, label in zip(filenames, labels):
        time_data, curvature_data = read_curvature_data(filename)
        plt.plot(time_data, curvature_data, label=label)

    plt.title("angel vel of the B-Spline Curves")
    plt.xlabel("Time (s)")
    plt.ylabel("W (1/m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = [
        #  'data/curv_geo.txt',  
        #  'data/ctrlYawDot_init.txt', 'data/w_init.txt', 
        #  'data/ctrlYawDot_opt.txt',  'data/w_opt.txt',
         'data/ctrlYawDot_mid.txt',  'data/w_mid.txt',
         'data/ctrlYawDot_perception.txt',  'data/w_perception.txt',
        #  'data/ctrlYawDot_yaw_constr.txt', 'data/w_yaw_constr.txt', 
        #  'data/ctrlYawDot_reall.txt', 'data/w_reall.txt',
         ]
labels = [
        #  'Curve Geo',   
        #  'CtrlYawDot Init', 'W Init', 
        #  'CtrlYawDot Opt',  'W Opt', 
         'CtrlYawDot Mid', 'W Mid', 
         'CtrlYawDot Perception', 'W Perception', 
        #  'CtrlYawDot Yaw Constr', 'W Yaw Constr', 
        #  'CtrlYawDot Yaw Reall', 'W Yaw Reall',

         ]
plot_curvatures(filenames, labels)

