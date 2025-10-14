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
        #  '../bspline_data/curv_geo.txt',  
        #  '../bspline_data/ctrlYawDot_init.txt', '../bspline_data/w_init.txt', 
        #  '../bspline_data/ctrlYawDot_opt.txt',  '../bspline_data/w_opt.txt',
         '../bspline_data/ctrlYawDot_mid.txt',  '../bspline_data/w_mid.txt',
         '../bspline_data/ctrlYawDot_perception.txt',  '../bspline_data/w_perception.txt',
        #  '../bspline_data/ctrlYawDot_yaw_constr.txt', '../bspline_data/w_yaw_constr.txt', 
        #  '../bspline_data/ctrlYawDot_reall.txt', '../bspline_data/w_reall.txt',
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

