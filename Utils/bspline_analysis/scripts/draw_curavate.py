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

def plot_curvatures(filenames, labels, output_filename='curvature_plot.png'):
    """绘制多条曲率曲线"""
    plt.figure(figsize=(10, 6))

    for filename, label in zip(filenames, labels):
        time_data, curvature_data = read_curvature_data(filename)
        plt.plot(time_data, curvature_data, label=label)

    plt.title("Curvature of the B-Spline Curves")
    plt.xlabel("Time (s)")
    plt.ylabel("Curvature (1/m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = [
        #  '../bspline_data/curv_geo.txt', '../bspline_data/curv_init.txt', 
        #  '../bspline_data/curv_opt.txt', '../bspline_data/curv_yaw_constr.txt', 
         '../bspline_data/curv_mid.txt', '../bspline_data/curv_perception.txt', 
        #  '../bspline_data/curv_reall.txt',
	    #  '../bspline_data/yaw_plan.txt', '../bspline_data/yaw_dot_plan.txt', '../bspline_data/yaw_dot_desire_plan.txt'
        #  , '../bspline_data/yaw_opt.txt'
        #  , '../bspline_data/yaw_dot_opt_plan.txt', '../bspline_data/yaw_wpt.txt'
         ]
labels = [
        #  'Curve Geo', 'Curve Init', 
        #  'Curve Opt', 'Curve Yaw Constr', 
         'Curve Mid', 'Curve Perception', 
        #  'Curve Yaw Reall',
	    #  'Yaw Plan', 'Yaw dot Plan', 'Yaw dot Desire'
        #  , 'Yaw Opt'
        #  , 'Yaw dot Opt', 'Yaw waypoint'
         ]
plot_curvatures(filenames, labels)

