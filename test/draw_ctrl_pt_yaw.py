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

def plot_curvatures(filenames, labels, output_filename='ctrl_pt_yaw.png'):
    """绘制多条曲率曲线"""
    plt.figure(figsize=(10, 6))

    for filename, label in zip(filenames, labels):
        time_data, curvature_data = read_curvature_data(filename)
        plt.plot(time_data, curvature_data, label=label)

    plt.title("Yaw of the B-Spline Ctrl Points")
    plt.xlabel("Time (s)")
    plt.ylabel("Vel (s/m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = [  
            #  'data/ctrlYaw_mid.txt', 
            #  'data/ctrlYaw_perception.txt', 
             'data/ctrlYaw_opt.txt', 
             'data/ctrlYaw_yaw_constr.txt', 

            #  'data/ctrlYawConstr_mid.txt', 
            #  'data/ctrlYawConstr_perception.txt', 
            #  'data/ctrlYawConstr_opt.txt', 
            #  'data/ctrlYawConstr_yaw_constr.txt', 

            #  'data/ctrlYawDot_mid.txt', 
            #  'data/ctrlYawDot_perception.txt', 
             'data/ctrlYawDot_opt.txt', 
             'data/ctrlYawDot_yaw_constr.txt'
             ]
labels = [  
          # 'Yaw mid', 
          # 'Yaw perception', 
          'Yaw opt', 
          'Yaw constr', 
          
          # 'YawConstr mid', 
          # 'YawConstr perception', 
        #   'YawConstr opt', 
        #   'YawConstr constr', 

          # 'YawDot mid', 
          # 'YawDot perception', 
          'YawDot opt', 
          'YawDot constr'
          ]
plot_curvatures(filenames, labels)

