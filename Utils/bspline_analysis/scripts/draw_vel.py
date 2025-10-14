import numpy as np
import matplotlib.pyplot as plt

def read_velocity_data(filename):
    """读取速度数据 vx, vy, vz 和时间戳"""
    time_data = []
    vx_data = []
    vy_data = []
    vnorm_data = []
    with open(filename, 'r') as file:
        for line in file:
            try:
                vx, vy, vz, time = map(float, line.strip().split())
                time_data.append(time)
                vx_data.append(vx)
                vy_data.append(vy)
                vnorm = np.sqrt(vx**2 + vy**2)
                vnorm_data.append(vnorm)
            except ValueError:
                print(f"Skipping invalid line: {line.strip()}")
    return time_data, vx_data, vy_data, vnorm_data

def plot_velocity_components(filenames, labels, output_filename='velocity_components.png'):
    """分别绘制 vx、vy 和 v_norm 的子图"""
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    for filename, label in zip(filenames, labels):
        time_data, vx_data, vy_data, vnorm_data = read_velocity_data(filename)

        axs[0].plot(time_data, vx_data, label=label)
        axs[1].plot(time_data, vy_data, label=label)
        axs[2].plot(time_data, vnorm_data, label=label)

    axs[0].set_ylabel("vx (m/s)")
    axs[0].set_title("vx vs Time")
    axs[0].grid(True)

    axs[1].set_ylabel("vy (m/s)")
    axs[1].set_title("vy vs Time")
    axs[1].grid(True)

    axs[2].set_ylabel("v_norm (m/s)")
    axs[2].set_title("||v|| = sqrt(vx² + vy²) vs Time")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True)

    for ax in axs:
        ax.legend()

    plt.tight_layout()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = ['../bspline_data/vel_init.txt', '../bspline_data/vel_opt.txt', '../bspline_data/vel_reall.txt', '../bspline_data/vel_geo.txt', '../bspline_data/vel_yaw_constr.txt']
labels = ['Vel init', 'Vel opt', 'Vel reall', 'Vel geo', 'Vel yaw constr']
plot_velocity_components(filenames, labels)

