import numpy as np
import matplotlib.pyplot as plt

def read_acceleration_data(filename):
    """读取加速度 ax, ay, az 和时间戳"""
    time_data = []
    ax_data = []
    ay_data = []
    anorm_data = []
    with open(filename, 'r') as file:
        for line in file:
            try:
                ax, ay, az, time = map(float, line.strip().split())
                time_data.append(time)
                ax_data.append(ax)
                ay_data.append(ay)
                anorm = np.sqrt(ax**2 + ay**2)
                anorm_data.append(anorm)
            except ValueError:
                print(f"Skipping invalid line: {line.strip()}")
    return time_data, ax_data, ay_data, anorm_data

def plot_acceleration_components(filenames, labels, output_filename='acc_plot.png'):
    """分别绘制 ax、ay 和 a_norm 的子图"""
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    for filename, label in zip(filenames, labels):
        time_data, ax_data, ay_data, anorm_data = read_acceleration_data(filename)

        axs[0].plot(time_data, ax_data, label=label)
        axs[1].plot(time_data, ay_data, label=label)
        axs[2].plot(time_data, anorm_data, label=label)

    axs[0].set_ylabel("ax (m/s²)")
    axs[0].set_title("ax vs Time")
    axs[0].grid(True)

    axs[1].set_ylabel("ay (m/s²)")
    axs[1].set_title("ay vs Time")
    axs[1].grid(True)

    axs[2].set_ylabel("a_norm (m/s²)")
    axs[2].set_title("||a|| = sqrt(ax² + ay²) vs Time")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True)

    for ax in axs:
        ax.legend()

    plt.tight_layout()
    plt.savefig(output_filename)
    plt.show()

# 主程序
filenames = ['data/acc_init.txt', 'data/acc_opt.txt', 'data/acc_yaw_constr.txt', 'data/acc_reall.txt']
labels = ['Acc init', 'Acc opt', 'Acc yaw constr',  'Acc reall']
plot_acceleration_components(filenames, labels)

