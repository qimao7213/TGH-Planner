import numpy as np
import matplotlib.pyplot as plt

# 初始化状态和控制量
state0 = np.array([0.0, 0.0, 3.14159 * 0.2, 1.5, 0.0, 0.0])  # 初始状态 [x, y, yaw, v, 其他]
um = np.array([1.5, 0.5])  # 控制量 [v, delta]
wheel_base = 0.5  # 车辆的轴距
tau = 0.3  # 时间步长
tau_long = 0.5 
tau_short = 0.01
total_time = 15.0  # 总时间

# 状态转移函数
def stateTransit(state0, um, tau, wheel_base):
    dot_s = np.array([
        um[0] * np.cos(state0[2]),  # v * cos(yaw)
        um[0] * np.sin(state0[2]),  # v * sin(yaw)
        um[0] * np.tan(um[1]) / wheel_base  # (v * tan(delta)) / wheel_base
    ])
    state1 = state0.copy()
    state1[:3] = state0[:3] + dot_s * tau
    state1[2] = (state1[2] + 2.0 * np.pi) % (2.0 * np.pi)
    state1[3] = um[0]
    return state1

# 初始化轨迹
trajectory = [state0[:2]]
state = state0
for _ in np.arange(tau_short, total_time + tau_short, tau_short):
    state = stateTransit(state, um, tau_short, wheel_base)
    trajectory.append(state[:2])
trajectory = np.array(trajectory)

trajectory2 = [state0[:2]]
state = state0
for _ in np.arange(tau, total_time + tau, tau):
    state = stateTransit(state0, um, _, wheel_base)
    trajectory2.append(state[:2])
trajectory2 = np.array(trajectory2)

trajectory3 = [state0[:2]]
state = state0
for _ in np.arange(tau_long, total_time + tau_long, tau_long):
    state = stateTransit(state, um, tau_long, wheel_base)
    trajectory3.append(state[:2])
trajectory3 = np.array(trajectory3)

# 绘制轨迹
plt.plot(trajectory[:, 0], trajectory[:, 1], "-o")
plt.plot(trajectory2[:, 0], trajectory2[:, 1], "--o")
# plt.plot(trajectory3[:, 0], trajectory3[:, 1], "--o")
plt.title("2D Trajectory")
plt.xlabel("x")
plt.ylabel("y")
plt.axis('equal')
plt.grid(True)

plt.show()


