import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# 示例数据：控制点
x_control_points = np.array([0, 1.2, 2.5, 3.7, 1.0, 5.9])
y_control_points = np.zeros_like(x_control_points)  # 所有y值为0

# 生成插值B样条
t = np.linspace(0, 1, len(x_control_points))
spl = make_interp_spline(t, x_control_points, k=3)  # k为B样条的阶数，k=3表示三次B样条

# 在均匀间隔的点上计算样条插值
t_dense = np.linspace(0, 1, 100)
x_dense = spl(t_dense)

# 可视化
plt.figure(figsize=(10, 4))
plt.plot(x_dense, np.zeros_like(x_dense), label='B-spline Curve')
plt.scatter(x_control_points, y_control_points, color='red', zorder=5, label='Control Points')
plt.axhline(0, color='grey', linewidth=0.5)  # 添加水平线
plt.xlabel('X')
plt.ylabel('Y')
plt.title('1D B-spline Curve Visualization')
plt.legend()
plt.grid(True)
plt.show()


