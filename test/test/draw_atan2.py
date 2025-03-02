import numpy as np
import matplotlib.pyplot as plt

# 创建网格点
x = np.linspace(-10, 10, 400)
y = np.linspace(-10, 10, 400)
X, Y = np.meshgrid(x, y)

# 计算每个点的atan2值
Z = np.arctan2(Y, X)

# 绘制曲线
plt.figure(figsize=(10, 8))
contour = plt.contourf(X, Y, Z, 50, cmap='hsv')
plt.colorbar(contour)
plt.title('atan2(y, x) function curve')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
plt.show()


