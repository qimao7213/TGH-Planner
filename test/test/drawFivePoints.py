import matplotlib.pyplot as plt

# 定义5个点的坐标
points = [
    (-2, 1),  # 左边第一个点
    (-1, 0.25),  # 左边第二个点
    (0, 0),  # 中间点
    (1, -0.25),  # 右边第二个点
    (2, -1)  # 右边第一个点
]

# 分别获取x和y坐标
x_coords, y_coords = zip(*points)

# 绘制这些点
plt.figure(figsize=(8, 6))
plt.plot(x_coords, y_coords, 'bo-', label='Points')  # 使用蓝色圆点，并用线连接
plt.title("Five Points Plot")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()
plt.axhline(0, color='gray', linewidth=0.5)
plt.axvline(0, color='gray', linewidth=0.5)
plt.xlim(-3, 3)
plt.ylim(-2, 2)

# 标注每个点
for (x, y) in points:
    plt.text(x, y, f'({x}, {y})', fontsize=12, ha='right')

plt.show()


