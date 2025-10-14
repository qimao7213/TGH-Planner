import numpy as np
import sys

class BsplineOptimizer:
    def __init__(self, order):
        self.order = order

    def calc_smoothness_cost(self, q):
        cost = 0.0
        gradient = np.zeros_like(q)

        for i in range(len(q) - self.order):
            jerk = q[i + 3, :] - 3 * q[i + 2, :] + 3 * q[i + 1, :] - q[i, :]
            cost += np.linalg.norm(jerk)**2
            temp_j = 2.0 * jerk
            gradient[i, :] += -temp_j
            gradient[i + 1, :] += 3.0 * temp_j
            gradient[i + 2, :] += -3.0 * temp_j
            gradient[i + 3, :] += temp_j

        return cost, gradient

# 读取控制点数据，并补全z=0
def read_control_points(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    # 假设每行数据是空格分隔的x, y值
    data = [list(map(float, line.split())) for line in lines]
    # 将数据转换为numpy数组，并补全z=0.5
    q = np.array([np.append(point, 0.5) for point in data])
    return q

# 主函数
def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <control_points_file>")
        sys.exit(1)

    # 设置阶数为3
    optimizer = BsplineOptimizer(order=3)
    control_points_file = sys.argv[1]  # 从命令行参数获取文件路径

    # 读取控制点
    q = read_control_points(control_points_file)

    # 计算smoothness cost和梯度
    cost, gradient = optimizer.calc_smoothness_cost(q)

    print("Cost:", cost)
    print("Gradient:")
    
    # 遍历梯度并打印编号和梯度值
    for i in range(len(gradient)):
        print(f"Control Point {i}: {gradient[i, :]}")

if __name__ == "__main__":
    main()
