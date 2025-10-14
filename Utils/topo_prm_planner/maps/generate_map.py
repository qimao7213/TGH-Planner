import cv2
import numpy as np
import random
import signal
import sys

def generate_obstacles(image_size, num_obstacles, radius, min_distance):
    img = np.ones((image_size, image_size, 3), dtype=np.uint8) * 0
    obstacles = []

    # 上半区域
    attempts = 0
    for _ in range(num_obstacles):
        if attempts >= num_obstacles:
            break
        x = random.randint(radius, image_size - radius)
        y = random.randint(radius, image_size // 4 - radius)
        attempts += 1
        if all(np.hypot(x - ox, y - oy) >= min_distance for ox, oy in obstacles):
            obstacles.append((x, y))

    # 下半区域
    attempts = 0
    for _ in range(num_obstacles):
        if attempts >= num_obstacles:
            break
        x = random.randint(radius, image_size - radius)
        y = random.randint(image_size * 3 // 4 + radius, image_size - radius)
        attempts += 1
        if all(np.hypot(x - ox, y - oy) >= min_distance for ox, oy in obstacles):
            obstacles.append((x, y))

    # 绘制障碍物
    for x, y in obstacles:
        cv2.circle(img, (x, y), radius, (255, 255, 255), -1)

    return img

def signal_handler(sig, frame):
    print("\n程序被手动中断，正在退出...")
    cv2.destroyAllWindows()
    sys.exit(0)

def main():
    # 捕获 Ctrl+C 中断信号
    signal.signal(signal.SIGINT, signal_handler)

    image_size = 1000
    num_obstacles = 1000
    radius = 4
    min_distance = 20

    img = generate_obstacles(image_size, num_obstacles, radius, min_distance)

    # cv2.imshow("Obstacle Map", img)
    # print("按任意键关闭图像窗口，或按 Ctrl+C 强制退出")
    # cv2.waitKey(0)
    cv2.imwrite('ObstacleMap.png', img)
    # cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
