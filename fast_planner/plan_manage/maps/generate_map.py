from PIL import Image, ImageDraw
import random
import math

def generate_circles(image_width, image_height, radius, num_circles, min_distance):
    # 创建一个白色背景的图像
    image = Image.new('RGB', (image_width, image_height), 'white')
    draw = ImageDraw.Draw(image)

    # 圆心列表，防止圆重叠
    centers = []

    for _ in range(num_circles):
        # 随机生成圆心位置
        x = random.randint(radius, image_width - radius)
        y = random.randint(radius, image_height - radius)

        # 检查圆心之间的距离是否满足最小距离要求
        if all(math.hypot(x - cx, y - cy) >= min_distance for cx, cy in centers):
            centers.append((x, y))
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), 'black')

    # 保存图像
    image.save('circles_empty.png')

# 参数
image_width = 250  # 图像宽度
image_height = 150  # 图像高度
radius = 10  # 圆的半径
num_circles = 0  # 圆的数量
min_distance = 2 * radius  # 圆心之间的大致距离，控制密度

# 生成图像
generate_circles(image_width, image_height, radius, num_circles, min_distance)
