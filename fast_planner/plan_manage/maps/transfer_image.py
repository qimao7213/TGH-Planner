from PIL import Image

def convert_to_black_white_image(image_path, output_path):
    # 读取原始图像
    image = Image.open(image_path)

    # 转换图像为RGB模式
    image = image.convert('RGB')

    # 获取图像的宽度和高度
    image_width, image_height = image.size

    # 创建一个新的图像，背景为白色
    new_image = Image.new('RGB', (image_width, image_height), 'white')

    # 遍历每个像素，判断是否为白色，非白色转换为黑色
    for x in range(image_width):
        for y in range(image_height):
            pixel = image.getpixel((x, y))

            # 判断像素是否是白色
            if pixel != (255, 255, 255):
                new_image.putpixel((x, y), (0, 0, 0))  # 非白色像素变为黑色

    # 保存结果图像
    new_image.save(output_path)
    new_image.show()

# 示例用法
input_image_path = 'map_maze.png'  # 输入图像路径
output_image_path = 'map_maze2.png'  # 输出图像路径

convert_to_black_white_image(input_image_path, output_image_path)

