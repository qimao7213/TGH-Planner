from PIL import Image

def convert_to_black_white_image(image_path, output_path):
    image = Image.open(image_path).convert('RGBA')
    image_width, image_height = image.size
    new_image = Image.new('RGB', (image_width, image_height), (255, 255, 255))

    for x in range(image_width):
        for y in range(image_height):
            pixel = image.getpixel((x, y))  # RGBA
            # 判断第一个通道（R）是否大于100
            if pixel[0] < 100:
                new_image.putpixel((x, y), (0, 0, 0))
    new_image.save(output_path)
    new_image.show()

# 示例用法
input_image_path = 'MapUniform.png'
output_image_path = 'MapUniform2.png'
convert_to_black_white_image(input_image_path, output_image_path)