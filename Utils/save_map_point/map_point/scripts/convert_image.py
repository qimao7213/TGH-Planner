# color2bw_resize_flip.py
import cv2
import sys

def process_image(image_path, save_path=None, target_width=1000):
    """
    1. 等比例缩放：宽固定为 target_width，高按比例自动计算
    2. 上下翻转（flip vertical）
    3. 转成单通道黑白图：
       RGB 均 ≥200 的像素 → 255（白），其余 → 0（黑）
    """
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(image_path)

    # 1. 等比例缩放
    h, w = img.shape[:2]
    new_w = target_width
    new_h = int(round(h * new_w / w))
    img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # 2. 上下翻转
    img_flipped = cv2.flip(img_resized, 0)  # 0 = flip vertical

    # 3. 黑白化
    mask = cv2.inRange(img_flipped, (200, 200, 200), (255, 255, 255))
    bw = mask.copy()
    bw[bw < 255] = 0

    if save_path:
        cv2.imwrite(save_path, bw)
        print(f"Saved: {save_path}  (size: {new_w}x{new_h}, flipped)")
    return bw

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python color2bw_resize_flip.py input.jpg [output.jpg]")
        sys.exit(1)
    in_file  = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else "bw_flip_" + in_file
    process_image(in_file, out_file)
