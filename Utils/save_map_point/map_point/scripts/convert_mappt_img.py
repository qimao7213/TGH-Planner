#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取 map_debug_topopathset 文本（每行: x y value），
将 value==255 的网格绘制为黑色像素，其余为白色，输出 PNG。

用法：
    python topopath_to_png.py input.txt output.png
    # 如果需要把 y 轴翻转（将几何y向上映射为图像y向下）：
    python topopath_to_png.py input.txt output.png --flip-y
"""

import argparse
import numpy as np
from PIL import Image

def load_points(path):
    pts = []
    with open(path, 'r', encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            parts = ln.replace(',', ' ').split()
            if len(parts) < 3:
                continue
            try:
                x = int(float(parts[0]))
                y = int(float(parts[1]))
                v = int(float(parts[2]))
                pts.append((x, y, v))
            except ValueError:
                # 跳过无法解析的行
                continue
    if not pts:
        raise ValueError("未在输入文件中解析到任何 (x, y, value) 数据。")
    return pts

def build_image(pts, flip_y=False):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    width  = max_x - min_x + 1
    height = max_y - min_y + 1
    if width <= 0 or height <= 0:
        raise ValueError("无效的图像尺寸，请检查输入数据。")

    # 初始化为白色（255）
    img = np.full((height, width), 255, dtype=np.uint8)

    # 填充黑色：value==255 的点
    for x, y, v in pts:
        col = x - min_x
        if flip_y:
            row = max_y - y      # 将几何y向上映射到图像y向下
        else:
            row = y - min_y      # 直接把最小y映射为图像顶部
        if 0 <= row < height and 0 <= col < width:
            if v == 255:
                img[row, col] = 0  # 黑色
            else:
                img[row, col] = 255  # 白色（可省略，初始化已白）

    return img

def main():
    ap = argparse.ArgumentParser(description="将 map_debug_topopathset 数据转成 PNG 图像（255=黑，其余=白）")
    ap.add_argument("input", help="输入文本路径（每行: x y value）")
    ap.add_argument("output", help="输出 PNG 路径")
    ap.add_argument("--flip-y", action="store_true",
                    help="翻转 y 轴：如果你的几何/地图 y 向上、而图像 y 向下，建议开启")
    args = ap.parse_args()

    pts = load_points(args.input)
    img = build_image(pts, flip_y=args.flip_y)
    Image.fromarray(img).save(args.output)
    print(f"已保存: {args.output}  大小: {img.shape[1]}x{img.shape[0]}")

if __name__ == "__main__":
    main()
