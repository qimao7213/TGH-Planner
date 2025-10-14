#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

def image_to_occupancy_grid(img, resolution=0.1, origin=(0.0, 0.0, 0.0)):
    height, width = img.shape
    data = []
    for y in range(height):
        for x in range(width):
            pixel = img[y, x]  # y轴翻转，左下角为原点
            if pixel == 0:      # 障碍物
                data.append(-1)
            else:                 # 其他（free或未知）
                data.append(0)
    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.frame_id = "world"
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin = Pose()
    grid.info.origin.position = Point(origin[0], origin[1], 0.0)
    grid.info.origin.orientation = Quaternion(0, 0, 0, 1)
    grid.data = data
    return grid

def main():
    rospy.init_node('image_to_occupancy_grid_node')
    pub = rospy.Publisher('/sdf_map/occupancy_2D', OccupancyGrid, queue_size=1) 
    img_path = rospy.get_param('~map_png_path')   
    
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        rospy.logerr(f"图像读取失败: {img_path}")
        return
    resolution = 0.1
    imgheight, imgwidth = img.shape[:2]
    offset_x = 0
    offset_y = 0
    occ_grid = image_to_occupancy_grid(img, resolution, origin=(-0.5*resolution*imgheight + offset_x, -0.5*resolution*imgwidth + offset_y, 0.0))
    rate = rospy.Rate(10)  # 10Hz
    rospy.loginfo("开始发布占据栅格地图...")
    while not rospy.is_shutdown():
        occ_grid.header.stamp = rospy.Time.now()
        pub.publish(occ_grid)
        rate.sleep()

if __name__ == '__main__':
    main()