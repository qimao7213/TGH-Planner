#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import sys

def read_map_points(file_path):
    """读取txt文件，返回(x, y, value)列表"""
    points = []
    with open(file_path, 'r') as f:
        for line in f:
            arr = line.strip().split()
            if len(arr) != 3:
                continue
            x, y, value = int(arr[0]), int(arr[1]), int(arr[2])
            if value > 127 :
                value = -1
            points.append((x + 0, y + 0, value))
    return points

def publish_costmap(txt_path, width, height, resolution, origin_x, origin_y):
    rospy.init_node('pub_map_from_txt', anonymous=True)
    pub = rospy.Publisher('/sdf_map/occupancy_2D', OccupancyGrid, queue_size=1, latch=True)

    # 读取点
    points = read_map_points(txt_path)

    # 初始化地图
    data = [125] * (width * height)  # 默认未知
    for x, y, value in points:
        if 0 <= x < width and 0 <= y < height:
            idx = y * width + x
            data[idx] = value

    # 构造OccupancyGrid消息
    msg = OccupancyGrid()
    msg.header.frame_id = "world"
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.position.z = 0.76
    msg.info.origin.orientation.w = 1.0
    msg.data = data

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo("Published costmap from txt")
        rate.sleep()

if __name__ == "__main__":
    txt_path = "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/save_map_point/map_point/icra0_egvg_tesear2.txt"
    txt_path = rospy.get_param("~map_txt_path", txt_path)
    width = 600
    height = 600
    print("请注意修改分辨率！！")
    
    resolution = 0.1
    origin_x = -0.5 * width * resolution
    origin_y = -0.5 * height* resolution
    publish_costmap(txt_path, width, height, resolution, origin_x, origin_y)