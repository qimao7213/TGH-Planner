#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

def read_trajectory_file(file_path):
    """
    读取轨迹文件，返回点的列表，每个点包含 x, y, z, 和 dis。
    """
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split()
            if len(data) < 7:
                continue  # 跳过无效行
            x, y, z, yaw, pitch, roll, dis, time = map(float, data)
            points.append((x, y, z, time))
    return points

def create_pointcloud(points, frame_id="world"):
    """
    将点的列表转换为 PointCloud2 消息。
    """
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # 将 z 坐标统一设置为 0.5，并构造点云数据
    cloud_points = []
    for point in points:
        x, y, z, time = point
        cloud_points.append((x-0, y-0, 0.5, time))  # z 统一为 0.5，intensity 为 time

    # 定义 PointCloud2 的字段
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]

    # 创建 PointCloud2 消息
    pointcloud_msg = pc2.create_cloud(header, fields, cloud_points)
    return pointcloud_msg

def main():
    rospy.init_node('multi_trajectory_to_pointcloud', anonymous=True)

    # 手动指定轨迹文件路径
    trajectory_files = [
        "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/traj_analysis/data2/scene_sprase/gvg/traj_real_egvg.txt",
        "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/traj_analysis/data2/scene_sprase/tgh/traj_real_egvg.txt",
        # "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/traj_analysis/data2/scene_sprase/far/trajectory_2025-9-21-13-24-39.txt",
        # "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/traj_analysis/data2/scene_6/egvg4/traj_real_egvg.txt"
    ]

    frame_id = rospy.get_param('~frame_id', 'world')
    publish_rate = rospy.get_param('~publish_rate', 1)  # 发布频率（Hz）

    # 创建发布器和缓存点云消息
    publishers = []
    cached_pointclouds = []
    for i, file in enumerate(trajectory_files):
        topic_name = f"/trajectory_pointcloud_{i+1}"
        pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
        publishers.append(pub)

        # 读取轨迹文件并生成点云消息
        points = read_trajectory_file(file)
        print(f"Read {len(points)} points from {file}")
        pointcloud_msg = create_pointcloud(points, frame_id)
        cached_pointclouds.append(pointcloud_msg)

        # rospy.loginfo(f"Created publisher for topic: {topic_name}, file: {file}")

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        for i, pub in enumerate(publishers):
            # 直接发布缓存的点云消息
            pub.publish(cached_pointclouds[i])
            # rospy.loginfo(f"Published pointcloud for topic: /trajectory_pointcloud_{i+1}")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass