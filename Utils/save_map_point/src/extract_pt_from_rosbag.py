#!/usr/bin/env python3
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np

def save_pointcloud_from_bag(bag_path, topic_name, output_file_base):
    try:
        bag = rosbag.Bag(bag_path, 'r')

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # 检查点云字段
            print(f"点云字段信息: {[field.name for field in msg.fields]}")

            # 读取点云数据
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if not points:
                print("点云消息为空，无法处理。")
                return

            # 检查点云数据范围
            points_np = np.asarray(points)
            min_x, max_x = points_np[:, 0].min(), points_np[:, 0].max()
            min_y, max_y = points_np[:, 1].min(), points_np[:, 1].max()
            min_z, max_z = points_np[:, 2].min(), points_np[:, 2].max()
            print(f"点云数据范围: x=[{min_x}, {max_x}], y=[{min_y}, {max_y}], z=[{min_z}, {max_z}]")

            # 转换为 Open3D 点云对象
            pointcloud = o3d.geometry.PointCloud()
            pointcloud.points = o3d.utility.Vector3dVector(points)


            # 保存为 ASCII 格式的 PLY 文件
            ply_file = f"{output_file_base}.ply"
            o3d.io.write_point_cloud(ply_file, pointcloud, write_ascii=True)
            print(f"点云已保存到 ASCII 格式的 PLY 文件: {ply_file}")
            
            # 保存为 PCD 文件
            points_np = np.asarray(pointcloud.points)  # 将点云数据转换为 NumPy 数组
            points_np[:, 0] -= 45  # 修改 x 坐标
            points_np[:, 1] -= 45  # 修改 y 坐标
            pointcloud.points = o3d.utility.Vector3dVector(points_np)  # 将修改后的数据赋值回点云对象

            pcd_file = f"{output_file_base}.pcd"
            o3d.io.write_point_cloud(pcd_file, pointcloud)
            print(f"点云已保存到 PCD 文件: {pcd_file}")         
            

            break

        bag.close()

    except Exception as e:
        print(f"处理 ROS bag 文件时出错: {e}")

if __name__ == "__main__":
    bag_path = "/home/bhrqhb/2025-09-17-22-30-23.bag"
    topic_name = "/explored_areas"
    output_file_base = "output"

    save_pointcloud_from_bag(bag_path, topic_name, output_file_base)
