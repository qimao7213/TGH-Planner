#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs.point_cloud2 as pc2

class OccupancyGridToPointCloud:
    def __init__(self):
        # 初始化节点
        rospy.init_node("occupancy_grid_to_pointcloud", anonymous=True)

        # 从参数服务器获取分辨率
        self.resolution = rospy.get_param("~resolution", 0.1)  # 默认分辨率为 0.1
        self.z_max = rospy.get_param("~z_max", 2.0)  # 最大高度
        self.z_min = rospy.get_param("~z_min", 0.0)  # 最小高度

        # 订阅 OccupancyGrid 消息
        self.occupancy_grid_sub = rospy.Subscriber("/sdf_map/occupancy_2D", OccupancyGrid, self.occupancy_grid_callback)
        self.voronoi_grid_sub = rospy.Subscriber("/voronoi/gridmap", OccupancyGrid, self.voronoi_grid_sub_callback)
        self.voronoi_marker_sub = rospy.Subscriber("/voronoi/gvg_markers", MarkerArray, self.voronoi_marker_callback)

        # 发布 PointCloud2 消息
        self.obs_pointcloud_pub = rospy.Publisher("/voronoi/obs_cloud", PointCloud2, queue_size=10)
        self.voronoi_grid_pub = rospy.Publisher("/voronoi/gridmap_pointcloud", PointCloud2, queue_size=10)
        self.gvg_marker_pub = rospy.Publisher("/voronoi/gvg_markers_vis", MarkerArray, queue_size=10)

    def voronoi_marker_callback(self, msg):
        modified_markers = MarkerArray()

        for marker in msg.markers:
            # 检查是否是绿色点
            if marker.color.g == 1.0 and marker.color.r == 0.0 and marker.color.b == 0.0:
                # 修改颜色为红色
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                # 修改大小为 0.6
                marker.scale.x = 0.6
                marker.scale.y = 0.6
                marker.scale.z = 0.6
            
            elif marker.color.r == 0.0 and marker.color.g == 0.0 and marker.color.b == 1.0:
                # 修改颜色为绿色
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                # 修改大小为 0.6
                marker.scale.x = 0.6
                marker.scale.y = 0.6
                marker.scale.z = 0.6

            # 将修改后的 Marker 添加到新的 MarkerArray 中
            modified_markers.markers.append(marker)

        # 发布修改后的 MarkerArray
        self.gvg_marker_pub.publish(modified_markers)


    def voronoi_grid_sub_callback(self, msg):
        """
        处理 VoronoiGrid 消息，将 data 值等于 -1 的网格转换为点云
        """
        # 获取地图信息
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # 初始化点云列表
        points = []

        # 遍历 VoronoiGrid 数据
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if msg.data[index]  == -128:  # 只处理 data 值等于 128 的网格
                    # print(f"map data: {msg.data[index]}")
                    # 计算网格中心的世界坐标
                    world_x = origin_x + (x + 0.5) * resolution
                    world_y = origin_y + (y + 0.5) * resolution

                    z = self.z_min
                    points.append([world_x, world_y, z])
        # 将点云转换为 PointCloud2 消息
        pointcloud_msg = self.create_pointcloud2_msg(points, msg.header)
        # 发布点云
        self.voronoi_grid_pub.publish(pointcloud_msg)


    def occupancy_grid_callback(self, msg):
        """
        处理 OccupancyGrid 消息，将 data 值等于 -1 的网格转换为点云
        """
        # 获取地图信息
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # 初始化点云列表
        points = []

        # 遍历 OccupancyGrid 数据
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if msg.data[index] == -1:  # 只处理 data 值等于 -1 的网格
                    # 计算网格中心的世界坐标
                    world_x = origin_x + (x + 0.5) * resolution
                    world_y = origin_y + (y + 0.5) * resolution

                    # 生成从 z_min 到 z_max 的多层点
                    z = self.z_min
                    while z <= self.z_max:
                        points.append([world_x, world_y, z])
                        z += self.resolution * 2

        # 将点云转换为 PointCloud2 消息
        pointcloud_msg = self.create_pointcloud2_msg(points, msg.header)

        # 发布点云
        self.obs_pointcloud_pub.publish(pointcloud_msg)

    def create_pointcloud2_msg(self, points, header):
        """
        将点云列表转换为 PointCloud2 消息
        """
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        return pointcloud_msg

if __name__ == "__main__":
    try:
        node = OccupancyGridToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass