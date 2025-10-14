#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class OccupancyGridToPointCloud {
public:
    OccupancyGridToPointCloud(ros::NodeHandle& nh) : nh_(nh) {
        // 加载全局地图
        std::string pcd_file;
        nh_.param<std::string>("pcd_file", pcd_file, "global_map.pcd");
        nh_.param<double>("offset_x", offset_x_, 0.0);
        nh_.param<double>("offset_y", offset_y_, 0.0);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *global_map_)) {
            ROS_ERROR("Can not loca PCD file : %s", pcd_file.c_str());
            return;
        }
        ROS_INFO("Loaded PCD file: %s", pcd_file.c_str());

        // 对全局地图进行降采样
        double voxel_size;
        nh_.param<double>("voxel_size", voxel_size, 0.1);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*downsampled_map_);
        // ROS_INFO("全局地图降采样完成，点数: %lu", downsampled_map_->size());

        // 发布全局地图点云
        global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/egvg_vis/global_map", 1);
        // 发布点云
        free_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/egvg_vis/free_cloud", 1);
        occupy_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/egvg_vis/occupy_cloud", 1);

        // 发布修改后的 MarkerArray
        modified_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/egvg_vis/voronoi_markers", 1);

        // 发布 voronoi 点云
        voronoi_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/egvg_vis/voronoi_cloud", 1);

        // 订阅 /
        occupancy_grid_sub_ = nh_.subscribe("/sdf_map/occupancy_2D", 1, &OccupancyGridToPointCloud::occupancyGridCallback, this);
        // 订阅 /voronoi/gridmap
        voronoi_grid_sub_ = nh_.subscribe("/voronoi/gridmap", 1, &OccupancyGridToPointCloud::voronoiGridCallback, this);
        // 订阅 /voronoi/gvg_markers
        voronoi_marker_sub_ = nh_.subscribe("/voronoi/gvg_markers", 1, &OccupancyGridToPointCloud::voronoiMarkerCallback, this);


        // 定时器，用于以 1Hz 发布全局地图
        timer_ = nh_.createTimer(ros::Duration(1.0), &OccupancyGridToPointCloud::publishGlobalMap, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher global_map_pub_;
    ros::Publisher free_cloud_pub_;
    ros::Publisher occupy_cloud_pub_;
    ros::Publisher modified_marker_pub_;
    ros::Publisher voronoi_cloud_pub_;
    ros::Subscriber voronoi_grid_sub_;
    ros::Subscriber voronoi_marker_sub_;
    ros::Subscriber occupancy_grid_sub_;
    ros::Timer timer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map_{new pcl::PointCloud<pcl::PointXYZ>()};

    double offset_x_, offset_y_;

    void publishGlobalMap(const ros::TimerEvent&) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*downsampled_map_, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        global_map_pub_.publish(cloud_msg);
    }

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr occupy_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        int width = msg->info.width;
        int height = msg->info.height;

        // 遍历 OccupancyGrid 数据
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                double world_x = origin_x + (x + 0.5) * resolution + offset_x_;
                double world_y = origin_y + (y + 0.5) * resolution + offset_y_;

                if (msg->data[index] == 0) {  // free 点
                    for (double z = 0.0; z <= 0.0; z += 0.2) {
                        free_cloud->points.emplace_back(world_x, world_y, z);
                    }
                }
                if (msg->data[index] == -1)
                {
                    for (double z = 0.0; z < 2.0; z += 0.2)
                    {
                        occupy_cloud->points.emplace_back(world_x, world_y, z);
                    }
                }
            }
        }

        // 发布 点云
        sensor_msgs::PointCloud2 free_cloud_msg;
        pcl::toROSMsg(*free_cloud, free_cloud_msg);
        free_cloud_msg.header.frame_id = "world";
        free_cloud_msg.header.stamp = ros::Time::now();
        free_cloud_pub_.publish(free_cloud_msg);

        sensor_msgs::PointCloud2 occupy_cloud_msg;
        pcl::toROSMsg(*occupy_cloud, occupy_cloud_msg);
        occupy_cloud_msg.header.frame_id = "world";
        occupy_cloud_msg.header.stamp = ros::Time::now();
        occupy_cloud_pub_.publish(occupy_cloud_msg);
    }


    void voronoiGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        int width = msg->info.width;
        int height = msg->info.height;

        // 遍历 OccupancyGrid 数据
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                double world_x = origin_x + (x + 0.5) * resolution;
                double world_y = origin_y + (y + 0.5) * resolution;

                if (msg->data[index] == -128) {  // voronoi 点
                    voronoi_cloud->points.emplace_back(world_x, world_y, 0.0);
                } 
            }
        }
        
        // 发布 voronoi 点云
        sensor_msgs::PointCloud2 voronoi_cloud_msg;
        pcl::toROSMsg(*voronoi_cloud, voronoi_cloud_msg);
        voronoi_cloud_msg.header.frame_id = "world";
        voronoi_cloud_msg.header.stamp = ros::Time::now();
        voronoi_cloud_pub_.publish(voronoi_cloud_msg);
    }

    void voronoiMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        visualization_msgs::MarkerArray modified_markers;

        for (const auto& marker : msg->markers) {
            visualization_msgs::Marker modified_marker = marker;

            if (marker.color.g == 1.0 && marker.color.r == 0.0 && marker.color.b == 0.0) {
                // 修改颜色为红色
                modified_marker.color.r = 1.0;
                modified_marker.color.g = 0.0;
                modified_marker.color.b = 0.0;
            } else if (marker.color.r == 0.0 && marker.color.g == 0.0 && marker.color.b == 1.0) {
                // 修改颜色为绿色
                modified_marker.color.r = 0.0;
                modified_marker.color.g = 1.0;
                modified_marker.color.b = 0.0;
            }

            // 修改大小
            modified_marker.scale.x = 0.6;
            modified_marker.scale.y = 0.6;
            modified_marker.scale.z = 0.6;

            modified_markers.markers.push_back(modified_marker);
        }

        // 发布修改后的 MarkerArray
        modified_marker_pub_.publish(modified_markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "egvg_vis");
    ros::NodeHandle nh("~");

    OccupancyGridToPointCloud node(nh);

    ros::spin();
    return 0;
}