//
// Created by hjl on 2021/9/18.
//
#ifndef TOPO_PLANNER_WS_SLAM_OUTPUT_H
#define TOPO_PLANNER_WS_SLAM_OUTPUT_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
class SlamOutput {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener tf_listener;
    ros::Publisher odom_pub;
    ros::Publisher reg_pub;
    ros::Publisher dwz_cloud_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_tf_sub;
    ros::Timer global_down_timer;

    tf::StampedTransform transform;
    ros::Rate rate;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> local_cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> local_odom_sub_;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
    SynchronizerLocalCloudOdom sync_local_cloud_odom_;

    std::string slam_frame_id;
    std::string child_frame_id;
    std::string lidar_frame, sensor_odom_topic;
    std::string base_frame = "base_link";
    std::string world_frame_id = "world"; // 全局坐标系
    bool is_get_first;
    bool is_sim = false; // 是否为仿真环境，如果是的话需要将odom转换为tf发布
    Eigen::Isometry3d T_B_L, T_W_SLAM; // SLAM初始坐标系在World下的转换
    std::vector<tf::StampedTransform> ST_B_Bi_vec;
    double vec_length;
    double offset_gazebo_world_x = 0.0;
    double offset_gazebo_world_y = 0.0;
    double offset_world_to_camerainit_x = 0.0;
    double offset_world_to_camerainit_y = 0.0;

    double base_link_z_ = 0.0; // base_link的z高度
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    double down_voxel_size;

    SlamOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud,const nav_msgs::OdometryConstPtr& input);
    void odomTfCallback(const nav_msgs::OdometryConstPtr& odom_msg);
};



#endif //TOPO_PLANNER_WS_SLAM_OUTPUT_H
