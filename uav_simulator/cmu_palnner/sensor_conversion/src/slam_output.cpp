//
// Created by hjl on 2021/9/18.
//

#include "slam_simulation/slam_output.h"

bool getTransform(const std::string& source_frame, const std::string& target_frame, const ros::Time& time,
                          Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
  static tf::TransformListener listener;

  try {
    // 使用 tf 查询变换
    static tf::StampedTransform transform;
    listener.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
    listener.lookupTransform(target_frame, source_frame, time, transform);

    // 提取平移
    position.x() = transform.getOrigin().x();
    position.y() = transform.getOrigin().y();
    position.z() = transform.getOrigin().z();

    // 提取旋转
    tf::Quaternion tf_quat = transform.getRotation();
    orientation = Eigen::Quaterniond(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());

    return true;
  } catch (tf::TransformException& ex) {
    ROS_WARN("Failed to get transform from %s to %s: %s", 
             source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}


SlamOutput::SlamOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private),
                                                                                       rate(100), vec_length(2.0),
                                                                                       is_get_first(false) {
    const std::string &ns = ros::this_node::getName();
    frame_id = "world";
    if (!ros::param::get(ns + "/frame_id", frame_id)) {
        ROS_WARN("No frame_id specified. Looking for %s. Default is 'map'.",
                 (ns + "/frame_id").c_str());
    }

    child_frame_id = "jackal/velodyne/VLP_16";
    if (!ros::param::get(ns + "/child_frame_id", child_frame_id)) {
        ROS_WARN("No child_frame_id specified. Looking for %s. Default is 'sensor'.",
                 (ns + "/child_frame_id").c_str());
    }

    down_voxel_size = 0.02;
    if (!ros::param::get(ns + "/down_voxel_size", down_voxel_size)) {
        ROS_WARN("No down_voxel_size specified. Looking for %s. Default is 'down_voxel_size'.",
                 (ns + "/down_voxel_size").c_str());
    }

    T_B_W = tf::Transform::getIdentity();

    downSizeFilter.setLeafSize(down_voxel_size, down_voxel_size, down_voxel_size);

    reg_pub = nh_.advertise<sensor_msgs::PointCloud2>("/registered_point_cloud", 1);
    dwz_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/dwz_scan_cloud", 1);

    local_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/jackal/velodyne/velodyne_points", 1));
    local_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/car/odom", 100));
    sync_local_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
            SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_));
    sync_local_cloud_odom_->registerCallback(boost::bind(&SlamOutput::pointCloudOdomCallback, this, _1, _2));
}

void SlamOutput::pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &scanIn,
                                        const nav_msgs::OdometryConstPtr &odomIn) {
    // 1. 提取base_link在世界坐标系下的位姿
    const auto& pose = odomIn->pose.pose;
    Eigen::Quaterniond q_base(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d t_base(pose.position.x, pose.position.y, pose.position.z);

    // 2. 第一次回调时，记录lidar到base_link的静态变换T_B_L
    static bool got_T_B_L = false;
    static Eigen::Isometry3d T_B_L = Eigen::Isometry3d::Identity();
    if (!got_T_B_L) {
        // lidar坐标系在base_link下的变换
        Eigen::Vector3d lidar_in_base = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q_lidar_in_base = Eigen::Quaterniond::Identity();
        if (getTransform(scanIn->header.frame_id, "base_link", scanIn->header.stamp, lidar_in_base, q_lidar_in_base)) {
            T_B_L.linear() = q_lidar_in_base.toRotationMatrix();
            T_B_L.translation() = lidar_in_base;
            got_T_B_L = true;
            ROS_INFO("Got T_B_L from tf.");
        } else {
            ROS_WARN("Failed to get T_B_L from tf, skip this frame.");
            return;
        }
    }

    // 3. 点云消息转PCL
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*scanIn, cloud);

    // 4. 去除无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);

    // 5. 降采样
    downSizeFilter.setInputCloud(cloud.makeShared());
    pcl::PointCloud<pcl::PointXYZI> cloud_ds;
    downSizeFilter.filter(cloud_ds);

    // 6. 计算lidar在世界下的变换
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    T_W_B.linear() = q_base.toRotationMatrix();
    T_W_B.translation() = t_base;
    Eigen::Isometry3d T_W_L = T_W_B * T_B_L;

    // 7. 点云变换到世界坐标系
    pcl::PointCloud<pcl::PointXYZI> cloud_world;
    pcl::transformPointCloud(cloud_ds, cloud_world, T_W_L.matrix());

    // 8. 发布
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_world, cloud_msg);
    cloud_msg.header.stamp = scanIn->header.stamp;
    cloud_msg.header.frame_id = frame_id; // 一般为"world"
    reg_pub.publish(cloud_msg);
}
