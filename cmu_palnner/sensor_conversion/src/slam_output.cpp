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
    std::string car_odom_topic, local_cloud_sub_topic;
    // 这个地方有问题，frame_id是SLAM的world坐标系（初始），不是全局的world坐标系
    nh_private.param("sensor_odom_parent_frame", slam_frame_id, std::string("camera_init"));
    nh_private.param("car_odom_topic", car_odom_topic, std::string("/car_odom"));
    nh_private.param("sensor_odom_topic", sensor_odom_topic, std::string("/jackal/velodyne/gazebo_gt/odometry"));
    nh_private.param("local_cloud_sub_topic", local_cloud_sub_topic, std::string("/jackal/velodyne/velodyne_points"));
    nh_private.param("lidar_frame", lidar_frame, std::string("jackal/velodyne/VLP_16_base_link"));
    nh_private.param("is_sim", is_sim, false);
    nh_private.param("offset_gazebo_world_x", offset_gazebo_world_x, 0.0);
    nh_private.param("offset_gazebo_world_y", offset_gazebo_world_y, 0.0);
    nh_private.param("down_voxel_size", down_voxel_size, 0.05);

    if (!is_sim)
    {
        offset_gazebo_world_x = 0.0;
        offset_gazebo_world_y = 0.0;
    }
    T_B_L = Eigen::Isometry3d::Identity();

    if (is_sim)
    {
        Eigen::Vector3d lidar_in_base = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q_lidar_in_base = Eigen::Quaterniond::Identity();
        while (ros::ok() && !getTransform(lidar_frame, "base_link", ros::Time(0), lidar_in_base, q_lidar_in_base)) {
            ROS_WARN("Waiting for TF: %s -> %s ...", lidar_frame.c_str(), "base_link");
            ros::Duration(0.5).sleep();
        }
        T_B_L.linear() = q_lidar_in_base.toRotationMatrix();
        T_B_L.translation() = lidar_in_base;    
    
    }else
    {
        Eigen::Vector3d t_base_in_lidar(0.0173, 0.0, -0.6322);
        Eigen::Quaterniond q_base_in_lidar(0.98480, 0.0, -0.17365, 0.0); // 四元数 (w, x, y, z)
        Eigen::Isometry3d T_L_B = Eigen::Isometry3d::Identity();
        T_L_B.linear() = q_base_in_lidar.toRotationMatrix();
        T_L_B.translation() = t_base_in_lidar;
        T_B_L = T_L_B.inverse(); // base_link在lidar_link下的变换   
        base_link_z_ = 0.0;      // 因为base_link在初始化的时候，和world坐标系是重合的（不考虑z和y的offset）
    }



    T_W_SLAM = Eigen::Isometry3d::Identity();
    Eigen::Vector3d slam_in_world = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_slam_in_world = Eigen::Quaterniond::Identity();
    while (ros::ok() && !getTransform(slam_frame_id, "world", ros::Time(0), slam_in_world, q_slam_in_world)) {
        ROS_WARN("Waiting for TF: %s -> %s ...", slam_frame_id.c_str(), "world");
        ros::Duration(0.5).sleep();
    }
    T_W_SLAM.linear() = q_slam_in_world.toRotationMatrix();
    T_W_SLAM.translation() = slam_in_world;


    downSizeFilter.setLeafSize(down_voxel_size, down_voxel_size, down_voxel_size);

    reg_pub = nh_.advertise<sensor_msgs::PointCloud2>("/registered_point_cloud", 1);
    dwz_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/dwz_scan_cloud", 1);
    odom_pub = nh_.advertise<nav_msgs::Odometry>(car_odom_topic, 1);

    local_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, local_cloud_sub_topic, 10));
    local_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, sensor_odom_topic, 100));
    sync_local_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
            SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_));
    sync_local_cloud_odom_->registerCallback(boost::bind(&SlamOutput::pointCloudOdomCallback, this, _1, _2));
    odom_tf_sub = nh_.subscribe<nav_msgs::Odometry>(sensor_odom_topic, 100, &SlamOutput::odomTfCallback, this);


}

// 现在里程计直接就是lidar_link的里程计了
void SlamOutput::pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &scanIn,
                                        const nav_msgs::OdometryConstPtr &odomIn) {
    // 1. odom表示lidar_link在SLAM系下的位姿和速度
    const auto& pose = odomIn->pose.pose;
    Eigen::Quaterniond q_lidar(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d t_lidar(pose.position.x - offset_gazebo_world_x, pose.position.y - offset_gazebo_world_y, pose.position.z);


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
    Eigen::Isometry3d T_SLAM_L = Eigen::Isometry3d::Identity();
    T_SLAM_L.linear() = q_lidar.toRotationMatrix();
    T_SLAM_L.translation() = t_lidar;

    Eigen::Isometry3d T_W_B = T_W_SLAM * T_SLAM_L * T_B_L.inverse();

    if (!is_sim)
    {
        Eigen::Matrix3d R_W_B = T_W_B.rotation();
        double yaw = atan2(R_W_B(1, 0), R_W_B(0, 0)); // 提取 yaw 角
        Eigen::Matrix3d R_corrected = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // 构造矫正后的 T_W_B
        Eigen::Isometry3d T_W_B_corrected = T_W_B;
        T_W_B_corrected.linear() = R_corrected;
        T_W_B_corrected.translation().z() = base_link_z_;

        // 将矫正后的 T_W_B 转换回 T_SLAM_B
        Eigen::Isometry3d T_SLAM_B_corrected = T_W_SLAM.inverse() * T_W_B_corrected;

        // 4. 更新 T_B_L
        auto T_L_B = T_SLAM_L.inverse() * T_SLAM_B_corrected;
        T_B_L = T_L_B.inverse();
        T_W_B = T_W_B_corrected;
        // std::cout << "Correct: \n" <<  (T_W_B_corrected * T_W_B.inverse()).matrix() << std::endl;    

        // 发布lidar_link->base_link的tf
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(
            T_L_B.translation().x(),
            T_L_B.translation().y(),
            T_L_B.translation().z()));
        Eigen::Quaterniond q(T_L_B.rotation());
        tf::Quaternion tf_quat(q.x(), q.y(), q.z(), q.w());
        tf_transform.setRotation(tf_quat);

        tf::StampedTransform stamped_tf(
            tf_transform,
            scanIn->header.stamp,  // 当前时间戳
            scanIn->header.frame_id,       // 父坐标系
            base_frame       // 子坐标系
        );
        broadcaster.sendTransform(stamped_tf);
    }

    // --- twist转换 ---
    // lidar_link在SLAM系下的速度
    const auto& twist = odomIn->twist.twist;
    Eigen::Vector3d v_lidar(twist.linear.x, twist.linear.y, twist.linear.z);
    Eigen::Vector3d w_lidar(twist.angular.x, twist.angular.y, twist.angular.z);

    // 1. lidar_link -> base_link
    Eigen::Matrix3d R_B_L = T_B_L.linear();
    Eigen::Vector3d v_base = R_B_L * v_lidar + R_B_L * w_lidar.cross(-T_B_L.translation());
    Eigen::Vector3d w_base = R_B_L * w_lidar;

    // 2. base_link -> world
    Eigen::Matrix3d R_W_B = (T_W_SLAM * T_SLAM_L * T_B_L.inverse()).linear();
    Eigen::Vector3d v_world = R_W_B * v_base;
    Eigen::Vector3d w_world = R_W_B * w_base;

    // 7. 点云变换到世界坐标系
    pcl::PointCloud<pcl::PointXYZI> cloud_world;
    Eigen::Isometry3d T_W_L = T_W_SLAM * T_SLAM_L;  
    pcl::transformPointCloud(cloud_ds, cloud_world, T_W_L.matrix());

    // 8. 发布
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_world, cloud_msg);
    cloud_msg.header.stamp = scanIn->header.stamp;
    cloud_msg.header.frame_id = world_frame_id; // 
    reg_pub.publish(cloud_msg);

    // 9. 发布里程计
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = odomIn->header.stamp;
    odom_msg.header.frame_id = world_frame_id; // 
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = T_W_B.translation().x();
    odom_msg.pose.pose.position.y = T_W_B.translation().y();
    odom_msg.pose.pose.position.z = T_W_B.translation().z();
    Eigen::Quaterniond q_W_B(T_W_B.rotation());
    odom_msg.pose.pose.orientation.x = q_W_B.x();
    odom_msg.pose.pose.orientation.y = q_W_B.y();
    odom_msg.pose.pose.orientation.z = q_W_B.z();
    odom_msg.pose.pose.orientation.w = q_W_B.w();
    odom_msg.twist.twist.linear.x = v_world.x();
    odom_msg.twist.twist.linear.y = v_world.y();
    odom_msg.twist.twist.linear.z = v_world.z();
    odom_msg.twist.twist.angular.x = w_world.x();
    odom_msg.twist.twist.angular.y = w_world.y();
    odom_msg.twist.twist.angular.z = w_world.z();

    odom_pub.publish(odom_msg);
}

// 这个是给gazebo仿真用的
void SlamOutput::odomTfCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    // 1. 提取base_link在世界坐标系下的位姿
    const auto& pose = odom_msg->pose.pose;
    Eigen::Quaterniond q_lidar(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d t_lidar(pose.position.x - offset_gazebo_world_x, pose.position.y - offset_gazebo_world_y, pose.position.z);
    // 2. 将odom转换为tf发布出来
    if (is_sim) 
    {   
        static double last_tf_pub_time = ros::Time::now().toSec();    
        double now = ros::Time::now().toSec();
        // if (now - last_tf_pub_time > 0.01) 
        { // 10Hz限制
            last_tf_pub_time = now;
            // 构造T_SLAM_L
            Eigen::Isometry3d T_SLAM_L = Eigen::Isometry3d::Identity();
            T_SLAM_L.linear() = q_lidar.toRotationMatrix();
            T_SLAM_L.translation() = t_lidar;

            // 计算T_SLAM_B
            Eigen::Isometry3d T_SLAM_B = T_SLAM_L * T_B_L.inverse();

            // 发布world->base_link的tf
            tf::Transform tf_transform;
            tf_transform.setOrigin(tf::Vector3(
                T_SLAM_B.translation().x(),
                T_SLAM_B.translation().y(),
                T_SLAM_B.translation().z()));
            Eigen::Quaterniond q(T_SLAM_B.rotation());
            tf::Quaternion tf_quat(q.x(), q.y(), q.z(), q.w());
            tf_transform.setRotation(tf_quat);

            tf::StampedTransform stamped_tf(
                tf_transform,
                odom_msg->header.stamp,
                slam_frame_id,    // 父坐标系
                base_frame   // 子坐标系
            );

            broadcaster.sendTransform(stamped_tf);
        }
        // Eigen::Isometry3d T_SLAM_L = Eigen::Isometry3d::Identity();
        // T_SLAM_L.linear() = q_lidar.toRotationMatrix();
        // T_SLAM_L.translation() = t_lidar;

        // Eigen::Isometry3d T_W_B = T_W_SLAM * T_SLAM_L * T_B_L.inverse();
        // // --- twist转换 ---
        // // lidar_link在SLAM系下的速度
        // const auto& twist = odom_msg->twist.twist;
        // Eigen::Vector3d v_lidar(twist.linear.x, twist.linear.y, twist.linear.z);
        // Eigen::Vector3d w_lidar(twist.angular.x, twist.angular.y, twist.angular.z);

        // // 1. lidar_link -> base_link
        // Eigen::Matrix3d R_B_L = T_B_L.linear();
        // Eigen::Vector3d v_base = R_B_L * v_lidar + R_B_L * w_lidar.cross(-T_B_L.translation());
        // Eigen::Vector3d w_base = R_B_L * w_lidar;

        // // 2. base_link -> world
        // Eigen::Matrix3d R_W_B = (T_W_SLAM * T_SLAM_L * T_B_L.inverse()).linear();
        // Eigen::Vector3d v_world = R_W_B * v_base;
        // Eigen::Vector3d w_world = R_W_B * w_base;
        // // 9. 发布里程计
        // nav_msgs::Odometry odom_msg;
        // odom_msg.header.stamp = odom_msg.header.stamp;
        // odom_msg.header.frame_id = world_frame_id; // 
        // odom_msg.child_frame_id = "base_link";
        // odom_msg.pose.pose.position.x = T_W_B.translation().x();
        // odom_msg.pose.pose.position.y = T_W_B.translation().y();
        // odom_msg.pose.pose.position.z = T_W_B.translation().z();
        // Eigen::Quaterniond q_W_B(T_W_B.rotation());
        // odom_msg.pose.pose.orientation.x = q_W_B.x();
        // odom_msg.pose.pose.orientation.y = q_W_B.y();
        // odom_msg.pose.pose.orientation.z = q_W_B.z();
        // odom_msg.pose.pose.orientation.w = q_W_B.w();
        // odom_msg.twist.twist.linear.x = v_world.x();
        // odom_msg.twist.twist.linear.y = v_world.y();
        // odom_msg.twist.twist.linear.z = v_world.z();
        // odom_msg.twist.twist.angular.x = w_world.x();
        // odom_msg.twist.twist.angular.y = w_world.y();
        // odom_msg.twist.twist.angular.z = w_world.z();

        // odom_pub.publish(odom_msg);
    }
}

