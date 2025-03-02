#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

Eigen::Vector3d offset_sim_gazebo;

class GazeboInfoProcess {
public:
    GazeboInfoProcess(ros::NodeHandle& nh) : nh_(nh) {
        double off_set_x, off_set_y, off_set_z;
        nh_.param("map_offset_x", off_set_x, 0.0);
        nh_.param("map_offset_y", off_set_y, 0.0);
        nh_.param("map_offset_z", off_set_z, 0.0);
        offset_sim_gazebo << off_set_x, off_set_y, off_set_z;
        std::cout << off_set_x << off_set_y <<  off_set_z << std::endl;
        cam02body << 0.0, 0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0, 0.0,
                    0.0, -1.0,0.0, 0.3,
                    0.0, 0.0, 0.0, 1.0;

        //init cam2world transformation
        cam2world = Eigen::Matrix4d::Identity();
        last_odom_stamp = ros::TIME_MAX;
        tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

        // 等待 Gazebo 服务启动
        ros::service::waitForService("/gazebo/set_model_state");
        client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        // 订阅仿真器发布的位姿话题
        sub_pose_ = nh_.subscribe("/car/odom", 10, &GazeboInfoProcess::poseCallback, this);
        sub_pointcloud_ = nh_.subscribe("/camera2/depth/points", 10, &GazeboInfoProcess::pointCloudCallback, this);        
        pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/camera_pose", 1000);
        pub_depth_ = nh_.advertise<sensor_msgs::Image>("/pcl_render_node/depth", 1000);
        pub_depth_info_ = nh_.advertise<sensor_msgs::Image>("/pcl_render_node/depth_info", 1000);
        pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera2/depth/points_new", 1000);

        it_ptr_ = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
        // sub_depth_ = it_ptr_->subscribeCamera("/camera2/depth/image_raw", 1, &GazeboInfoProcess::DepthImageCallback, this);
    }

    ~GazeboInfoProcess() {}

private:
    ros::NodeHandle& nh_;
    ros::ServiceClient client_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_pointcloud_;
    image_transport::CameraSubscriber sub_depth_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_depth_info_;
    ros::Publisher pub_point_cloud_;
    std::unique_ptr<image_transport::ImageTransport> it_ptr_;
    Eigen::Matrix4d cam02body, cam2world;
    Eigen::Quaterniond cam2world_quat;

    ros::Time last_odom_stamp;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
    tf2_ros::Buffer tf_buffer_;
    // 回调函数：处理从仿真器接收到的位姿
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {

        last_odom_stamp = msg->header.stamp;
        Eigen::Vector3d robot_position;
        Eigen::Quaterniond robot_pose;
        Eigen::Matrix4d robot_pose2;
        robot_position.x() = msg->pose.pose.position.x;
        robot_position.y() = msg->pose.pose.position.y;
        robot_position.z() = msg->pose.pose.position.z;
        robot_pose.x() = msg->pose.pose.orientation.x;
        robot_pose.y() = msg->pose.pose.orientation.y;
        robot_pose.z() = msg->pose.pose.orientation.z;
        robot_pose.w() = msg->pose.pose.orientation.w;
        robot_pose2.block<3,3>(0,0) = robot_pose.toRotationMatrix();
        robot_pose2(0,3) = robot_position(0);
        robot_pose2(1,3) = robot_position(1);
        robot_pose2(2,3) = robot_position(2);
        cam2world = robot_pose2 * cam02body;
        cam2world_quat = cam2world.block<3,3>(0,0);

        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header = msg->header;
        camera_pose.header.frame_id = "world";
        camera_pose.pose.position.x = cam2world(0,3);
        camera_pose.pose.position.y = cam2world(1,3);
        camera_pose.pose.position.z = cam2world(2,3);
        camera_pose.pose.orientation.w = cam2world_quat.w();
        camera_pose.pose.orientation.x = cam2world_quat.x();
        camera_pose.pose.orientation.y = cam2world_quat.y();
        camera_pose.pose.orientation.z = cam2world_quat.z();
        pub_pose_.publish(camera_pose);

        // 创建 Gazebo 的模型状态消息
        gazebo_msgs::SetModelState set_model_state_srv;
        set_model_state_srv.request.model_state.model_name = "simple_box";
        set_model_state_srv.request.model_state.pose = msg->pose.pose;
        set_model_state_srv.request.model_state.pose.position.x += offset_sim_gazebo(0);
        set_model_state_srv.request.model_state.pose.position.y += offset_sim_gazebo(1);
        set_model_state_srv.request.model_state.pose.position.z += offset_sim_gazebo(2);
        set_model_state_srv.request.model_state.reference_frame = "world";
        

        // 调用 Gazebo 服务更新模型状态
        if (client_.call(set_model_state_srv)) {
            // ROS_INFO("Updated model position");
        } else {
            ROS_ERROR("Failed to update model position");
        }
    }

    void DepthImageCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                            const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg) {
        
        const std::string target_frame = "world";
        // std::cout << image_msg->header.frame_id << std::endl;
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(target_frame, image_msg->header.frame_id,
                                                        ros::Time(image_msg->header.stamp), ros::Duration(0.01));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("[GlobalMapperRos::DepthImageCallback] %s", ex.what());
            return;
        }

        Eigen::Affine3d eigen_transform, cam2world2;
        eigen_transform = tf2::transformToEigen(transform_stamped);

        cam2world = eigen_transform * cam02body;
        cam2world2.matrix() = cam2world;
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header = image_msg->header;
        camera_pose.header.frame_id = target_frame;
        camera_pose.pose.position.x = cam2world2(0,3);
        camera_pose.pose.position.y = cam2world2(1,3);
        camera_pose.pose.position.z = cam2world2(2,3);
        Eigen::Quaterniond quaternion(cam2world2.rotation());
        camera_pose.pose.orientation.w = quaternion.w();
        camera_pose.pose.orientation.x = quaternion.x();
        camera_pose.pose.orientation.y = quaternion.y();
        camera_pose.pose.orientation.z = quaternion.z();
        pub_pose_.publish(camera_pose);

        auto updated_image = *image_msg;
        auto updated_camera_info = *camera_info_msg;
        updated_image.header.stamp = image_msg->header.stamp;
        updated_camera_info.header.stamp = image_msg->header.stamp;

        // pub_depth_.publish(updated_image);
        // pub_depth_info_.publish(updated_camera_info);

        // ROS_INFO("Published depth image and camera info with updated timestamp.");
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& depth_point_cloud)
    {
        const std::string target_frame = "world";
        // std::cout << image_msg->header.frame_id << std::endl;
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(target_frame, "camera_link",
                                                        ros::Time(depth_point_cloud->header.stamp), ros::Duration(0.01));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("[GlobalMapperRos::DepthImageCallback] %s", ex.what());
            return;
        }
        Eigen::Affine3d eigen_transform, cam2world2;
        eigen_transform = tf2::transformToEigen(transform_stamped);        

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*depth_point_cloud, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, cam02body);

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*transformed_cloud, output_msg);
        output_msg.header.frame_id = "camera_link";  
        output_msg.header.stamp = ros::Time::now();

        pub_point_cloud_.publish(output_msg);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_model_pose");
    ros::NodeHandle nh("~");

    GazeboInfoProcess gazebo_info_process(nh);

    ros::spin();

    return 0;
}