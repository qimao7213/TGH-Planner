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
        // 从参数服务器读取 remap 的参数
        nh_.param<std::string>("odom_name", odom_name_, "/car/odom");
        nh_.param<std::string>("robot_model_name", robot_model_name_, "simple_box");
        nh_.param<std::string>("reference_frame", reference_frame_, "world");
        std::cout << "Odom Name: " << odom_name_ << std::endl;
        std::cout << "Robot Model Name: " << robot_model_name_ << std::endl;
        std::cout << "Reference Frame: " << reference_frame_ << std::endl;
    
        last_odom_stamp = ros::TIME_MAX;
        tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

        // 等待 Gazebo 服务启动
        ros::service::waitForService("/gazebo/set_model_state");
        client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        // 订阅仿真器发布的位姿话题
        sub_pose_ = nh_.subscribe(odom_name_, 10, &GazeboInfoProcess::poseCallback, this);
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
    std::string odom_name_, robot_model_name_, reference_frame_;

    // 回调函数：处理从仿真器接收到的位姿
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {

        last_odom_stamp = msg->header.stamp;
        // 创建 Gazebo 的模型状态消息
        gazebo_msgs::SetModelState set_model_state_srv;
        set_model_state_srv.request.model_state.model_name = robot_model_name_;
        set_model_state_srv.request.model_state.pose = msg->pose.pose;
        set_model_state_srv.request.model_state.pose.position.x += offset_sim_gazebo(0);
        set_model_state_srv.request.model_state.pose.position.y += offset_sim_gazebo(1);
        set_model_state_srv.request.model_state.pose.position.z += offset_sim_gazebo(2);
        set_model_state_srv.request.model_state.reference_frame = reference_frame_;


        // 调用 Gazebo 服务更新模型状态
        if (client_.call(set_model_state_srv)) {
            // ROS_INFO("Updated model position");
        } else {
            ROS_ERROR("Failed to update model position");
        }
    }


};

// 2025.05.17 简化了这个函数，只用来set模型的位姿，不再处理点云和深度图像
int main(int argc, char** argv) {
    ros::init(argc, argv, "control_model_pose");
    ros::NodeHandle nh("~");

    GazeboInfoProcess gazebo_info_process(nh);

    ros::spin();

    return 0;
}