#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

class PointCloudAccumulator
{
public:
    PointCloudAccumulator()
    {
        // 订阅和发布
        cloud_sub_ = nh_.subscribe("/sdf_map/depth_cloud", 1, &PointCloudAccumulator::cloudCallback, this);
        odom_sub_ = nh_.subscribe("/car_odom", 1, &PointCloudAccumulator::odomCallback, this);
        accum_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);
        global_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_point_cloud", 1);
        
        // 定时发布全局点云
        global_cloud_timer_ = nh_.createTimer(ros::Duration(1.0), &PointCloudAccumulator::publishGlobalCloud, this);
        
        // 初始化点云积累容器
        accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);


        start_time_ = ros::Time::now().toSec();
        // 载入全局点云（假设全局点云是预先存储的）
        ROS_WARN("Loading global point cloud...");
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/bhrqhb/EGVG/REUSE_downsampled.pcd", *global_cloud_) == -1) {
            ROS_ERROR("Couldn't load global cloud!");
        } else {
            ROS_INFO("Loaded global point cloud.");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 处理里程计数据
        static int idx = 0;
        if (idx++ >= 1) {
            accum_distance_ += sqrt(pow(msg->pose.pose.position.x - odom_position_.x, 2) +
                                    pow(msg->pose.pose.position.y - odom_position_.y, 2));
        }
        accum_time_ = ros::Time::now().toSec() - start_time_;
        odom_position_ = msg->pose.pose.position;
        double avg_speed = accum_distance_ / accum_time_;
        ROS_WARN_STREAM("Accumulated Time: " << accum_time_ << "s, Accumulated Distance: " << accum_distance_ << " m, Average Speed: " << avg_speed << " m/s");
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // 将ROS点云转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        // 过滤掉7米以外的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : input_cloud->points)
        {
            if (point.z > 3.0) continue; 
            double distance = sqrt(pow(point.x - odom_position_.x, 2) +
                                   pow(point.y - odom_position_.y, 2));
            if (distance <= 7.0)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        // 将过滤后的点云添加到积累点云中
        *accumulated_cloud_ += *filtered_cloud;

        // 检查积累点云是否为空
        if (accumulated_cloud_->empty()) {
            ROS_WARN("Accumulated point cloud is empty!");
        }

        // 进行降采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(accumulated_cloud_);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);  // 设置体素大小（降采样参数）
        voxel_grid.filter(*accumulated_cloud_);

        // 将处理后的点云发布出去
        pcl::toROSMsg(*accumulated_cloud_, accumulated_cloud_msg_);
        accumulated_cloud_msg_.header.stamp = ros::Time::now();
        accumulated_cloud_msg_.header.frame_id = "world";  // 或者你需要的任何frame
        accum_cloud_pub_.publish(accumulated_cloud_msg_);
    }

    void publishGlobalCloud(const ros::TimerEvent&)
    {
        // 每秒发布一次全局点云
        if (global_cloud_->empty()) {
            ROS_WARN("Global point cloud is empty!");
        }
        pcl::toROSMsg(*global_cloud_, global_cloud_msg_);
        global_cloud_msg_.header.stamp = ros::Time::now();
        global_cloud_msg_.header.frame_id = "world";  // 例如世界坐标系
        global_cloud_pub_.publish(global_cloud_msg_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher accum_cloud_pub_, global_cloud_pub_;
    ros::Timer global_cloud_timer_;
    
    // 用于处理的PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;
    
    // 用于发布ROS消息的类型
    sensor_msgs::PointCloud2 accumulated_cloud_msg_;
    sensor_msgs::PointCloud2 global_cloud_msg_;
    
    // 存储里程计位置
    geometry_msgs::Point odom_position_;
    double accum_distance_ = 0.0, accum_time_ = 0.0;
    double start_time_ = 0.0;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_accumulator_node");

    // 创建 PointCloudAccumulator 对象
    PointCloudAccumulator accumulator;

    // 进入ROS循环
    ros::spin();

    return 0;
}
