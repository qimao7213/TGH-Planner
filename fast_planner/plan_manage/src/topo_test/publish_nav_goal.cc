#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <cstdlib> // for rand() and RAND_MAX
#include <cmath>   // for sqrt
#include <Eigen/Core>
// 生成随机浮点数的辅助函数
double randomDouble(double min, double max) {
    return min + (max - min) * (static_cast<double>(rand()) / RAND_MAX);
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "nav_goal_publisher");
    ros::NodeHandle nh;

    // 创建发布器
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::Publisher obs_pub = nh.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);

    // 发布频率设置为 1Hz
    ros::Rate rate(5);
    std::vector<Eigen::Vector3d> obs_pts(7);
    obs_pts[0] << 0, -1.2, 0;
    obs_pts[1] << 0, -4.1, 0;
    obs_pts[2] << 0, 1.6, 0;
    obs_pts[3] << -2.7, -3, 0;
    obs_pts[4] << -2.7, 0, 0;
    obs_pts[5] << 3, 0, 0;
    obs_pts[6] << 3, 2.7, 0;

    bool pub_obs = false;
    // pub_obs = true;
    // 循环发布 160 次
    for (int i = 0; i < 100; ++i) {
        if (!ros::ok()) {
            break;
        }

        // 创建 PoseStamped 消息
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map"; // 假设目标点在 "map" 坐标系下

        // 设置随机位置 (x, y)
        goal.pose.position.x = randomDouble(-1.0, 1.0); // 随机生成 x 坐标
        goal.pose.position.y = randomDouble(-1.0, 1.0); // 随机生成 y 坐标
        goal.pose.position.z = 0.0; // 平面导航，z 坐标为 0

        // 设置随机方向 (四元数)
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = randomDouble(-1.0, 1.0); // 随机生成 z
        goal.pose.orientation.w = randomDouble(-1.0, 1.0); // 随机生成 w

        // 归一化四元数
        double norm = sqrt(goal.pose.orientation.x * goal.pose.orientation.x +
                           goal.pose.orientation.y * goal.pose.orientation.y +
                           goal.pose.orientation.z * goal.pose.orientation.z +
                           goal.pose.orientation.w * goal.pose.orientation.w);
        goal.pose.orientation.x /= norm;
        goal.pose.orientation.y /= norm;
        goal.pose.orientation.z /= norm;
        goal.pose.orientation.w /= norm;

        // 发布目标点
        ROS_INFO("Publishing goal %d/150: [x: %.2f, y: %.2f]", i + 1, goal.pose.position.x, goal.pose.position.y);
        ROS_INFO_STREAM("Iter: " << i);
        goal_pub.publish(goal);


        int idx = i / 15;
        int remainder = i - idx * 15;
        // if(pub_obs && i > 20 && remainder == 0 && idx <= 8)
        // {
        //     geometry_msgs::PointStamped msg;
        //     msg.header.stamp = ros::Time::now();  // 设置时间戳
        //     msg.header.frame_id = "map";          // 设置参考坐标系
        //     msg.point.x = obs_pts[idx - 2].x();        // 设置 x 坐标
        //     msg.point.y = obs_pts[idx - 2].y() ;        // 设置 y 坐标
        //     msg.point.z = obs_pts[idx - 2].z() ;        // 设置 z 坐标
        //     // 发布消息
        //     obs_pub.publish(msg);       
        // } 
        if(pub_obs && i > 30 && remainder == 0 && idx <= 8)
        {
            geometry_msgs::PointStamped msg;
            msg.header.stamp = ros::Time::now();  // 设置时间戳
            msg.header.frame_id = "map";          // 设置参考坐标系
            msg.point.x = obs_pts[0].x();        // 设置 x 坐标
            msg.point.y = obs_pts[0].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[0].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
            msg.point.x = obs_pts[1].x();        // 设置 x 坐标
            msg.point.y = obs_pts[1].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[1].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
            msg.point.x = obs_pts[2].x();        // 设置 x 坐标
            msg.point.y = obs_pts[2].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[2].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
            msg.point.x = obs_pts[3].x();        // 设置 x 坐标
            msg.point.y = obs_pts[3].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[3].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
        } 
        if(pub_obs && i > 50 && remainder == 0 && idx <= 8)
        {
            geometry_msgs::PointStamped msg;
            msg.header.stamp = ros::Time::now();  // 设置时间戳
            msg.header.frame_id = "map";          // 设置参考坐标系
            msg.point.x = obs_pts[4].x();        // 设置 x 坐标
            msg.point.y = obs_pts[4].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[4].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
            msg.point.x = obs_pts[5].x();        // 设置 x 坐标
            msg.point.y = obs_pts[5].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[5].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
            msg.point.x = obs_pts[6].x();        // 设置 x 坐标
            msg.point.y = obs_pts[6].y() ;        // 设置 y 坐标
            msg.point.z = obs_pts[6].z() ;        // 设置 z 坐标
            obs_pub.publish(msg);    
        } 

        // 等待下一个周期
        rate.sleep();
    }

    ROS_INFO("Finished publishing 160 goals.");
    return 0;
}
