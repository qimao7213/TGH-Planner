#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <cmath>
#include <vector>

// TUM格式文件输出路径
std::string OUTPUT_FILE;

// 目标点位姿（用户自定义）
const double off_set_x = 0;
const double off_set_y = 0;

// 误差阈值（到达目标点的判断依据）
const double POSITION_TOLERANCE = 0.15;  // 位置误差阈值（米）

// 记录位姿到TUM文件
std::ofstream pose_file;

// 计算两点之间的欧几里得距离
double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// 定义目标点结构
struct Goal {
    double x, y, z, yaw;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_and_logger");
    ros::NodeHandle nh;
    nh.param<std::string>("output_file", OUTPUT_FILE, "xx/traj_real.txt");
    ros::Rate rate(200);  // 10Hz

    // 多个目标点列表
    std::vector<Goal> goal_list = {
        {5.75 + off_set_x, -0.433 + off_set_y, 1.0, -0.3}
        ,{4.11 + off_set_x,  -4.02 + off_set_y, 1.0, -2.35619}
        // ,{25 + off_set_x,  -5 + off_set_y, 1.0, -1.5707}
        // 可按需继续添加
    };
    int goal_index = 0;

    // 1. 发布目标点
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = goal_list[goal_index].x;
    goal.pose.position.y = goal_list[goal_index].y;
    goal.pose.position.z = goal_list[goal_index].z;

    // 将yaw角转为四元数
    tf::Quaternion q;
    q.setRPY(0, 0, goal_list[goal_index].yaw);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    // 2. 打开TUM格式文件
    pose_file.open(OUTPUT_FILE);
    if (!pose_file.is_open()) {
        ROS_ERROR("Failed to open file: %s", OUTPUT_FILE.c_str());
        return -1;
    }
    ROS_INFO("Started logging robot poses to %s", OUTPUT_FILE.c_str());

    // 3. TF监听器初始化
    tf::TransformListener listener;

    bool goal_pubed = false;
    ROS_INFO("Publishing goal at x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);

    while (ros::ok()) {
        tf::StampedTransform transform;

        try {
            // 获取机器人在map坐标系中的位姿
            listener.lookupTransform("world", "SQ01s", ros::Time(0), transform);
            // listener.lookupTransform("world", "JA01", ros::Time(0), transform);

            double robot_x = transform.getOrigin().x() - off_set_x;
            double robot_y = transform.getOrigin().y() - off_set_y;
            double robot_z = -0.5;

            // 获取时间戳
            double timestamp = ros::Time::now().toSec();

            // 转换四元数
            tf::Quaternion q = transform.getRotation();
            double qx = q.x();
            double qy = q.y();
            double qz = q.z();
            double qw = q.w();

            // 4. 记录位姿到TUM文件
            pose_file << std::fixed << timestamp << " "
                     << robot_x << " " << robot_y << " " << robot_z << " "
                     << qx << " " << qy << " " << qz << " " << qw << std::endl;

            // 5. 判断是否到达当前目标点
            double distance = calculateDistance(robot_x, robot_y,
                                                goal_list[goal_index].x - off_set_x,
                                                goal_list[goal_index].y - off_set_y);
            ROS_INFO("Logged Pose: x=%.2f, y=%.2f, dis_to_goal=%.2f", robot_x, robot_y, distance);
            if (distance < POSITION_TOLERANCE) {
                ROS_INFO("Goal %d reached!", goal_index + 1);
                goal_index++;

                if (goal_index >= goal_list.size()) {
                    ROS_INFO("All goals reached. Stopping logging.");
                    break;
                }

                // 发布下一个目标点
                goal.pose.position.x = goal_list[goal_index].x;
                goal.pose.position.y = goal_list[goal_index].y;
                goal.pose.position.z = goal_list[goal_index].z;

                tf::Quaternion q;
                q.setRPY(0, 0, goal_list[goal_index].yaw);
                goal.pose.orientation.x = q.x();
                goal.pose.orientation.y = q.y();
                goal.pose.orientation.z = q.z();
                goal.pose.orientation.w = q.w();

                goal.header.stamp = ros::Time::now();
                goal_pub.publish(goal);
                ROS_INFO("Publishing goal at x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                goal_pubed = true;
            }

        } catch (tf::TransformException& ex) {
            ROS_WARN("TF Error: %s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();

        if(!goal_pubed)
        {
            goal.header.stamp = ros::Time::now();
            goal_pub.publish(goal); 
            goal_pubed = true;      
        }
    }

    // 6. 关闭文件
    pose_file.close();
    ROS_INFO("Finished logging. File saved to %s", OUTPUT_FILE.c_str());

    return 0;
}

