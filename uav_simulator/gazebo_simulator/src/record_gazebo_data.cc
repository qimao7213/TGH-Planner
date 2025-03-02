#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <cmath>

// TUM格式文件输出路径
const std::string OUTPUT_FILE = "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/test/traj_record/traj_real.txt";

// 目标点位姿（用户自定义）
const double off_set_x = 20;
const double off_set_y = 0;
const double GOAL_RVIZ_X = 17;
const double GOAL_RVIZ_Y = -10;
const double GOAL_X = GOAL_RVIZ_X + off_set_x;  // 目标x坐标
const double GOAL_Y = GOAL_RVIZ_Y + off_set_y;  // 目标y坐标
const double GOAL_Z = 1.0;  // 目标z坐标
const double GOAL_YAW = -1.5707;  // 目标偏航角，弧度

// 误差阈值（到达目标点的判断依据）
const double POSITION_TOLERANCE = 0.15;  // 位置误差阈值（米）

// 记录位姿到TUM文件
std::ofstream pose_file;

// 计算两点之间的欧几里得距离
double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_and_logger");
    ros::NodeHandle nh;
    ros::Rate rate(200);  // 10Hz

    // 1. 发布目标点
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = GOAL_X;
    goal.pose.position.y = GOAL_Y;
    goal.pose.position.z = GOAL_Z;

    // 将yaw角转为四元数
    tf::Quaternion q;
    q.setRPY(0, 0, GOAL_YAW);
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

    bool goal_reached = false;
    bool goal_pubed = false;
    // goal_pub.publish(goal);
    // ros::Duration(1.0).sleep();
    // 发布目标位姿
    ROS_INFO("Publishing goal at x: %.2f, y: %.2f", GOAL_X, GOAL_Y);

    while (ros::ok() && !goal_reached) {
        tf::StampedTransform transform;
        // if(!goal_pubed)
        // {
        //     goal_pub.publish(goal); 
        //     goal_pubed = true;      
        // }

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

            // 5. 判断是否到达目标点
            double distance = calculateDistance(robot_x, robot_y, GOAL_RVIZ_X, GOAL_RVIZ_Y);
            ROS_INFO("Logged Pose: x=%.2f, y=%.2f, dis_to_goal=%.2f", robot_x, robot_y, distance);
            if (distance < POSITION_TOLERANCE) {
                ROS_INFO("Goal reached! Stopping logging.");
                goal_reached = true;
            }

        } catch (tf::TransformException& ex) {
            ROS_WARN("TF Error: %s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
        if(!goal_pubed)
        {
            goal_pub.publish(goal); 
            goal_pubed = true;      
        }
    }

    // 6. 关闭文件
    pose_file.close();
    ROS_INFO("Finished logging. File saved to %s", OUTPUT_FILE.c_str());

    return 0;
}

