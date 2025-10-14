#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>
#include "common_srvs/traj_record.h"
#include <std_msgs/Empty.h>

using namespace std;
using bfmt = boost::format;

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
string waypoint_type = string("manual");
bool is_odom_ready;
nav_msgs::Odometry odom;
nav_msgs::Path waypoints;

// series waypoint needed
std::deque<nav_msgs::Path> waypointSegments;
ros::Time trigged_time;

// waypoints 2D
vector<Eigen::Matrix<double, 6, 1>> waypoints_2d_;
int waypoint_2d_num_;
ros::Timer waypt_switch_timer;
bool goal_trig_ = false;
bool goal_trig_last_ = false;
bool reach_waypt_ = false;
int current_wp_ = 0;
ros::Publisher pub_goal_2D_;
ros::ServiceClient traj_record_client_;
ros::Subscriber unknown_check_sub_;
ros::Time goal_pub_time_;

double quaternion_to_yaw(const Eigen::Quaterniond& q) 
{
    // 提取四元数的分量
    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();
    // 计算偏航角
    double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0* (y * y + z * z));
    return yaw; // 返回偏航角，单位为弧度
}

Eigen::Quaterniond yaw_to_quaternion(double yaw) {
    // 计算偏航角对应的四元数
    double half_yaw = yaw / 2.0;
    double w = std::cos(half_yaw);
    double z = std::sin(half_yaw);

    // 创建四元数
    Eigen::Quaterniond quaternion(w, 0, 0, z);
    return quaternion;
}

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

void load_seg(ros::NodeHandle& nh, int segid, const ros::Time& time_base) {
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw;
    double time_from_start;
    ROS_INFO("Getting segment %d", segid);
    ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
    ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
    ROS_ASSERT(nh.getParam(seg_str + "time_from_start", time_from_start));
    ROS_ASSERT(time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    ROS_ASSERT(nh.getParam(seg_str + "x", ptx));
    ROS_ASSERT(nh.getParam(seg_str + "y", pty));
    ROS_ASSERT(nh.getParam(seg_str + "z", ptz));

    ROS_ASSERT(ptx.size());
    ROS_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::Path path_msg;

    path_msg.header.stamp = time_base + ros::Duration(time_from_start);

    double baseyaw = tf::getYaw(odom.pose.pose.orientation);
    
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::PoseStamped pt;
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }

    waypointSegments.push_back(path_msg);
}

void load_waypoints(ros::NodeHandle& nh, const ros::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();
    ROS_ASSERT(nh.getParam("segment_cnt", seg_cnt));
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(nh, i, time_base);
        if (i > 0) {
            ROS_ASSERT(waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
        }
    }
    ROS_INFO("Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = ros::Time::now();
    pub1.publish(waypoints);
    geometry_msgs::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
    // pub2.publish(waypoints);
    waypoints.poses.clear();
}

void publish_waypoints_vis() {
    nav_msgs::Path wp_vis = waypoints;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = ros::Time::now();

    {
        geometry_msgs::Pose init_pose;
        init_pose = odom.pose.pose;
        init_pose.position.z = -0.5;
        poseArray.poses.push_back(init_pose);
    }

    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::Pose p;
        p = it->pose;
        p.position.z = -0.5;
        poseArray.poses.push_back(p);
    }
    pub2.publish(poseArray);
}


// 感觉这个是用来判断机器人是否到某个waypoints，然后切换到下一个的
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    is_odom_ready = true;
    odom = *msg;

    if (waypointSegments.size()) {
        ros::Time expected_time = waypointSegments.front().header.stamp;
        if (odom.header.stamp >= expected_time) {
            waypoints = waypointSegments.front();

            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % trigged_time.toSec();
            for (auto& pose_stamped : waypoints.poses) {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                          pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                          pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                          pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                          pose_stamped.pose.orientation.z << std::endl;
            }
            ROS_INFO_STREAM(ss.str());

            publish_waypoints_vis();
            publish_waypoints();

            waypointSegments.pop_front();
        }
    }
    
    //在这里一直check是否到达waypt
    if(waypoint_type == "waypts-target-2D" && goal_trig_)
    {
        const auto& odom_pose = odom.pose.pose;
        const auto& odom_linear_v = odom.twist.twist.linear;
        int waypt_idx = max(current_wp_ - 1, 0);
        double dis_error = (Eigen::Vector2d(odom_pose.position.x, odom_pose.position.y)
                           - waypoints_2d_[waypt_idx].head(2)).norm();
        double yaw_error = quaternion_to_yaw(Eigen::Quaterniond(odom_pose.orientation.w, odom_pose.orientation.x,
                           odom_pose.orientation.y, odom_pose.orientation.z)) - waypoints_2d_[waypt_idx](5);
        yaw_error = Mod2Pi(yaw_error);
        yaw_error = 0; // 先不考虑yaw误差
        double vel_value = Eigen::Vector3d(odom_linear_v.x, odom_linear_v.y, odom_linear_v.z).norm();
        if((dis_error < 0.2 && std::abs(yaw_error) < 0.1)
            || (dis_error < 0.4 && std::abs(yaw_error) < 0.25 && vel_value < 0.02)
            || (dis_error < 0.4 && std::abs(yaw_error) < 0.8 && vel_value < 0.01)
            )
        {
            
            reach_waypt_ = true;
            if(current_wp_ == waypoint_2d_num_)
            {
                common_srvs::traj_record traj_record_srv;
                traj_record_srv.request.start_record = false;
                traj_record_client_.call(traj_record_srv);
                goal_trig_ = false;
                current_wp_ = -1; // 重置起点之后可以重新call
            }
        }
        // std::cout << "current_wp_: " << current_wp_ << std::endl;
        // std::cout << "waypt_idx: " << waypt_idx <<  ", dis_error: " << dis_error << ", yaw_error: " 
        // << yaw_error << ", vel_value: " << vel_value << ", reach_waypt_:" << reach_waypt_ << std::endl;

    }
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
/*    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }*/

   if(waypoint_type == string("waypts-target-2D") && !goal_trig_)
   {
    goal_trig_ = true;
    current_wp_ = 0;
    common_srvs::traj_record traj_record_srv;
    traj_record_srv.request.start_record = true;
    traj_record_client_.call(traj_record_srv);
    return;
   }

    trigged_time = ros::Time::now(); //odom.header.stamp;
    //ROS_ASSERT(trigged_time > ros::Time(0));

    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));
    
    if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(n, trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        if (msg->pose.position.z > -0.1) {
            // if height > 0, it's a valid goal;
            geometry_msgs::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            ROS_WARN("[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }
    } else if (waypoint_type == string("waypts-target-2D")){
        if (msg->pose.position.z > -0.1) {
            // if height > 0, it's a valid goal;
            geometry_msgs::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            ROS_WARN("[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        if (msg->pose.position.z > 0) {
            // if height > 0, it's a normal goal;
            geometry_msgs::PoseStamped pt = *msg;
            if (waypoint_type == string("noyaw")) {
                double yaw = tf::getYaw(odom.pose.pose.orientation);
                pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            // if 0 > height > -1.0, remove last goal;
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            // if -1.0 > height, end of input
            if (waypoints.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

//目前没有东西可以触发这个。
void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }

    ROS_WARN("[waypoint_generator] Trigger!");
    trigged_time = odom.header.stamp;
    ROS_ASSERT(trigged_time > ros::Time(0));

    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));

    ROS_ERROR_STREAM("Pattern " << waypoint_type << " generated!");
    if (waypoint_type == string("free")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
   } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(n, trigged_time);
    }
}

void wayptSwitchCallback(const ros::TimerEvent& e)
{
    if(!goal_trig_ && !goal_trig_last_) return;
    else if((goal_trig_ && !goal_trig_last_) || reach_waypt_)//收到了rviz的goal信号，这里需要发布一个
    {
        if(current_wp_ < waypoint_2d_num_ && current_wp_ >= 0)
        {
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = ros::Time::now();
            goal.header.frame_id = "world"; // 使用world坐标系
            goal.pose.position.x = waypoints_2d_[current_wp_](0); // x坐标
            goal.pose.position.y = waypoints_2d_[current_wp_](1); // y坐标
            goal.pose.position.z = waypoints_2d_[current_wp_](2); // z坐标，通常设置为0
            double yaw = waypoints_2d_[current_wp_](5);
            goal.pose.orientation.x = 0.0; // 四元数x分量
            goal.pose.orientation.y = 0.0; // 四元数y分量
            goal.pose.orientation.z = sin(0.5 * yaw); // 四元数z分量
            goal.pose.orientation.w = cos(0.5 * yaw); // 四元数w分量
            pub_goal_2D_.publish(goal);
            reach_waypt_ = false;
            std::cout << waypoints_2d_[current_wp_].transpose() << std::endl;
    
            current_wp_++;
            if(current_wp_ == waypoint_2d_num_)
            {
                // goal_trig_ = false;
                ROS_WARN("Have published all waypts, will STOP!");
                goal_pub_time_ = ros::Time::now();
            }            
        }
    }
    goal_trig_last_ = goal_trig_;

}

void unknownCheckCallback(std_msgs::Empty msg)
{
    ros::Time unknown_check_time = ros::Time::now();
    ROS_ERROR_STREAM("The check point is observed by " << 
                    (unknown_check_time - goal_pub_time_).toSec() * 1000 << " ms.");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));
    ros::Subscriber sub1 = n.subscribe("odom", 100, odom_callback);
    ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
    ros::Subscriber sub3 = n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);
    pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);
    pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

    //自己发布预定义的waypoints为/move_base_simple/goal话题
    pub_goal_2D_ = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
    std::vector<double> waypts_x, waypts_y, waypts_yaw;
    n.getParam("waypts_x", waypts_x);
    n.getParam("waypts_y", waypts_y);
    n.getParam("waypts_yaw", waypts_yaw);
    ROS_WARN_STREAM("---Load Waypoints From YAML, num: " << waypts_yaw.size());
    ROS_ASSERT(waypts_x.size() == waypts_y.size() && waypts_yaw.size() == waypts_x.size());
    waypoint_2d_num_ = waypts_x.size();
    waypoints_2d_.resize(waypoint_2d_num_);
    for(int i = 0; i < waypoint_2d_num_; ++i)
    {
        waypoints_2d_[i] << waypts_x[i], waypts_y[i], 0.5, 0.0, 0.0, waypts_yaw[i] * M_PI/180.0;
        // std::cout << "waypoints_2d_[i]: " << waypoints_2d_[i].transpose() << std::endl;
    }
    waypt_switch_timer = n.createTimer(ros::Duration(0.002), wayptSwitchCallback); 
    traj_record_client_ = n.serviceClient<common_srvs::traj_record>("/planning/traj_record");
    unknown_check_sub_ = n.subscribe<std_msgs::Empty>("/sdf_map/check_point_signal", 10, unknownCheckCallback);

    trigged_time = ros::Time(0);

    ros::spin();
    return 0;
}
