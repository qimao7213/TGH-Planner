#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <algorithm>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;  // 车辆是否可以双向行驶（前进和后退）
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double speeddownGain = 0.7;  //让减速的时候更平滑
double maxYawRate = 45.0;
double maxYawAccel = 10;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2; 
double slowDwnDisThre = 1.0; 
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;         //记录路径点时的车辆位置
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

Eigen::Vector3d lastest_goal_;
bool arrive_goal_ = false;
bool arrive_goal_pos_ = false;
bool near_goal_ = false;

nav_msgs::Path path;  //这个应该是在全局坐标系下的？得看看是谁发布的，我感觉像是在机器人坐标系下的

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn) {
    odomTime = odomIn->header.stamp.toSec();

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    vehicleZ = odomIn->pose.pose.position.z;

    if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
        stopInitTime = odomIn->header.stamp.toSec();
    }

    if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
         fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
        slowInitTime = odomIn->header.stamp.toSec();
    }
}

tf::TransformListener *tf_listener_;

void pathHandler(const nav_msgs::Path::ConstPtr &pathIn) {
    int pathSize = pathIn->poses.size();
    path.poses.resize(pathSize);
    for (int i = 0; i < pathSize; i++) {
        path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
        path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
        path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }

    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll;
    vehiclePitchRec = vehiclePitch;
    vehicleYawRec = vehicleYaw;

    pathPointID = 0;
    pathInit = true;
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
    joyTime = ros::Time::now().toSec();

    joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed = joySpeedRaw;
    if (joySpeed > 1.0) joySpeed = 1.0;
    if (joy->axes[4] == 0) joySpeed = 0;
    joyYaw = joy->axes[3];
    if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

    if (joy->axes[4] < 0 && !twoWayDrive) {
        joySpeed = 0;
        joyYaw = 0;
    }

    if (joy->axes[2] > -0.1) {
        autonomyMode = false;
    } else {
        autonomyMode = true;
    }
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed) {
    double speedTime = ros::Time::now().toSec();

    if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
        joySpeed = speed->data / maxSpeed;

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }
}

void stopHandler(const std_msgs::Int8::ConstPtr &stop) {
    safetyStop = stop->data;
}

bool suspendVel = false;

void suspendHandler(const std_msgs::Bool::ConstPtr &suspend) {
    suspendVel = suspend->data;
}

void goalHandler(const geometry_msgs::PoseStampedConstPtr& msg)
{
  Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  Eigen::Quaterniond goal_orient;
  goal_orient.w() = msg->pose.orientation.w;
  goal_orient.x() = msg->pose.orientation.x;
  goal_orient.y() = msg->pose.orientation.y;
  goal_orient.z() = msg->pose.orientation.z;
  Eigen::Vector3d rot_x = goal_orient.toRotationMatrix().block(0, 0, 3, 1);
  goal.z() = atan2(rot_x(1), rot_x(0));

  lastest_goal_ = goal;
  arrive_goal_ = false;
  arrive_goal_pos_ = false;
  near_goal_ = false;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pathFollower");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("pubSkipNum", pubSkipNum);
    nhPrivate.getParam("twoWayDrive", twoWayDrive);
    nhPrivate.getParam("lookAheadDis", lookAheadDis);
    nhPrivate.getParam("yawRateGain", yawRateGain);
    nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
    nhPrivate.getParam("speeddownGain", speeddownGain);
    nhPrivate.getParam("maxYawRate", maxYawRate);
    nhPrivate.getParam("maxYawAccel", maxYawAccel);
    nhPrivate.getParam("maxSpeed", maxSpeed);
    nhPrivate.getParam("maxAccel", maxAccel);
    nhPrivate.getParam("switchTimeThre", switchTimeThre);
    nhPrivate.getParam("dirDiffThre", dirDiffThre);
    nhPrivate.getParam("stopDisThre", stopDisThre);
    nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
    nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
    nhPrivate.getParam("inclRateThre", inclRateThre);
    nhPrivate.getParam("slowRate1", slowRate1);
    nhPrivate.getParam("slowRate2", slowRate2);
    nhPrivate.getParam("slowTime1", slowTime1);
    nhPrivate.getParam("slowTime2", slowTime2);
    nhPrivate.getParam("useInclToStop", useInclToStop);
    nhPrivate.getParam("inclThre", inclThre);
    nhPrivate.getParam("stopTime", stopTime);
    nhPrivate.getParam("noRotAtStop", noRotAtStop);
    nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
    nhPrivate.getParam("autonomyMode", autonomyMode);
    nhPrivate.getParam("autonomySpeed", autonomySpeed);
    nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

    tf_listener_ = new tf::TransformListener;

    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odomHandler);

    ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/path", 5, pathHandler);

    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

    ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

    ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);

    ros::Subscriber subSuspend = nh.subscribe<std_msgs::Bool>("/suspend", 5, suspendHandler);

    ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 5, goalHandler);

    ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    geometry_msgs::Twist cmd_vel;

    if (autonomyMode) {
        joySpeed = autonomySpeed / maxSpeed; 

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }

    double cmd_rate = 100.0;
    ros::Rate rate(cmd_rate);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (pathInit) {

            if (arrive_goal_)
            {
                // ROS_WARN("Arrived at goal, waiting for new goal.");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                pubSpeed.publish(cmd_vel);
                status = ros::ok();
                rate.sleep();
                continue;
            }
            double dxGoal = vehicleX - lastest_goal_.x();
            double dyGoal = vehicleY - lastest_goal_.y();
            double dist_to_goal = sqrt(dxGoal * dxGoal + dyGoal * dyGoal);
            if (!arrive_goal_pos_ && dist_to_goal < 0.2) {
                arrive_goal_pos_ = true;
                near_goal_ = true; // 表示已经到了目标周围，后续的终点就直接设置为goal。这是防止arrive_goal_pos_判定之后，又晃来晃去到不了终点
            }
            else
            {
                arrive_goal_pos_ = false;
            }
            if (arrive_goal_pos_) {

                // ROS_WARN("Arrived at goal position, waiting for orientation to match goal orientation.");
                // 停止移动
                cmd_vel.linear.x = 0;

                // 计算朝向差
                double goal_yaw = lastest_goal_.z();
                double yaw_diff = goal_yaw - vehicleYaw;
                // 归一化到[-PI, PI]
                while (yaw_diff > PI) yaw_diff -= 2 * PI;
                while (yaw_diff < -PI) yaw_diff += 2 * PI;

                // 如果朝向差大于阈值，原地旋转
                if (fabs(yaw_diff) > 2.0/57.3) {
                    cmd_vel.angular.z = 0.5 * yawRateGain * yaw_diff;
                    cmd_vel.angular.z = std::max(-1.0, std::min(1.0, cmd_vel.angular.z));
                } else {
                    cmd_vel.angular.z = 0;
                    arrive_goal_ = true; // 这个是最终的判断，这个为true了，就不再发布速度了，只有当新的goal进来才会重置
                }
                pubSpeed.publish(cmd_vel);
                status = ros::ok();
                rate.sleep();
                continue;
            }

            // 当前车辆相对于该path进来时的位置，并且已经转到了path进来时的坐标系下
            float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
                                + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
            float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
                                + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

            int pathSize = path.poses.size();
            // 机器人当前位置和该path终点的关系
            float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
            float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
            float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

            // 找到该路径上，离机器人当前位置0.5m的前向点，作为local goal
            float disX, disY, dis;
            while (pathPointID < pathSize - 1) {
                disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
                disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
                dis = sqrt(disX * disX + disY * disY);
                if (dis < lookAheadDis) {
                    pathPointID++;
                } else {
                    break;
                }
            }
          
            disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
            disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
            dis = sqrt(disX * disX + disY * disY);
            float pathDir = atan2(disY, disX); 

            float dirDiff = vehicleYaw - vehicleYawRec - pathDir; 
            if (dirDiff > PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI) dirDiff += 2 * PI;
            if (dirDiff > PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI) dirDiff += 2 * PI;

            if (twoWayDrive) { 
                double time = ros::Time::now().toSec();
                if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
                    navFwd = false;
                    switchTime = time;
                } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
                    navFwd = true;
                    switchTime = time;
                }
            }

            float joySpeed2 = maxSpeed * joySpeed; 
            if (!navFwd) { 
                dirDiff += PI;
                if (dirDiff > PI) dirDiff -= 2 * PI;
                joySpeed2 *= -1;
            }

            if (fabs(vehicleSpeed) < 2.0 * maxAccel / cmd_rate) vehicleYawRate = -stopYawRateGain * dirDiff;
            else vehicleYawRate = -yawRateGain * dirDiff; 

            if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
            else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0; 

            if (joySpeed2 == 0 && !autonomyMode) {
                vehicleYawRate = maxYawRate * joyYaw * PI / 180.0; 
            } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
                vehicleYawRate = 0;
            }

            if (pathSize <= 1) {
                joySpeed2 = 0;
            } else if (endDis / slowDwnDisThre < joySpeed) {
                joySpeed2 *= endDis / slowDwnDisThre; 
            }

            float joySpeed3 = joySpeed2;
            if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
            else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0)
                joySpeed3 *= slowRate2; 

            if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
                if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / cmd_rate;
                else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / cmd_rate;
            } else { 
                if (vehicleSpeed > 0) {
                    // 减速力度与 dist_to_goal 成正比，防止提前停止
                    double deceleration_factor = std::max(0.1, dist_to_goal / 0.8); // 确保最小减速力度
                    vehicleSpeed -= maxAccel * (dist_to_goal < 0.8 ? 1.6 : 1.0) / cmd_rate;
                } else if (vehicleSpeed < 0) {
                    // 同样对反向速度进行平滑减速
                    double deceleration_factor = std::max(0.1, dist_to_goal / 0.8); // 确保最小减速力度
                    vehicleSpeed += maxAccel * (dist_to_goal < 0.8 ? 1.6 : 1.0) / cmd_rate;
                }
            }

            // if (near_goal_)
            // {
            //     if (vehicleSpeed > 0) vehicleSpeed = 0.5 * dist_to_goal;
            //     else if (vehicleSpeed < 0) vehicleSpeed = -0.5 * dist_to_goal;
            // }

            if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
                vehicleSpeed = 0;
                vehicleYawRate = 0;
            }

            if (safetyStop >= 1) vehicleSpeed = 0;
            if (safetyStop >= 2) vehicleYawRate = 0;

            pubSkipCount--;
            if (pubSkipCount < 0) {
                if (fabs(vehicleSpeed) <= maxAccel / cmd_rate)
                    cmd_vel.linear.x = 0;
                else
                    cmd_vel.linear.x = vehicleSpeed;
                cmd_vel.angular.z = vehicleYawRate;
                if (!suspendVel) {
                    pubSpeed.publish(cmd_vel);
                }
                pubSkipCount = pubSkipNum;
            }
        }
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
