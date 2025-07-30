/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include "common_srvs/traj_record.h"

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_real_pub, traj_cmd_pub, vel_pub_;
ros::ServiceServer traj_record_pub_;
nav_msgs::Odometry odom;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
bool traj_record_switch_ = false;
bool is_storing_ = false;
std::ofstream traj_cmd_file_, traj_real_file_; 
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;
double time_forward_;
//traj_real_存放的是所有的历史轨迹，traj_cmd_存放的是所有的历史期望轨迹
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void saveOdomToFile(std::ofstream& file, const nav_msgs::Odometry& odom_msg)
{
  double timestamp = odom_msg.header.stamp.toSec();
  const auto& odom_pose =odom_msg.pose.pose;

  // 将数据写入文件，使用TUM轨迹格式保存
  file << std::fixed << std::setprecision(6)
        << timestamp << " "
        << odom_pose.position.x << " " << odom_pose.position.y << " " << -0.5 << " "
        << odom_pose.orientation.x << " " << odom_pose.orientation.y << " " << odom_pose.orientation.z << " " << 
        odom_pose.orientation.w << std::endl;
}

void saveOdomToFile(std::ofstream& file, const Eigen::Vector3d& pos, double yaw, double time_stamp) {
    // 将数据写入文件，使用TUM轨迹格式保存
    file << std::fixed << std::setprecision(6)
         << time_stamp << " "  // 假设你没有时间戳，或者你可以替换为具体的时间戳
         << pos.x() << " " << pos.y() << " " << -0.5 << " "
         << 0.0 << " " << 0.0 << " " << sin(yaw / 2.0) << " " << cos(yaw / 2.0) << std::endl;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  if(id == 1)
    traj_real_pub.publish(mk);
  else
    traj_cmd_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  if(id == 1)
    traj_real_pub.publish(mk);
  else
    traj_cmd_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

// 就是把B样条轨迹存下来，给cmd解算调用
// 具体地说，在这里把位置B样条和yaw B样条都给恢复了
void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  // parse pos traj

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  // double traj_duration_yaw = yaw_traj.getTimeSum();
  //虽然控制点的节点向量不同，但是duration是相同的
  // std::cout << "position duration: " << traj_duration_ << ", yaw duration: " << traj_duration_yaw << std::endl;
  if(traj_record_switch_ == true)
  {
    is_storing_ = true;
  }
  else
  {
    is_storing_ = false;
  }
  receive_traj_ = true;
}

//重置时间
void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

//清空内存
void newCallback(std_msgs::Empty msg) {
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));

  std_msgs::Float64 vel_value;
  geometry_msgs::Twist twist = odom.twist.twist;
  vel_value.data = Eigen::Vector3d(twist.linear.x, twist.linear.y,twist.linear.z).norm();
  vel_pub_.publish(vel_value);
  if(is_storing_) saveOdomToFile(traj_real_file_, msg);
  if (traj_real_.size() > 50000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 5000);
}

// 固定频率发布可视化结果
void visCallback(const ros::TimerEvent& e) {
  displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1), 1);

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

// 固定频率解析B样条轨迹（x,y,z,yaw），然后转化为cmd发出
// 就是计算了期望（一个时间）的位置、速度、加速度和yaw和yaw的一阶导数。通过B样条可以很方便地计算
void cmdCallback(const ros::TimerEvent& e) {
  /* no publishing before receive traj_ */
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  // 为什么要计算当前的这个pos？这是当前时间的轨迹的位置，也就是期望的？为什么不向前计算一点？
  // 因为轨迹就是带时间的，只要每个周期都执行好当前的就可以了。

  Eigen::Vector3d pos, vel, acc, pos_f;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    cout << "[Traj server]: invalid time." << endl;
  }
  if(is_storing_) saveOdomToFile(traj_cmd_file_, pos, yaw, time_now.toSec());

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  auto pos_err = pos_f - pos;
  // if (pos_err.norm() > 1e-3) {
  //   cmd.yaw = atan2(pos_err(1), pos_err(0));
  // } else {
  //   cmd.yaw = last_yaw_;
  // }
  // cmd.yaw_dot = 1.0;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);

  // draw cmd

  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  Eigen::Vector3d dir(cos(yaw), sin(yaw), pos(2));
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 50000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 5000);
}

bool traj_record_trig(common_srvs::traj_record::Request &req, 
                      common_srvs::traj_record::Response &res)
{
  if(req.start_record == true)
  {
    ROS_WARN("Will record traj info when next Bspline come!");
    traj_record_switch_ = true;
  }
  else if(req.start_record == false)
  {
    ROS_WARN("Will STOP record traj info Right now!");
    traj_record_switch_ = false;
    is_storing_ = false;
  }
  res.success = true;
  return 1;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
  std::string file_path;
  std::string default_path = "ChangeToYourPath";
  nh.param("FilePath", file_path, default_path);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_real_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj_real", 10);
  traj_cmd_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj_cmd", 10);
  vel_pub_ = nh.advertise<std_msgs::Float64>("vel_value", 100);
  traj_record_pub_ = nh.advertiseService("/planning/traj_record", traj_record_trig);
  std::string traj_real_file_path = file_path + "test/traj_record/traj_real.txt";
  std::string traj_cmd_file_path = file_path + "test/traj_record/traj_cmd.txt";

  traj_cmd_file_.open(traj_cmd_file_path.c_str(), std::ios::out);
  traj_real_file_.open(traj_real_file_path.c_str(), std::ios::out);
  if(!traj_cmd_file_.is_open() || !traj_real_file_.is_open())
  {
    ROS_ERROR_STREAM("Unable to open file to save traj info! at: " << traj_real_file_path);
  }


  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);//1.5
  last_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  traj_cmd_file_.close();
  traj_real_file_.close();
  return 0;
}
