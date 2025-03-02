#include <ros/ros.h>
#include <Eigen/Geometry>
#include "bspline/non_uniform_bspline.h"
#include "tcp_communication.h"
#include "nav_msgs/Odometry.h"
#include "/home/bhrqhb/catkin_ws_fast_planner/devel/include/plan_manage/Bspline.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"


ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_real_pub, traj_cmd_pub;

nav_msgs::Odometry odom;

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// tcpip
TCPServer server_(PORT);

//traj_real_存放的是所有的历史轨迹，traj_cmd_存放的是所有的历史期望轨迹
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

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
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
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

    CustomData data;
    data.value1 = vel.norm();
    data.value2 = yaw;

    server_.sendData(data);
    // std::cout << "[Cmd Sender] Sended data: ";
    // std::cout << "Value1: " << data.value1;
    // std::cout << ", Value2: " << data.value2 << std::endl;

  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_sender");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  // ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  if (!server_.start()) {
      std::cerr << "Server failed to start" << std::endl;
      return -1;
  }

  std::cout << "Waiting for client..." << std::endl;
  server_.waitForClient();


  ros::Duration(1.0).sleep();

  ROS_WARN("[Cmd Sender]: ready.");

  ros::spin();

  return 0;
}
