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



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include "common_srvs/reset_env.h"

using std::vector;

namespace fast_planner {

class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  ros::NodeHandle nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(ros::NodeHandle& node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3};

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  // 到终点no_replan_thresh_的话，就不会重新规划了；离起点replan_thresh_内的话，也不进行重规划
  // 速度增加的话，这两个值也应该增加
  double no_replan_thresh_, replan_thresh_; 
  double waypoints_[50][3];
  int waypoint_num_;//手动设置的waypoint数目，如果target_type_是2的话，就会执行这些waypoint

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  double last_plan_time_;
  bool use_teb_ = false;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;              // start state
  Eigen::Vector3d start_yaw_, end_yaw_;                                     // 是yaw, dotyaw, dotdotyaw
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;                                                // 在设置了一系列waypt时，指明当前要前往的waypt序号
  vector<Eigen::Vector3d> start_change_;                          // 以0.2的频率记录里程计信息，给topo图更新start用

  /* ROS utils */
  ros::NodeHandle node_;
  /* exec_timer_和safety_timer_是周期性调用，一个check状态机，一个check碰撞*/
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_, odom_record_timer_;
  ros::Timer topo_update_timer_;
  /* waypoint_sub_接收rviz选择的终点，odom_sub_更新状态 */
  ros::Subscriber waypoint_sub_, odom_sub_;
  /* replan_pub_是告知需要重规划，new_pub_没有使用，好像是用来重置什么状态，bspline_pub_发布生成的B样条曲线*/
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void odomRecordCallback(const ros::TimerEvent& e);
  void TopoContainerUpdate(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);


  ros::ServiceServer reset_srv_;
  bool reset_env(common_srvs::reset_env::Request &req, common_srvs::reset_env::Response &res);

public:
  KinoReplanFSM(/* args */) {
  }
  ~KinoReplanFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif