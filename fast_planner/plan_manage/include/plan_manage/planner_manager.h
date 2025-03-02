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



#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/kinodynamic_astar_2D.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

namespace fast_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, Eigen::Vector3d start_yaw, Eigen::Vector3d end_yaw,
                         vector<Eigen::Vector3d>& start_change);
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);//topo用
  bool topoReplan(bool collide);//topo用

  //yaw角单独规划，规划的结果放到local_data_里面
  void planYaw(const Eigen::Vector3d& start_yaw, const Eigen::Vector3d& end_yaw = Eigen::Vector3d::Zero());//

  void initPlanModules(ros::NodeHandle& nh);//初始化规划的模块
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);//topo用

  bool checkTrajCollision(double& distance);

  LocalTrajData local_data_;   //存放的是最近一次规划的结果，位置和yaw角的0阶导、1阶导、2阶导都用B样条曲线表示
  GlobalTrajData global_data_; //给topo用的

  PlanParameters pp_;                       // 规划相关的参数
  MidPlanData plan_data_;                   // kino用来存放轨迹，和yaw角的规划，但是放yaw角的规划在这里由什么用呢？
  EDTEnvironment::Ptr edt_environment_;
  ObjPredictor::Ptr obj_predictor_;
  bool only2D()
  {
    return this->only2D_;
  }
  void resetTopoPathContainer()
  {
    topo_prm_->resetPathContainer();
  }
  void topoUpdate(std::vector<Eigen::Vector3d>& start_change);
  
private:
  /* main planning algorithms & modules */
  SDFMap::Ptr sdf_map_;

  unique_ptr<Astar> geo_path_finder_;
  // unique_ptr<KinodynamicAstar> kino_path_finder_;
  unique_ptr<KinodynamicAstar2D> kino_path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  
  //每一条轨迹是一个B样条曲线，对应一个优化器。如果是topo的话，会生成很多个轨迹，所以需要多个优化器
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;
  string src_file = "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/test/data/";
  std::ofstream record_file_;
  void updateTrajInfo();//

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  void selectBestTraj(NonUniformBspline& traj);
  void refineTraj(NonUniformBspline& best_traj, double& time_inc);
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  // heading planning
  void calcNextYaw(const double& last_yaw, double& yaw);

  void saveBsplineInfo(NonUniformBspline& bspline, const string& feature, bool isUniform = true);
  void saveYawBsplineInfo(NonUniformBspline& bspline, const string& feature, bool isUniform = true);
  void saveGeoPathInfo(const vector<Eigen::Vector3d> & path, const string & feature, const double & delta_t);
  // !SECTION stable

  // SECTION developing

  bool only2D_ = false;
public:
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION
};
}  // namespace fast_planner

#endif