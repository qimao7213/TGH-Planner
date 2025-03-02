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



#ifndef _ASTAR_H_2D
#define _ASTAR_H_2D

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
// #include "grad_spline/sdf_map.h"
#include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>

#include <path_searching/astar.h>

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class Astar2D {
private:
  /* ---------- main data structure ---------- */
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::vector<NodePtr> path_nodes_;

  /* ---------- record data ---------- */
  EDTEnvironment::Ptr edt_environment_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double lambda_heu_;
  double margin_;
  int allocate_num_;
  double tie_breaker_;
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(const Eigen::Vector3d& pt);
  int timeToIndex(double time);
  void retrievePath(NodePtr end_node);

  /* heuristic function */
  double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

public:
  Astar2D(){};
  ~Astar2D();

  enum { REACH_END = 1, NO_PATH = 2 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  int search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt, double margin,
             bool dynamic = false, double time_start = -1.0);

  void setEnvironment(const EDTEnvironment::Ptr& env);
  std::vector<Eigen::Vector3d> getPath();
  std::vector<NodePtr> getVisitedNodes();

  typedef shared_ptr<Astar2D> Ptr;
};

}  // namespace fast_planner

#endif