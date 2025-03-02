/**
 * *********************************************************
 *
 * @file: jump_point_search.h
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef JUMP_POINT_SEARCH_H
#define JUMP_POINT_SEARCH_H

#include <queue>
#include <unordered_set>
#include <ros/ros.h>
#include <ros/console.h>
#include "plan_env/edt_environment.h"
#include <Eigen/Eigen>
#include <path_searching/astar.h>

namespace fast_planner
{
/**
 * @brief lass for objects that plan using the Jump Point Search(JPS) algorithm
 */
class JumpPointSearch
{
public:
  enum {NO_PATH = 0, REACH_END = 1};
  JumpPointSearch(){};
  void setEnvironment(const EDTEnvironment::Ptr& env)
  {
    this->edt_environment_ = env;
  }
  void setParam(ros::NodeHandle& nh) {
    nh.param("JPS/resolution", resolution_, -1.0);
    nh.param("JPS/lambda_heu", lambda_heu_, -1.0);
    nh.param("JPS/allocate_num", allocate_num_, -1);
    tie_breaker_ = 1.0 + 1.0 / 10000;

  }
  
  ~JumpPointSearch() {
    for (int i = 0; i < allocate_num_; i++) {
      delete path_node_pool_[i];
    }
  }


  void init() {
    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    edt_environment_->getMapRegion(origin_, map_size_3d_);
    map_size_3d_ += origin_;
    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;
    fail_sign_ = map_size_3d_ + Eigen::Vector3d(1, 1, 1);
    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++) {
      path_node_pool_[i] = new Node;
    }
    expand_motions_ = {
      Eigen::Vector2d(resolution_, resolution_),
      Eigen::Vector2d(resolution_, 0),
      Eigen::Vector2d(resolution_, -resolution_),
      Eigen::Vector2d(0, resolution_),
      Eigen::Vector2d(0, -resolution_),
      Eigen::Vector2d(-resolution_, resolution_), 
      Eigen::Vector2d(-resolution_, 0),
      Eigen::Vector2d(-resolution_, -resolution_)};

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  void reset() {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++) {
      NodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }


  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start);
  Eigen::Vector2d jump(const Eigen::Vector2d& cur_pos, const Eigen::Vector2d& motion);
  bool detectForceNeighbor(const Eigen::Vector2d& cur_pos, const Eigen::Vector2d& motion);
  vector<Eigen::Vector3d> getJpsPath() const {return jps_path_;}
  vector<Eigen::Vector3d> getJpsPathPoints() const {return jps_path_points_;}
  vector<double> getJpsPathDistance() const {return jps_path_distance_;}
  void arrangePath();
  void arrangePath(const vector<Eigen::Vector3d>& guide_path);
  void setZ(double z) {z_ = z;}
private:
  /* ---------- main data structure ---------- */
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::vector<NodePtr> path_nodes_;
  Eigen::Vector3i end_index_;
  EDTEnvironment::Ptr edt_environment_;

  //jps_path_是稀疏的，jps_path_points_是稠密的
  vector<Eigen::Vector3d> jps_path_, jps_path_points_;
  std::vector<double> jps_path_distance_;
  /*----------- parameter -------------*/
  double resolution_, lambda_heu_, margin_,  tie_breaker_;
  double inv_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  Eigen::Vector3d fail_sign_;
  int allocate_num_;
  double z_;

  vector<Eigen::Vector2d> expand_motions_;
  void retrievePath(NodePtr end_node) {
    NodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL) {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  Eigen::Vector3i posToIndex(Eigen::Vector3d pt) {
    Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    return idx;
  }
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) 
  {
    return tie_breaker_ * (x2 - x1).norm();
  }
  double getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2) 
  {
    return tie_breaker_ * (x2 - x1).norm();
  }
  std::vector<Eigen::Vector3d> getPath() {
    vector<Eigen::Vector3d> path;
    for (int i = 0; i < path_nodes_.size(); ++i) {
      path.push_back(path_nodes_[i]->position);
      // std::cout << "position: " <<  path_nodes_[i]->position.transpose() << ", cost: " << path_nodes_[i]->g_score << std::endl;
    }
    return path;
  }
  Eigen::Vector2i stateToIndex2D(Eigen::Vector2d state);
  Eigen::Vector2d indexToState2D(Eigen::Vector2i index);
};
}  // namespace global_planner
#endif  // JUMP_POINT_SEARCH_H
