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



#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H

#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <random>
#include <array>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include "path_searching/dubins.h"
#include "path_searching/astar_2D.h"
#include "threadPool.h"

namespace fast_planner {

class TopoPath {
public:
  std::vector<Eigen::Vector3d> path;
  double length = std::numeric_limits<double>::max();
  bool safty = true;
  std::pair<vector<Eigen::Vector3d>, vector<Eigen::Vector3d>> path_break = {};
  bool selected = false; // 是否是当前正被选中作为引导路径的，path container里面只允许一个true
public:
  TopoPath(const std::vector<Eigen::Vector3d>& path_, double length_)
      : path(path_), length(length_){}
  TopoPath(){}
  bool operator<(const TopoPath& other) const {
    return this->length < other.length;
  }
};

struct RecordData {
  int sample_num;
  int path_num;
  std::vector<double> path_lengths; // 定义为动态大小的向量
  double run_time;
  double preprocess_time;
  // 构造函数，用于初始化向量大小
  RecordData() : sample_num(0), path_num(0), path_lengths(5), run_time(0.0) {}

  void reset() {
    sample_num = 0;
    path_num = 0;
    std::fill(path_lengths.begin(), path_lengths.end(), 0.0); // 将向量中的所有元素填充为0.0
    run_time = 0.0;
    preprocess_time = 0.0;
  }
};


/* ---------- used for iterating all topo combination ---------- */
class TopoIterator {
private:
  /* data */
  vector<int> path_nums_;
  vector<int> cur_index_;
  int combine_num_;
  int cur_num_;

  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  TopoIterator(vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    std::cout << "[Topo]: merged path num: " << combine_num_ << std::endl;
  }
  TopoIterator() {
  }
  ~TopoIterator() {
  }

  bool nextIndex(vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
class GraphNode {
private:
  /* data */

public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode(/* args */) {
  }
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {
  }

  vector<shared_ptr<GraphNode>> neighbors_;
  Eigen::Vector3d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

  typedef shared_ptr<GraphNode> Ptr;
};

class TopologyPRM {
private:
  /* data */
  EDTEnvironment::Ptr edt_environment_;  // environment representation

  // sampling generator
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d sample_r_;
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;
  Eigen::Vector3d map_origin_, map_size_; 

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector3d>> raw_paths_;    //使用DFS搜索出来的，然后选择了前max_raw_path2_个
  vector<vector<Eigen::Vector3d>> short_paths_;  //对raw_paths shorten之后，再判断一次同伦后剩下的路径
  vector<vector<Eigen::Vector3d>> final_paths_;  //最终选出的前n条路径
  vector<vector<Eigen::Vector3d>> dubins_shot_paths_;  //从起点对final_paths进行shot后的路径
  vector<Eigen::Vector3d> start_pts_, end_pts_;
  vector<Eigen::Vector3d> sample_area_;
  vector<bool> dubins_shot_succ_;
  // raycasting
  vector<RayCaster> casters_;
  Eigen::Vector3d offset_;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_;  // DFS搜索的最大路径
  int max_raw_path2_; // 按节点对DFS路径按节点数量排序，选前max_raw_path2_个
  int short_cut_num_;
  Eigen::Vector3d sample_inflate_;
  double resolution_;

  double ratio_to_short_;
  int reserve_num_;

  bool parallel_shortcut_;

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
  vector<vector<Eigen::Vector3d>> searchPaths();
  void shortcutPaths();
  vector<vector<Eigen::Vector3d>> pruneEquivalent(const vector<vector<Eigen::Vector3d>>& paths);
  vector<vector<Eigen::Vector3d>> pruneEquivalentV2(const vector<vector<Eigen::Vector3d>>& paths);
  vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>>& paths, int step);
  void selectShortPathsV2(const vector<vector<Eigen::Vector3d>>& paths);
  /* ---------- helper ---------- */
  inline Eigen::Vector3d getSample();
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);  // find pairs of visibile guard
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                      Eigen::Vector3d pt);  // test redundancy with existing
                                            // connection between two guard
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                 Eigen::Vector3d& pc, int caster_id = 0, int skip_mode = 0);
  bool lineVisib2(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                            Eigen::Vector3d& pc, int caster_id);
  bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
  void pruneGraph();

  void depthFirstSearch(vector<GraphNode::Ptr>& vis);
  void depthFirstSearchBid(vector<GraphNode::Ptr>& vis1, vector<GraphNode::Ptr>& vis2, const bool& dirState);
  vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
  vector<vector<Eigen::Vector3d>> discretizePaths(vector<vector<Eigen::Vector3d>>& path);

  vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path);
  void shortcutPath(const vector<Eigen::Vector3d>& path, int path_id, int iter_num = 2);
  void shortcutPath(const int& path_id, const bool& inFront, const int& iter_num = 4);
  vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);
  bool sameTopoPath(const vector<Eigen::Vector3d>& path1, const vector<Eigen::Vector3d>& path2,
                    double thresh, bool preprocess = false);
  Eigen::Vector3d getOrthoPoint(const vector<Eigen::Vector3d>& path);

  int shortestPath(vector<vector<Eigen::Vector3d>>& paths);

  //QHB: For 2D
  bool only2D_ = true;
  void publishGraph(int sample_num);
  void publishTestPath(const vector<Eigen::Vector3d>& path, const int& pub_num);
  ros::Publisher grah_vis_pub_;
  vector<Eigen::Vector3d> sampled_points_;
  double skip_scale_;
  bool use_skip_;
  bool topo_test_ = false;
  ros::Publisher path_1_pub_, path_2_pub_, path_3_pub_;
  void creatSampleRegion();
  int sampleEdgePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                        const Eigen::Vector3d& p3, const Eigen::Vector3d& p4,
                        vector<vector<Eigen::Vector2d>>& samples);
  void sampleEdge(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                  const int& numSample, const double& numSample_inv,
                  std::vector<Eigen::Vector2d>& samples);
  void generatePoints(const Eigen::Vector2d& pt, 
                    std::vector<Eigen::Vector2d>& points,
                    double l);
  const int numSamples_ = 20;  // 边缘的采样点
  vector<set<vector<int>>> unique_paths_;
  int path_node_num_max_ = 21;
  std::shared_ptr<ThreadPool> threadPool_;
  const double ground_height_ = -0.5;

  const int path_container_size_ = 10;
  const int path_container_size_half_ = 5;
  int path_container_num_ = 0;   // 在path_container里面的有效路径数量，用于判断新路径是怎么插入的
  // 我要把这个容器分为front和back两个，而不是放一个
  std::vector<TopoPath> path_container_front_;
  std::vector<TopoPath> path_container_back_;
  RecordData record_data_;

  std::vector<Eigen::Vector3d> colli_pts_;
  std::vector<Eigen::Vector3d> start_change_;
  bool last_success_ = true;
public:
  double clearance_;
  double clearance_line_;

  TopologyPRM(/* args */);
  ~TopologyPRM();

  void init(ros::NodeHandle& nh);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  void findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, vector<Eigen::Vector3d> start_pts,
                     vector<Eigen::Vector3d> end_pts, list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector3d>>& raw_paths,
                     vector<vector<Eigen::Vector3d>>& filtered_paths,
                     vector<vector<Eigen::Vector3d>>& select_paths);

  double pathLength(const vector<Eigen::Vector3d>& path);
  vector<Eigen::Vector3d> pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num);
  
  void initForTest(ros::NodeHandle& nh);  
  inline void setOnly2D(bool only2D){only2D_ = only2D;}
  inline vector<Eigen::Vector3d> getSampleArea()
  {
    return this->sample_area_;
  }
  void CommonStartEnd(const std::vector<Eigen::Vector3d>& vec1, 
                      const std::vector<Eigen::Vector3d>& vec2, int& sameStart, int& sameEnd);
  void findDubinsShot(const vector<Eigen::Vector3d>& path, const int& path_id,
                      const Eigen::Vector3d& start_state, const double& radius);
  vector<Eigen::Vector3d> findDubinsShots(const Eigen::Vector3d& start_state,
                        const double& radius);
  vector<vector<Eigen::Vector3d>> getPathContainer(const int& label = 0);
  void preprocess();
  void checkPathContainerObstacle();
  void checkPathObstacle(const int& path_id, const bool& inFront);
  bool checkPathObstacle2(const std::vector<Eigen::Vector3d>& onePath);
  bool checkLineCollid(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
                       Eigen::Vector3d& collid_pt, Eigen::Vector3d& break_pt, const double& dis_to_obs);
  vector<Eigen::Vector3d> backtrackFromEnd(const std::vector<Eigen::Vector3d>& path, double dis);
  vector<Eigen::Vector3d> backtrackFromStart(const std::vector<Eigen::Vector3d>& path, double dis);
  void reconnectTopoPaths();
  void reconnectBreakPath(const int& path_id, const bool& inFront);
  unique_ptr<Astar2D> astar2D_path_finder_;
  void setStartChange(vector<Eigen::Vector3d>& start_change);
  void resetPathContainer();
  void updateAllPaths();
  RecordData getRecordData(){return record_data_;}
  void publishTestPath(const vector<Eigen::Vector2i>& path, const int& pub_num);
};

}  // namespace fast_planner

#endif