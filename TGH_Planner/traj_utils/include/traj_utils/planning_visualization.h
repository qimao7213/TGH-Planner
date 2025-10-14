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



#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <path_searching/topo_prm.h>
#include <plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include "nav_msgs/Path.h"

using std::vector;
namespace fast_planner {
class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500,
    GUIDE_PATH = 600,
    SEARCH_TREE = 700,
    PERCEPTION_INFO = 800
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,
    GRAPH_EDGE = 100,
    RAW_PATH = 200,
    FILTERED_PATH = 300,
    SELECT_PATH = 400,
    SAMPLE_AREA = 500
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;      // 0
  ros::Publisher topo_pub_;      // 1
  ros::Publisher predict_pub_;   // 2
  ros::Publisher visib_pub_;     // 3, visibility constraints
  ros::Publisher frontier_pub_;  // 4, frontier searching。
  ros::Publisher yaw_pub_;       // 5, yaw trajectory
  ros::Publisher pointcloud_pub_;// 6
  ros::Publisher traj_2D_pub_;   // 7
  ros::Publisher guide_path_pub_;  // 8
  ros::Publisher search_tree_pub_; //9
  ros::Publisher perception_info_pub_; //10
  ros::Publisher topo_path_pub_;   // 11
  ros::Publisher voronoi_topo_path_pub_;  // 12
  vector<ros::Publisher> pubs_;  //

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;
  int last_topo_voronoi_path_num_;
  std_msgs::ColorRGBA color_red_, color_green_, color_blue_, color_pink_, color_yellow_;
public:
  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  PlanningVisualization(ros::NodeHandle& nh);

  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCloudpoint(const vector<Eigen::Vector3d>& pointcloud, int pub_id);

  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);

  // draw a polynomial trajectory
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
                          int id = 0);

  // draw a bspline trajectory
  void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0, int draw_tmp_traj = 0);

  // draw a set of bspline trajectories generated in different phases
  void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
  void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

  // draw topological graph and paths
  void drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
                     const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
                     const Eigen::Vector4d& color3, int id = 0);

  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);
  void drawTopoSampleArea(const vector<Eigen::Vector3d>& corners, double line_width, const Eigen::Vector4d& color);
  void drawTopoVoronoiPaths(vector<vector<Eigen::Vector3d>>& paths, double line_width);


  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
  // 看看这个是怎么使用的
  // DYNAMIC
  void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

  Eigen::Vector4d getColor(double h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // SECTION developing
  void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);
  void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);

  void drawVisitedNodes(const vector<Eigen::Vector3d>& pointcloud);
  void drawGuidePath(const vector<Eigen::Vector3d>& guide_path, const double& size,
                   const Eigen::Vector4d &color);
  void drawSearchTree(const vector<Eigen::Vector4d>& search_tree, const double ground_height, 
                      const Eigen::Vector4d & color);
  void drawPerceptionInfo(const vector<Eigen::Vector3d>& points, const double& size);
};
}  // namespace fast_planner
#endif