
#pragma once

#include <memory>

#include <boost/thread.hpp>
#include "costmap_2d/GenericPluginConfig.h"
#include "costmap_2d/cost_values.h"
#include "costmap_2d/layer.h"
#include "costmap_2d/layered_costmap.h"
#include "dvr/dynamicvoronoi.h"
#include "dvr/GVG.h"
#include "plan_env/raycast.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>  // NOLINT
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <traj_utils/planning_visualization.h>
using namespace gvg;
using std::vector;
namespace DynaVoro
{
class VoronoiLayer
{
public:
  VoronoiLayer(ros::NodeHandle& nh);

  const DynamicVoronoi& getVoronoi() const;
  boost::mutex& getMutex();
  bool plan(Eigen::Vector3d start, Eigen::Vector3d goal, double start_yaw = 0.0);


private:
  void publishVoronoiGrid();
  void publishDistanceCloud();
  void publishGVG(const std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>& gvg);
  void publishPath(const std::vector<IntPoint>& path_nodes);
  void publishPath2(const std::vector<Eigen::Vector2d>& path_nodes);
  void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
  void drawESDFToMat(cv::Mat& esdf_img);
  void publishTestPath(const vector<Eigen::Vector2d>& path, const int& pub_num);
  void publishTestPath(const vector<Eigen::Vector2i>& path, const int& pub_num);
  void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);  
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg); 
  void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg); 

  float coordToWorldNoOffsetX(double x) const
  {
    return (x) * resolution_ - 0.5 * last_size_x_ * resolution_;
  }
  float coordToWorldNoOffsetY(double y) const
  {
    return (y) * resolution_ - 0.5 * last_size_y_ * resolution_;
  }

  float coordToWorldX(int x) const
  {
    return (x + 0.5f) * resolution_ - 0.5 * last_size_x_ * resolution_;
  }
  float coordToWorldY(int y) const
  {
    return (y + 0.5f) * resolution_ - 0.5 * last_size_y_ * resolution_;
  }
  int WorldToMapX(float x) const
  {
    return std::floor((x + 0.5 * last_size_x_ * resolution_) / resolution_);
  }
  int WorldToMapY(float y) const
  {
    return std::floor((y + 0.5 * last_size_y_ * resolution_) / resolution_);
  }

  inline bool isInMap(const Eigen::Vector2d& pos, double margin = 1e-4) {
    if (pos(0) < 0 + margin || pos(1) < 0 + margin) {
      return false;
    }
    if (pos(0) > last_size_x_ - margin || pos(1) > last_size_y_ - margin) {
      return false;
    }
    return true;
  }
  inline void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) {
    for (int i = 0; i < 2; ++i) id(i) = floor(pos(i));
  }
  
  inline void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) {
    for (int i = 0; i < 2; ++i) pos(i) = (id(i) + 0.5);
  }
  inline void boundIndex(Eigen::Vector2i& id) {
    Eigen::Vector2i id1;
    id1(0) = std::max(std::min(id(0), (int)last_size_x_ - 1), 0);
    id1(1) = std::max(std::min(id(1), (int)last_size_y_ - 1), 0);
    id = id1;
  }
  inline double getDis(const Eigen::Vector2d& pos) {
    Eigen::Vector2i id;
    posToIndex(pos, id);
    boundIndex(id);
    return voronoi_.getDistance(id(0), id(1));
  }

  ros::Publisher voronoi_grid_pub_, gvg_marker_pub_, distance_cloud_pub_, voronoi_occupy_pub_;
  ros::Publisher voronoi_grid_origin_pub_;
  ros::Publisher path_pub_, path_pub2_;
  ros::Publisher path_pub_test_1_, path_pub_test_2_, path_pub_test_3_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber odometry_sub_, rviz_goal_sub_;
  ros::Timer plan_timer_, voronoi_update_timer_;

  DynamicVoronoi voronoi_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  Eigen::Vector2d offset_{0.5, 0.5};
  float resolution_;
  float clearance_low_, clearance_high_;
  float clearance_low_thr_, clearance_high_thr_;
  boost::mutex mutex_;
  std::shared_ptr<GVG> gvg_;
  pcl::KdTreeFLANN<pcl::PointXY> GvgNodeKdTree_;
  std::shared_ptr<gvg::Planner> gvg_planner_;
  fast_planner::PlanningVisualization::Ptr visualization_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Vector3d lastest_goal_;
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;
  std::vector<Eigen::Vector2d> last_best_path_;  // 上次规划的路径
  Eigen::Vector3d goal_last_;
  bool has_odom_ = false;  // 是否有里程计数据
  bool has_goal_ = false;  // 是否有目标点

  bool gvg_updated_ = false;  // 是否更新了GVG

  // 这里是处理其他的东西
  double pathLength(const vector<Eigen::Vector2d>& path);
  vector<Eigen::Vector2d> shortcutPath(const std::vector<IntPoint>& path, int path_id = 0, int iter_num = 4);
  bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double thresh,
                 Eigen::Vector2d& pc, int caster_id = 0, int skip_mode = -1);
  bool lineVisib(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2, std::vector<IntPoint>& rayline_list,
                 double thresh, Eigen::Vector2i& pc, int caster_id = 0, int skip_mode = -1);
  bool lineVisib2(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2, std::vector<IntPoint>& rayline_list,
                             double thresh, Eigen::Vector2i& pc, int caster_id = 0, int skip_mode = -1);
  void evaluateEDTWithGrad(const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d& grad);
  void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], double dists[2][2], 
       Eigen::Vector2d& diff);
  void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, double& value, Eigen::Vector2d& grad);
  void planTimerCallback(const ros::TimerEvent&);
  void voronoiUpdateTimerCallback(const ros::TimerEvent&);

  vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path);
  vector<vector<Eigen::Vector2d>> discretizePaths(vector<vector<Eigen::Vector2d>>& path);
  vector<Eigen::Vector2d> discretizeLine(Eigen::Vector2d p1, Eigen::Vector2d p2);
  GraphNode::Ptr creatPathNode(
    const Eigen::Vector2i& node_idx,
    int& start_graph_id,
    bool& node_on_graph,
    const vector<IntPoint>& node_strong_cloud,
    int KdNeighborNum,
    bool startNode,
    GraphNode::Ptr node_tmp_ptr);
  void removeNodeAndConnections(gvg::GraphNode::Ptr node);
  void removeNodeAndRepairStrongConnection(gvg::GraphNode::Ptr node_tmp_ptr);
};

class SimpleTimer {
  public:
      SimpleTimer(const std::string& name = "") : name_(name) {
          start_ = std::chrono::steady_clock::now();
      }
      void reset() {
          start_ = std::chrono::steady_clock::now();
      }
      double elapsedMs() const {
          auto end = std::chrono::steady_clock::now();
          return std::chrono::duration<double, std::milli>(end - start_).count();
      }
      double elapsedSec() const {
          auto end = std::chrono::steady_clock::now();
          return std::chrono::duration<double>(end - start_).count();
      }
      double print(const std::string& msg = " ") const {
          std::cout << " [TimeCost]: " << (name_.empty() ? " " : name_ + " ") << msg
                    << " " << elapsedMs() << " ms." << std::endl;
          return elapsedMs();
      }
  private:
      std::chrono::steady_clock::time_point start_;
      std::string name_;
  };
}  // namespace DynaVoro
