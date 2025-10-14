
#pragma once
#define SavePathData
#include <memory>

#include <boost/thread.hpp>
#include "costmap_2d/GenericPluginConfig.h"
#include "costmap_2d/cost_values.h"
#include "costmap_2d/layer.h"
#include "costmap_2d/layered_costmap.h"
#include "dvr/dynamicvoronoi.h"
#include "dvr/GVG.h"
#include "dvr/raycast.h"
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
#include <std_msgs/Float32MultiArray.h>
#include <chrono>  // NOLINT
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <fstream>
// #include <traj_utils/planning_visualization.h>

using std::vector;
namespace DynaVoro
{

#ifdef SavePathData
struct GraphInfo {
    int branch_count;  // 分支数量
    std::vector<Eigen::Vector2i> strong_nodes;  // 强节点坐标
    std::vector<Eigen::Vector2i> weak_nodes;    // 弱节点坐标
    int strong_node_count;  // 强节点数量
    int weak_node_count;    // 弱节点数量
    std::vector<int> neighbor_counts;  // 每个节点的邻居数量
    std::vector<std::vector<float>> neighbor_path_lengths;  // 每个节点的路径长度

    // GraphInfo();

    void init (std::shared_ptr<gvg::GVG> gvg) {
        this->branch_count = gvg->getGraphsSize();  // 假设 GVG 提供分支数量的接口

        for (const auto& graph : gvg->getGraphs()) {  // 假设 GVG 提供图的接口
            for (const auto& node_pair : graph) {
                auto node_ptr = node_pair.second;
                if (!node_ptr) 
                {
                  ROS_ERROR("Node pointer is null in GraphInfo constructor.");
                  continue;
                }
                Eigen::Vector2i node_pos(node_ptr->pos.x, node_ptr->pos.y);
                
                if (node_ptr->type == gvg::GraphNode::Strong) {
                    this->strong_nodes.emplace_back(node_pos);
                }
                else if (node_ptr->type == gvg::GraphNode::Weak) {
                    this->weak_nodes.emplace_back(node_pos);
                }   
                else {
                    ROS_ERROR("Node type is neither Strong nor Weak, skipping.");
                    continue;  // 如果节点类型不是强或弱，则跳过
                }

                this->neighbor_counts.push_back(node_ptr->neighbors.size());
                std::vector<float> path_lengths;
                for (int i = 0; i < node_ptr->neighbors.size(); ++i) {
                    path_lengths.push_back(node_ptr->neighbor_paths[i].path_length);
                }
                this->neighbor_path_lengths.push_back(path_lengths);
                this->strong_node_count = this->strong_nodes.size();
                this->weak_node_count = this->weak_nodes.size();
            }
          }
    }


    // 比较函数
    bool operator==(const GraphInfo& other) const {
        return branch_count == other.branch_count &&
               strong_nodes == other.strong_nodes &&
               weak_nodes == other.weak_nodes &&
               strong_node_count == other.strong_node_count &&
               weak_node_count == other.weak_node_count &&
               neighbor_counts == other.neighbor_counts &&
               neighbor_path_lengths == other.neighbor_path_lengths;
    }

    // 打印函数
    void print() const {
        std::cout << "Graph Info:" << std::endl;
        std::cout << "Branch Count: " << branch_count << std::endl;
        std::cout << "Strong Nodes: ";
        for (const auto& node : strong_nodes) {
            std::cout << "(" << node.x() << ", " << node.y() << ") ";
        }
        std::cout << std::endl;
        std::cout << "Weak Nodes: ";
        for (const auto& node : weak_nodes) {
            std::cout << "(" << node.x() << ", " << node.y() << ") ";
        }
        std::cout << std::endl;
        std::cout << "Strong Node Count: " << strong_node_count << std::endl;
        std::cout << "Weak Node Count: " << weak_node_count << std::endl;
        std::cout << "Neighbor Counts: ";
        for (const auto& count : neighbor_counts) {
            std::cout << count << " ";
        }
        std::cout << std::endl;
        std::cout << "Path Lengths: ";
        for (const auto& lengths : neighbor_path_lengths) {
            for (const auto& length : lengths) {
              std::cout << length << " ";
        }
        std::cout << std::endl;
    }
  }
};

struct PathData {
  std::string save_file;
  std::ofstream save_file_stream;
  int iter_num = 0;
  double t_search = 0.0;
  double t_shortcut = 0.0;
  double min_path_length = 100000000;
  double min_raw_path_length = 100000000;
  double start_x = 0.0;
  double start_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;

  ~PathData() {
    if (save_file_stream.is_open()) {
      save_file_stream.close();
      std::cout << "File stream closed for: " << save_file << std::endl;
    }
  }
  void reset() {
    t_search = 0.0;
    t_shortcut = 0.0;
    min_path_length = 100000000;
    min_raw_path_length = 100000000;
  }
  void saveToFile() {
    if (save_file_stream.is_open()) {
    save_file_stream << iter_num << " ";
    save_file_stream << start_x << " ";
    save_file_stream << start_y << " ";
    save_file_stream << goal_x << " ";
    save_file_stream << goal_y << " ";
    save_file_stream << t_search << "  ";
    save_file_stream << t_shortcut << "  ";
    save_file_stream << min_path_length << " ";
    save_file_stream << min_raw_path_length << " ";
    save_file_stream << "\n";  // 换行
    } else {
      std::cerr << "Failed to open file for saving path data." << std::endl;
    }
  }
};
#endif  // SavePathData

class VoronoiLayer
{
public:
  VoronoiLayer(ros::NodeHandle& nh);
  bool update_by_occupancy_map(const std::vector<char>& occupancy_map, int map_size_x, int map_size_y);

  const DynamicVoronoi& getVoronoi() const;
  boost::mutex& getMutex();
  bool plan(Eigen::Vector3d start, Eigen::Vector3d goal, double start_yaw = 0.0);
  std::vector<std::vector<Eigen::Vector3d>> getVoroPaths(double ground_height_) const
  {
    std::vector<std::vector<Eigen::Vector3d>> voro_paths;
    for (const auto& path : voro_paths_shortcut_) {
      std::vector<Eigen::Vector3d> path3d;
      for (const auto& pt : path) {
        path3d.emplace_back(pt.x(), pt.y(), ground_height_);
      }
      voro_paths.emplace_back(path3d);
    }
    return voro_paths;
  }

  double getESDF(const int& idx) const  {return esdf_data_[idx];}

  double getESDF(const int& x, const int& y) const {return esdf_data_[y * last_size_x_ + x]; }

  void getESDFMap(std::vector<double>& esdf_map) const {esdf_map = esdf_data_;}

  #ifdef SavePathData
  void closeSaveFile() {
    path_data_.save_file_stream.close();
  }
  #endif

private:
  void publishVoronoiGrid();
  void publishDistanceCloud();
  void publishGVG(const std::vector<std::unordered_map<IntPoint, gvg::GraphNode::Ptr>>& gvg);
  void publishPath(const std::vector<IntPoint>& path_nodes);
  void publishPath2(const std::vector<Eigen::Vector2d>& path_nodes);
  void publishRuntime(double gvd_runtime, double egvg_runtime);
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
  ros::Publisher egvg_runtime_pub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber odometry_sub_, rviz_goal_sub_;
  #ifdef SavePathData
  ros::Subscriber raw_path_sub_;
  #endif

  ros::Timer plan_timer_, voronoi_update_timer_;

  DynamicVoronoi voronoi_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  Eigen::Vector2d offset_{0.5, 0.5};
  float resolution_, resolutuon_inv_;
  float clearance_low_, clearance_high_;
  float clearance_low_thr_, clearance_high_thr_;
  boost::mutex mutex_;
  std::shared_ptr<gvg::GVG> gvg_;
  pcl::KdTreeFLANN<pcl::PointXY> GvgNodeKdTree_;
  std::shared_ptr<gvg::Planner> gvg_planner_;
  #ifdef SavePathData
  PathData path_data_;
  GraphInfo last_graph_info_;  
  RayCaster raycaster_;  
  #endif 
  // 这个都是世界坐标系下的，要传递出去的.voro_paths_raw_world_是稠密的，voro_paths_shortcut_是稀疏的
  std::vector<std::vector<Eigen::Vector2d>> voro_paths_raw_world_, voro_paths_shortcut_;

  std::vector<double> esdf_data_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Vector3d lastest_goal_;
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;
  std::vector<Eigen::Vector2d> last_best_path_;  // 上次规划的路径
  Eigen::Vector3d goal_last_ = Eigen::Vector3d(-100, -100, -100);  // 上次规划的目标点
  bool has_odom_ = false;  // 是否有里程计数据
  bool has_goal_ = false;  // 是否有目标点
  bool gvg_updated_ = false;  // 是否更新了GVG  

  // std::vector<bool> voronoi_grid_;
  // std::vector<float> voronoi_esdf_;

  // bool voronoiCheck(int x, int y);
  // float voronoiESDF(int x, int y);

  // 这里是处理其他的东西
  double pathLength(const vector<Eigen::Vector2d>& path);
  double pathLength(const vector<IntPoint>& path);
  vector<Eigen::Vector2d> shortcutPath(const std::vector<IntPoint>& path, int path_id = 0, int iter_num = 4);
  vector<Eigen::Vector2d> shortcutPath2(const std::vector<IntPoint>& path, int path_id = 0, int iter_num = 4);
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
  #ifdef SavePathData
  void rawPathCallback(const nav_msgs::Path::ConstPtr& msg);
  #endif

  std::vector<IntPoint> SampleAstar(IntPoint start, IntPoint goal);

  vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path);
  vector<vector<Eigen::Vector2d>> discretizePaths(vector<vector<Eigen::Vector2d>>& path);
  vector<Eigen::Vector2d> discretizeLine(Eigen::Vector2d p1, Eigen::Vector2d p2);
  gvg::GraphNode::Ptr creatPathNode(
    const Eigen::Vector2i& node_idx,
    int& start_graph_id,
    bool& node_on_graph,
    const vector<IntPoint>& node_strong_cloud,
    int KdNeighborNum,
    bool startNode,
    gvg::GraphNode::Ptr node_tmp_ptr);
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
          // std::cout << " [TimeCost]: " << (name_.empty() ? " " : name_ + " ") << msg
          //           << " " << elapsedMs() << " ms." << std::endl;
          return elapsedMs();
      }
  private:
      std::chrono::steady_clock::time_point start_;
      std::string name_;
  };
}  // namespace DynaVoro
