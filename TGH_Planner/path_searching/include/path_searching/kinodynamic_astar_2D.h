#ifndef _KINODYNAMIC_ASTAR_2D_H
#define _KINODYNAMIC_ASTAR_2D_H

// #include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"
#include <path_searching/kinodynamic_astar.h>
#include "path_searching/dubins.h"
#include <path_searching/jump_point_search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
namespace fast_planner {

class KinodynamicAstar2D {
 private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;//大小为10000的node pool
  int use_node_num_, iter_num_, use_node_num_last_ = 0;
  int use_JPS_times_ = 0; //要连续使用4次JPS引导
  NodeHashTable expanded_nodes_; //这个是close_set?
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;//这就是priority_queue的写法，底层用std::vector<PathNodePtr>来储存对象
  std::vector<PathNodePtr> path_nodes_; //当一次搜索结束后，用来存放reverse后的路径节点的。start节点也在里面

  // ---------- JPS path ---- ---------//
  unique_ptr<JumpPointSearch> jps_path_finder_;
  bool useJPS_, succJPS_ = false;
  bool use_kino_replan_ = true; // true表示用kino replanning，false表示用cmu_planner
  std::vector<double> jps_path_distance_;
  pcl::KdTreeFLANN<pcl::PointXY> JPSPathKdTree;

  // ---------- Topo path -------------//
  vector<Eigen::Vector3d> topo_path_;     //要分清是稀疏的点or稠密的点
  std::vector<double> topo_path_distance_;
  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_, start_pt_;
  // shared_ptr<SDFMap> sdf_map;
  EDTEnvironment::Ptr edt_environment_;
  bool is_shot_succ_ = false;
  double total_t_;            //总的轨迹时间
  int case_id_;
  double t1_, t2_, t3_, v_p_;
  double shot_len_, expand_len_, total_len_;
  bool has_path_ = false;
  DubinsPath::DubinsPath path_;
  double one_shot_vel_;      //one shot时使用的速度
  vector<double> steering_angle_rads_;
  double ground_height_;
  double start_yaw_, end_yaw_;
  /* ---------- parameter ---------- */
  /* search */
  // max_tau_ = 0.6， init_max_tau_ = 0.8
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;
  double obs_dis_;

  double steering_angle_;
  int steering_angle_discrete_num_;
  int vel_discrete_num_;      
  double wheel_base_;         //轴距，即前后轮的距离
  double steering_radius_;
  double steering_penalty_, steering_change_penalty_, reversing_penalty_;
  double vehicle_length_, vehicle_width_, vehicle_rear_dis_;
  double steering_radian_;
  bool reverse_enable_;
  double shot_distance_;
  double expand_time_;        //扩展节点时，每一个中间点的时间间隔，设为0.1s。越小轨迹就越准确。这里，扩展时的、getKinoTraj和getSample的时间间隔都必须一样才能保证轨迹时一样的
  int mid_states_size_;
  bool show_search_tree_ = false; //现在这个不能关闭了，因为涉及到getKinoTraj()
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  double yaw_resolution_, inv_yaw_resolution_;
  /* helper */
  Eigen::Vector3i stateToIndex(Eigen::Vector3d state);
  Eigen::Vector2i stateToIndex2D(Eigen::Vector2d state);
  Eigen::Vector2d indexToState2D(Eigen::Vector2i index);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);//通过BVP来计算
  double estimateG(const Eigen::Matrix<double, 6, 1>& state0, const int & steering0,
                  const Eigen::Matrix<double, 6, 1>& state1, const int & steering1,
                  double ts) const;
  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, 
                    Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector2d um, double tau);
  int calVelOnShotTraj(const double& v0);
  double getDist(const double& t, const double & v0);
  double calScaleFactor(const double& t, const double & v0, const double& curv_tmp);
 public:
  KinodynamicAstar2D(){};
  ~KinodynamicAstar2D();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4, ONE_SHOT_FAIL = 5 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  // TODO: 留了一个变量gen_search，想的是如果是轨迹和新发现的障碍物碰撞那就不要触发use_JPS_times的计数
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, double start_yaw,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, double end_yaw, 
             bool init, bool dynamic = false,
             double time_start = -1.0, bool gen_search = 1);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  std::vector<Eigen::Vector3d> getKinoTraj(double& delta_t);

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);
  void getDerivatives(vector<Eigen::Vector3d>& start_end_derivatives);
  std::vector<PathNodePtr> getVisitedNodes();
  std::vector<Eigen::Vector3d> getJpsPath() 
  {
    if(useJPS_) return jps_path_finder_->getJpsPath();
    else return std::vector<Eigen::Vector3d>();
  }
  bool showSearchTree()
  {
    return show_search_tree_;
  }
  void setGuidePath(const vector<Eigen::Vector3d>& topo_path)
  {
    this->topo_path_ = topo_path;
  }
  vector<Eigen::Vector4d> getSearchTree();
  double getSteerRadius() {return steering_radius_;}
  typedef shared_ptr<KinodynamicAstar2D> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif