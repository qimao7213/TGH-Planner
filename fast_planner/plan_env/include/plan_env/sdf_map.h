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



#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PolygonStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>
#include <cmath>
#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos，米制，最小值是(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_)
  Eigen::Vector3i map_voxel_num_;                        // 非负的，单位是栅格数目
  Eigen::Vector3i map_min_idx_, map_max_idx_;            // map range in index，单位是栅格数目，最小是（0，0，0）
  Eigen::Vector3d local_update_range_;                   //（5.5，5.5，4.5）无人机前方的一个区域，只处理这个区域内的数据
  double resolution_, resolution_inv_;                   // resolution_ = 0.1
  double obstacles_inflation_;                           // 0.099，碰撞障碍物的边界，从而忽略机器人的尺寸
  string frame_id_;                                      // world
  int perception_data_type_;                             // 
  string lidar_link_, camera_link_, camera_frame_;       // 
  Eigen::Matrix3d camera_LinkToFrame_;                      // camera的旋转矩阵
  Eigen::Vector3d sensor_on_base_pos_;
  Eigen::Quaterniond sensor_on_base_q_;
  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_; //只使用这个depth（0.2，5.0）范围内的深度信息，其他深度点就忽略
  double depth_filter_tolerance_;                      //检查和上一帧的一致性，没有启用
  int depth_filter_margin_;                                                     //图像边缘的就跳过
  bool use_depth_filter_;
  double k_depth_scaling_factor_;                                               //尺度因子
  int skip_pixel_;                                                              //降采样

  /* raycasting *///和占据栅格地图的更新有关
  // occupancy probability，对应的是十四讲上的x，范围是（0，1）

  double p_hit_;     // 0.65
  double p_miss_;    // 0.35
  double p_min_;     // 0.12，x的最小值，占据概率变化的最小值，如果比这个还小，则就是Unknown
  double p_max_;     // 0.90
  double p_occ_;     // 0.80
  
  // logit of occupancy probability
  // 这几个值对应的是十四讲上的y，范围是（-inf，+inf）
  double prob_hit_log_;      //0.2688，如果hit的更新量
  double prob_miss_log_;     //-0.2688，如果miss的更新量
  double clamp_min_log_;     //-0.8653
  double clamp_max_log_;     //0.95442
  double min_occupancy_log_; //0.6020，判定为occpancy的阈值
  // range of doing raycasting（0.5，4.5），只在这个范围内进行raycast更新栅格信息                 
  double min_ray_length_, max_ray_length_; 

  /* local map update and clear */
  double local_bound_inflate_; //local？什么的膨胀范围，值为0
  int local_map_margin_;       //10个像素，esdf更新范围，扩大margin个像素

  /* visualization and computation time display */

  double esdf_slice_height_; //esdf地图其实是3D的，但只切片了一层来可视化。这就是切片层的高度
  double visualization_truncate_height_; //发布local占据地图map的时候最高的高度
         // 什么地图？添加了一层天花板，所以无人机搜索路径的时候不会飞得太高。
         // 又这个值比visualization_truncate_height_大，所以可视化的时候不会把天花板显示出来
  double virtual_ceil_height_;     
  // ground_height_，地面高度，其实是栅格地图的z轴的最小值，应该小于真实的地面高度把地面包含进去
  // 这个值应该由初始化的时候，call tf，获取robot base_link（地面）在world下的z轴坐标来确定
  // sensor_height_，其实表示的是传感器在world下的高度，因为里程计和原始点云都在传感器坐标系下
  double ground_height_; 
  double sensor_height_; 
  double ground_height_diff_ = 0.5;
  bool show_esdf_time_, show_occ_time_; //是不是显示地图更新的耗时

  /* active mapping */
  double unknown_flag_;

  // QHB: 2D
  bool need_map_2D_ = true;
  double ground_height_2D_;
  // 将world坐标系下[height_obs_min_2D_, height_obs_max_2D_]的设置为障碍物
  double height_obs_max_2D_;      
  double height_obs_min_2D_;      

  double scan_angle_increment_; // 从3D点云生成 2D激光扫描的角度增量，单位是弧度.如果原始点云比较密集，这个值可以设置小一点；否则，这个值应该大
  double half_fov_;             // 对2D激光扫描进行稠密化时考虑的半视场角，单位是弧度
};

// intermediate mapping data for fusion, esdf

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance
  // 注意，这些buffer的数据都不会重置，因为维护的是整个地图的信息，每次更新的时候都会继续使用
  std::vector<double> occupancy_buffer_;        //每个值的范围是（-inf，+inf），十四讲的y，和occupancy_buffer_inflate_的区别
  std::vector<char> occupancy_buffer_neg;       //和occupancy_buffer_inflate_完全是一对，完全取决于occupancy_buffer_inflate_
  std::vector<char> occupancy_buffer_inflate_;  //这里面每个值应该是0或者1。没有膨胀。现在加一个-1，来表示未知。
  std::vector<double> distance_buffer_;         //ESDF地图的正、负和总的
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;

  // QHB: 2D
  std::vector<double> occupancy_buffer_2D_;        //每个值的范围是（-inf，+inf），十四讲的y，和occupancy_buffer_inflate_的区别
  std::vector<char> occupancy_buffer_inflate_2D_;      //膨胀后的2D占据地图。SDF是根据膨胀后的计算的.0-free, 1-占据, -1-未知
  std::vector<char> occupancy_buffer_inflate_2D_v2_;   //raycast2D更新出来的地图
  std::vector<char> occupancy_buffer_neg_2D_;    
  std::vector<double> distance_buffer_pos_2D_;         //2D ESDF地图的正、负和总的
  std::vector<double> distance_buffer_neg_2D_;
  std::vector<double> distance_buffer_all_2D_;
  nav_msgs::OccupancyGrid occpancy_map_2D_;
  nav_msgs::OccupancyGrid occpancy_map_2D_binary_;     //把未知的也设置为了0
  nav_msgs::OccupancyGrid occpancy_map_2D_v2_;         //用raycast2D更新出来的地图
  std::vector<double> tmp_buffer3_;
  std::vector<Eigen::Vector3i> freed_idx_;             //给2D occupancy_buffer_inflate_2D_清理地图用

  // camera position and pose data
  // sensorOnBase是指lidar在base_link下的位姿
  Eigen::Vector3d camera_pos_, last_camera_pos_, scan_pos_;
  Eigen::Quaterniond camera_q_, last_camera_q_, scan_q_;
  ros::Time time_of_received_pointcloud_;
  // depth image data

  cv::Mat depth_image_, last_depth_image_;      //last_depth_image_是用来检查一致性的。目前没有用上
  int image_cnt_;

  // flags of map state

  bool occ_need_update_; //收到深度图和位姿之后，occ_need_update_就变为true了；更新Occupancy地图之后，变为false，等待新的数据
  //再更新Occupancy的raycast之后设置为true；更新Occupancy地图之后，变为false，等待新的数据
  //专门设置local_updated_这个量是说，有时候raycast的点云数量为0，直接就退出了，这个时候就不用更新esdf
  bool local_updated_;   
  bool esdf_need_update_;//更新Occupancy之后，就需要更新esdf；更新esdf后，就变为false
  bool has_first_depth_;
  bool has_odom_, has_cloud_;
  bool sub_cloudpoint_ = false;
  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_; //深度图里，可以用来更新占据栅格地图的点云。先分配了固定大小的，具体有多少个点根据proj_points_cnt来确定
  vector<Eigen::Vector3d> proj_points_local_;
  sensor_msgs::LaserScan scan_pt_;
  vector<Eigen::Vector3d> scan_pt_refined_; 
  ros::Time sensor_time_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_; //count_hit_是统计hit的次数，count_hit_and_miss_是统计hit和miss的次数，即这个节点被访问的次数
  vector<char> flag_traverse_, flag_rayend_;     //
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // 2D raycast
  vector<short> count_hit_2D_, count_hit_and_miss_2D_; //count_hit_是统计hit的次数，count_hit_and_miss_是统计hit和miss的次数，即这个节点被访问的次数
  vector<char> flag_traverse_2D_, flag_rayend_2D_;     //
  char raycast_num_2D_;
  queue<Eigen::Vector3i> cache_voxel_2D_;


  // range of updating ESDF
  // esdf地图的更新范围，和updateESDF3d()有关，在raycastProcess时确定，不是全部地图
  Eigen::Vector3i local_bound_min_, local_bound_max_; 
  Eigen::Vector3i total_bound_min_, total_bound_max_; //更新后的整个地图的范围
  // computation time

  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SDFMap {
public:
  SDFMap() {}
  ~SDFMap() {}

  enum { DEPTH_IMAGE = 1, LIDAR_POINT = 2, INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline void indexToPos2D(const Eigen::Vector2i& id, Eigen::Vector3d& pos); // 专门给visLine写的一个重载
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  // 直接在occupancy_buffer_inflate_地图里面操作，把一个栅格设置为占据
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  inline void boundIndex(Eigen::Vector3i& id);            //把一个index限定再bound内 
  inline bool isUnknown(const Eigen::Vector3i& id);       //是否Unknown，什么样的算Unknown呢
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  inline char queryState2D(const Eigen::Vector3i& id)
  {
      Eigen::Vector3i id1 = id;
      boundIndex(id1);
      return md_.occupancy_buffer_inflate_2D_[toAddress2D(id1.head(2))];
  }
  inline char queryState2D(const Eigen::Vector3d& pos)
  {
      Eigen::Vector3i idc;
      posToIndex(pos, idc);
      return queryState2D(idc);
  }
  double getMinDisViaLineSeg2D(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2);
  void clearMapByRobot(const Eigen::Vector3d& pos);
  void setMapObstacleByPoint(const Eigen::Vector3d& pos, double radius);
  // distance field management
  // 从esdf地图里面获取dist数据
  inline double getDistance(const Eigen::Vector3d& pos);
  inline double getDistance(const Eigen::Vector3i& id);
  inline double getDistance2D(const Eigen::Vector2d& pos);
  inline double getDistance2D(const Eigen::Vector3d& pos3d);
  inline double getDistance2D(const Eigen::Vector2i& id);
  
  //通过线形插值的方式，从esdf地图中计算某个double点的距离场梯度
  inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  //应该是插值的时候用的把吧，得到一个double的pos，返回3维8邻域的点
  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
  void getSurroundPts2D(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2], Eigen::Vector3d& diff);
  // /inline void setLocalRange(Eigen::Vector3d min_pos, Eigen::Vector3d
  // max_pos);
  void pointcloudToLaserScan(const std::vector<Eigen::Vector3d>& cloud, sensor_msgs::LaserScan& scan_msg,
    float angle_min, float angle_max, float angle_increment, float range_min, float range_max, float min_height,
    float max_height, bool use_inf = false, float inf_epsilon = 0.1);
  void updateESDF3d();
  void getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                    vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad,
                    int sign = 1);  // 1 pos, 2 neg, 3 combined
  void initMap(ros::NodeHandle& nh);

  // 这个几个pub的函数要好好看看
  void publishMap();                               //这个就是发布local range的地图
  void publishMapInflate(bool all_info = false);   //这个和publishMap差不多的啊，就是all_info=true的时候范围会大一点
  void publishESDF();
  void publishUpdateRange();

  void publishUnknown();                          //y值小于mp_.clamp_min_log_ - 1e-3的就是Unknown
  void publishUnknown2D();
  void publishDepth();
  void publishScanMsg();
  void publishMissOcc();
  void checkDist();
  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<SDFMap> Ptr;
  // QHB: 2D  
  inline int getInflateOccupancy2D(Eigen::Vector2d pos);  
  inline bool isInMap2D(const Eigen::Vector2d& pos, const double& margin = 0.0);
  inline bool isInMap2D(const Eigen::Vector2i& idx);
  inline double getGroundHeight();
  inline void setNeed2DMap(bool need_map_2D) {this->mp_.need_map_2D_ = need_map_2D;}
  inline void indexToPos2D(const Eigen::Vector2i& id, Eigen::Vector2d& pos);

  // QHB：用来测试topo_rpm
  void initMapFromCostMap(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  // get depth image and camera pose
  // 通过什么形式获取点云和传感器位姿信息
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_pt, const nav_msgs::OdometryConstPtr& odom);
  void depthCallback(const sensor_msgs::ImageConstPtr& img);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);

  // update occupancy by raycasting, and update ESDF
  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  // main update process
  void projectDepthImage();
  void raycastProcess();
  void raycastProcess2D();
  void clearAndInflateLocalMap();
  void densifyScanPoints(const sensor_msgs::LaserScan& scan_pt,
                         std::vector<Eigen::Vector3d>& dense_points, double half_fov = M_PI,
                         double min_dist = 0.15, double max_dist = 0.50);
  //把一个index pt附近step范围内的检索出来
  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ, bool check_occ = false);
  int setCacheOccupancy2D(Eigen::Vector3d pos, int occ, bool check_occ = false);
  // 如果一个点超出地图范围内，那么就找一个地图内的最近的点给它。这样就不会出现索引越界的报错了
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
  bool getTransform(const std::string& source_frame, const std::string& target_frame, const ros::Time& time,
    Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;
  SynchronizerCloudOdom sync_cloud_odom_;

  ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
  // 区分map_pub_和map_inf_pub_
  ros::Publisher map_pub_, map_inf_pub_, 
                 esdf_pub_, 
                 update_range_pub_;//那个长方体
  ros::Publisher unknown_pub_, 
                 depth_pub_,      //一定范围内的深度点云
                 scan_pub_,
                 scan_refined_pub_;
  ros::Publisher depth_pub2_;     //测试发布的点云是否对齐用
  ros::Publisher update_region_poly_pub_;
  ros::Timer occ_timer_, esdf_timer_, vis_timer_;

  //给深度图加噪声用的，但是先在没有使用
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;

  // QHB: 2D
  void clearAndInflateLocalMap2D();
  ros::Publisher map_pub_2D_, esdf_pub_2D_, unknown_pub_2D_, map_pub_2D_binary_;
  ros::Publisher uknown_check_pub_;
  inline int toAddress2D(const Eigen::Vector2i& id);
  inline int toAddress2D(const int& x, const int& y);
  void publishMap2D(); 
  void publishESDF2D();
  bool pub_all_map_ = true;
  void updateESDF2d();
  inline void posToIndex2D(const Eigen::Vector2d& pos, Eigen::Vector2i& id);

  inline void boundIndex2D(Eigen::Vector2i& id);  

  //QHB: test topo_rpm
  ros::Subscriber cost_map_sub_, click_point_sub_;
  void ClickpointCallback(const geometry_msgs::PointStampedConstPtr &click_point);
  void CostmapCallback(const nav_msgs::OccupancyGridConstPtr &map);
  bool has_costmap_ = false;
  bool map_complete_ = false;
  bool topo_test_ = false;
};

/* ============================== definition of inline function
 * ============================== */

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int SDFMap::toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

// 注意，2D的遍历顺序和3D的是不一样的
inline int SDFMap::toAddress2D(const Eigen::Vector2i& id) {
  return id(0) + id(1) * mp_.map_voxel_num_(0);
}

inline int SDFMap::toAddress2D(const int& x, const int& y) {
  return x + y * mp_.map_voxel_num_(0);
}

inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline void SDFMap::boundIndex2D(Eigen::Vector2i& id) {
  Eigen::Vector2i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id = id1;
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

inline double SDFMap::getDistance2D(const Eigen::Vector2d& pos) {
  Eigen::Vector2i id;
  posToIndex2D(pos, id);
  boundIndex2D(id);

  return md_.distance_buffer_all_2D_[toAddress2D(id)];
}
inline double SDFMap::getDistance2D(const Eigen::Vector3d& pos3d) {
  Eigen::Vector2d pos = pos3d.head(2);
  Eigen::Vector2i id;
  posToIndex2D(pos, id);
  boundIndex2D(id);

  return md_.distance_buffer_all_2D_[toAddress2D(id)];
}
inline double SDFMap::getDistance2D(const Eigen::Vector2i& id) {
  Eigen::Vector2i id1 = id;
  boundIndex2D(id1);
  return md_.distance_buffer_all_2D_[toAddress2D(id1)];
}


inline bool SDFMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool SDFMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool SDFMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool SDFMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * mp_.resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= mp_.resolution_inv_;

  return dist;
}

inline void SDFMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

// 在occupancy_buffer_上操作，但是occ只能等于0或者1？？
inline void SDFMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int SDFMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int SDFMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getInflateOccupancy2D(Eigen::Vector2d pos) {
  if (!isInMap2D(pos)) return -1;

  Eigen::Vector2i id;
  posToIndex2D(pos, id);

  return int(md_.occupancy_buffer_inflate_2D_[toAddress2D(id)]);
}

inline int SDFMap::getOccupancy(Eigen::Vector3i id) {
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

inline bool SDFMap::isInMap2D(const Eigen::Vector2d& pos, const double& margin) {
  if (pos(0) - margin < mp_.map_min_boundary_(0) + 1e-4 || pos(1) - margin < mp_.map_min_boundary_(1) + 1e-4 ) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) + margin > mp_.map_max_boundary_(0) - 1e-4 || pos(1) + margin > mp_.map_max_boundary_(1) - 1e-4 ) {
    return false;
  }
  return true;
}

inline bool SDFMap::isInMap2D(const Eigen::Vector2i& idx) {
  if (idx(0) < 0 || idx(1) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1) {
    return false;
  }
  return true;
}

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void SDFMap::posToIndex2D(const Eigen::Vector2d& pos, Eigen::Vector2i& id) {
  for (int i = 0; i < 2; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void SDFMap::indexToPos2D(const Eigen::Vector2i& id, Eigen::Vector2d& pos) {
  for (int i = 0; i < 2; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void SDFMap::indexToPos2D(const Eigen::Vector2i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 2; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}
inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  if(mp_.need_map_2D_)
  {
    for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      // for (int z = -step; z <= step; ++z) 
      {
        if(step !=0 && abs(x) == step && abs(y) == step) continue;
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + 0);
      }
  }
  else
  {
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }    
  }

}

inline double SDFMap::getGroundHeight()
{
  return this->mp_.ground_height_2D_;
}


inline void SDFMap::clearMapByRobot(const Eigen::Vector3d& pos)
{
  int inf_step = ceil(0.3 / mp_.resolution_);
  vector<Eigen::Vector3i> inf_pts;
  inf_pts.resize(pow(2 * inf_step + 1, 2) - 4);
  Eigen::Vector3i inf_pt;
  Eigen::Vector3i pos_id;
  posToIndex(pos, pos_id);
  inflatePoint(pos_id, inf_step, inf_pts);
  for (int k = 0; k < (int)inf_pts.size(); ++k) 
  {
    inf_pt = inf_pts[k];
    if(!this->isInMap2D(Eigen::Vector2i(inf_pt.x(), inf_pt.y())))
    {
      continue;
    }
    md_.occupancy_buffer_inflate_2D_[toAddress2D(inf_pt.x(), inf_pt.y())] = 0;
    // flate_num ++;
  }
}

inline void SDFMap::setMapObstacleByPoint(const Eigen::Vector3d& pos, double radius)
{
  int inf_step = ceil(radius / mp_.resolution_);
  vector<Eigen::Vector3i> inf_pts;
  inf_pts.resize(pow(2 * inf_step + 1, 2) - 4);
  Eigen::Vector3i inf_pt;
  Eigen::Vector3i pos_id;
  posToIndex(pos, pos_id);
  inflatePoint(pos_id, inf_step, inf_pts);
  for (int k = 0; k < (int)inf_pts.size(); ++k) 
  {
    inf_pt = inf_pts[k];
    if(!this->isInMap2D(Eigen::Vector2i(inf_pt.x(), inf_pt.y())))
    {
      continue;
    }
    md_.occupancy_buffer_inflate_2D_[toAddress2D(inf_pt.x(), inf_pt.y())] = 1;
    // flate_num ++;
  }
}

#endif