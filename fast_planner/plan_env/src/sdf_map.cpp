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



#include "plan_env/sdf_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]
Eigen::Vector2d check_point_(-1.95, 10.1);

void SDFMap::initMap(ros::NodeHandle& nh) {
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("sdf_map/resolution", mp_.resolution_, -1.0);//分辨率
  node_.param("sdf_map/map_size_x", x_size, -1.0);//整个地图的尺寸。在后面主要是用来判断一个点是不是超过边界了
  node_.param("sdf_map/map_size_y", y_size, -1.0);
  node_.param("sdf_map/map_size_z", z_size, -1.0);
  node_.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);//当前处理的sdf地图的范围，看起来像是在无人机的正前方的区域
  node_.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);//碰撞障碍物的边界，从而忽略机器人的尺寸

  node_.param("sdf_map/fx", mp_.fx_, -1.0);//相机参数
  node_.param("sdf_map/fy", mp_.fy_, -1.0);
  node_.param("sdf_map/cx", mp_.cx_, -1.0);
  node_.param("sdf_map/cy", mp_.cy_, -1.0);

  node_.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);//对深度图进行降采样，并筛选一定范围的点来构建地图
  node_.param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("sdf_map/p_hit", mp_.p_hit_, 0.70);//占据地图更新的参数
  node_.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("sdf_map/p_min", mp_.p_min_, 0.12);
  node_.param("sdf_map/p_max", mp_.p_max_, 0.97);
  node_.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);//进行raycast的范围，这个范围是在深度图有效值的范围内的
  node_.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);//在可视化的时候只显示了一层的切片（2D），而不是整个范围内的3D ESDF地图。
  node_.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);//无人机的最高飞行高度。这以上的高度全部被设置为了有障碍物

  node_.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);//计算占据地图和esdf地图的计算时间
  node_.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
  node_.param("sdf_map/pose_type", mp_.pose_type_, 1);//1代表位姿信息是来自pose，2代表位姿信息是来自odom。对应了不同的解码方式

  node_.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);//bound是一个膨胀
  node_.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);        //margin是另一个膨胀
  node_.param("sdf_map/ground_height", mp_.ground_height_, 1.0);//地面高度是怎么使用的呢？
  node_.param("sdf_map/camera_height", mp_.camera_height_, 0.0);//地面高度是怎么使用的呢？

  // QHB: 2D
  mp_.ground_height_2D_ = mp_.ground_height_;
  node_.param("sdf_map/height_max_2D", mp_.height_max_2D_, 2.0);
  node_.param("sdf_map/need_map_2D", mp_.need_map_2D_, false);


  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);//local map update and clear
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;//unknown是代表什么呢？

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;
  cout << "occ need hit times: " << ceil((mp_.min_occupancy_log_ - mp_.clamp_min_log_ + mp_.unknown_flag_) / mp_.prob_hit_log_) << std::endl;
  cout << "free need miss times: " << ceil((mp_.clamp_max_log_ - mp_.min_occupancy_log_) / -mp_.prob_miss_log_) << std::endl;


  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_; // map_max_boundary_的z会超过0.5 * z_size

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers
  // 这里放的是整个地图的？？
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, -1);

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_.raycast_num_ = 0;
  //TODO：这里不能直接写
  md_.proj_points_.resize((4 * mp_.cy_ * mp_.cx_ + 400) / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  // QHB: 2D
  int buffer_size_2D = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);
  md_.occupancy_buffer_inflate_2D_ = vector<char>(buffer_size_2D, -1);
  md_.occupancy_buffer_neg_2D_ = vector<char>(buffer_size_2D, 0);  
  md_.occpancy_map_2D_.header.frame_id = mp_.frame_id_;
  md_.occpancy_map_2D_.info.resolution = mp_.resolution_;
  md_.occpancy_map_2D_.info.height = mp_.map_voxel_num_(1); //occpancy_map_2D_的height是行
  md_.occpancy_map_2D_.info.width = mp_.map_voxel_num_(0);  //occpancy_map_2D_的width是列


  md_.occpancy_map_2D_.info.origin.position.x = mp_.map_origin_(0);
  md_.occpancy_map_2D_.info.origin.position.y = mp_.map_origin_(1);
  md_.occpancy_map_2D_.info.origin.position.z = -0.5;
  md_.occpancy_map_2D_.info.origin.orientation.w = 1.0;
  md_.occpancy_map_2D_.data.resize(buffer_size_2D, 0); //TODO: 不是很懂为什么这里是0而不是125
  md_.occpancy_map_2D_binary_ = md_.occpancy_map_2D_;

  md_.tmp_buffer3_ = vector<double>(buffer_size_2D, 0);
  md_.distance_buffer_pos_2D_ = vector<double>(buffer_size_2D, 10000);
  md_.distance_buffer_neg_2D_ = vector<double>(buffer_size_2D, 10000);
  md_.distance_buffer_all_2D_ = vector<double>(buffer_size_2D, 10000);

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/sdf_map/depth", 50));

  // 不是用的odom，用的时POSE
  if (mp_.pose_type_ == POSE_STAMPED) {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/sdf_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));

  } else if (mp_.pose_type_ == ODOMETRY) {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/sdf_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
  }

  // 现在的pose_type_是1，也就是说订阅的是pose和depth image。所以这些都没有用上
  // 不是用的odom
  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

  // indep_pose_sub_ = 
      // node_.subscribe<geometry_msgs::PoseStamped>("/pcl_render_node/camera_pose", 10, &SDFMap::poseCallback, this);
  // 固定时间周期更新地图并发布可视化信息
  occ_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  unknown_pub_2D_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown2D", 10);
  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
  
  // QHB: 2D
  map_pub_2D_ = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D", 10);
  map_pub_2D_binary_ = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D_binary", 10);
  esdf_pub_2D_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf_2D", 10);
  uknown_check_pub_ = node_.advertise<std_msgs::Empty>("/sdf_map/check_point_signal", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;
  has_costmap_ = true;

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  rand_noise2_ = normal_distribution<double>(0, 0.2);
  random_device rd;
  eng_ = default_random_engine(rd());
}

void SDFMap::initMapFromCostMap(ros::NodeHandle& nh)
{
  topo_test_ = true;
  node_ = nh;
  // perception地图为0.02，topo test的为0.1
  mp_.resolution_ = 0.1; //这个我要卡死为0.1。这个是内部地图的resolution，不是订阅的地图的resolution
  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  cost_map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &SDFMap::CostmapCallback, this);
  click_point_sub_ = node_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &SDFMap::ClickpointCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.1), &SDFMap::visCallback, this);
}

void SDFMap::ClickpointCallback(const geometry_msgs::PointStampedConstPtr& click_point)
{
  Eigen::Vector3d pt(click_point->point.x, click_point->point.y, mp_.ground_height_2D_);
  setMapObstacleByPoint(pt, 0.4);
  updateESDF2d();
}

void SDFMap::CostmapCallback(const nav_msgs::OccupancyGridConstPtr &map_msg)
{

  if(has_costmap_) 
  {
    return;
  }
  
  /* get parameter */
  double x_size, y_size, z_size; //米制的
  z_size = mp_.resolution_;
  x_size = map_msg->info.width * map_msg->info.resolution;
  y_size = map_msg->info.height * map_msg->info.resolution;
  int resolution_scale = ceil( map_msg->info.resolution/mp_.resolution_);
  mp_.local_update_range_ = Eigen::Vector3d(5.5, 5.5, 4.5);
  mp_.obstacles_inflation_ = 0.0;
  mp_.frame_id_ = "world";
  mp_.local_bound_inflate_ = 0.0;
  mp_.local_map_margin_ = 10;
  mp_.ground_height_ = -1.0;
  mp_.ground_height_2D_ = mp_.ground_height_;
  mp_.need_map_2D_ = true;

  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);//local map update and clear
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);


  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_; // map_max_boundary_的z会超过0.5 * z_size

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  // QHB: 2D
  int buffer_size_2D = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);
  md_.occupancy_buffer_inflate_2D_ = vector<char>(buffer_size_2D, -1);
  md_.occupancy_buffer_neg_2D_ = vector<char>(buffer_size_2D, 0);  
  md_.occpancy_map_2D_.header.frame_id = mp_.frame_id_;
  md_.occpancy_map_2D_.info.resolution = mp_.resolution_;
  md_.occpancy_map_2D_.info.height = mp_.map_voxel_num_(1); //occpancy_map_2D_的height是行
  md_.occpancy_map_2D_.info.width = mp_.map_voxel_num_(0);  //occpancy_map_2D_的width是列


  md_.occpancy_map_2D_.info.origin.position.x = mp_.map_origin_(0);
  md_.occpancy_map_2D_.info.origin.position.y = mp_.map_origin_(1);
  md_.occpancy_map_2D_.info.origin.position.z = -0.5;
  md_.occpancy_map_2D_.info.origin.orientation.w = 1.0;
  md_.occpancy_map_2D_.data.resize(buffer_size_2D, 0);
  md_.tmp_buffer3_ = vector<double>(buffer_size_2D, 0);
  md_.distance_buffer_pos_2D_ = vector<double>(buffer_size_2D, 10000);
  md_.distance_buffer_neg_2D_ = vector<double>(buffer_size_2D, 10000);
  md_.distance_buffer_all_2D_ = vector<double>(buffer_size_2D, 10000);
  md_.occpancy_map_2D_binary_ = md_.occpancy_map_2D_;
  // QHB: 2D
  map_pub_2D_ = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D", 10);
  esdf_pub_2D_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf_2D", 10);
  map_pub_2D_binary_ = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D_binary", 10);
  
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  vector<Eigen::Vector3i> inf_pts;
  if(mp_.need_map_2D_) inf_pts.resize(pow(2 * inf_step + 1, 2) - 2*2);
  else inf_pts.resize(pow(2 * inf_step + 1, 3));
  Eigen::Vector3i inf_pt;
  
  // 读取costmap的信息
  for(int w = 0; w < map_msg->info.width; ++w)
  {
    for(int h = 0; h < map_msg->info.height; ++h)
    {
      int map_idx = w + h * map_msg->info.width;
      int map_idx2;
      if (map_msg->data[map_idx] == 0)
      {
        for(int i = 0; i < resolution_scale; ++i)
        {
          for(int j = 0; j < resolution_scale; ++j)
          {
            map_idx2 = toAddress2D((int)(resolution_scale * w + i), (int)(resolution_scale * h + j));
            // md_.occpancy_map_2D_.data[map_idx2] = 0;
            md_.occupancy_buffer_inflate_2D_[map_idx2] = 0;            
          }
        }
      }
      else if(map_msg->data[map_idx] != 0)
      {
        for(int i = 0; i < resolution_scale; ++i)
        {
          for(int j = 0; j < resolution_scale; ++j)
          {

            inflatePoint(Eigen::Vector3i(resolution_scale * w + i, resolution_scale * h + j, 0.5),
                          inf_step, inf_pts);
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
        }
      }
    }
  }
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  updateESDF2d();
  has_costmap_ = true;
}

// rest整个地图
void SDFMap::resetBuffer() {
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

//指定立方体的两个顶点，reset这个立方体
void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);
  // 这个也要
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), -1);
  std::fill(md_.occupancy_buffer_inflate_2D_.begin(), md_.occupancy_buffer_inflate_2D_.end(), -1);
  std::fill(md_.occpancy_map_2D_.data.begin(), md_.occpancy_map_2D_.data.end(), 0);  
  std::fill(md_.occpancy_map_2D_binary_.data.begin(), md_.occpancy_map_2D_binary_.data.end(), 0);

  std::fill(md_.distance_buffer_all_2D_.begin(), md_.distance_buffer_all_2D_.end(), 10000);
  std::fill(md_.distance_buffer_pos_2D_.begin(), md_.distance_buffer_pos_2D_.end(), 10000);
  std::fill(md_.distance_buffer_neg_2D_.begin(), md_.distance_buffer_neg_2D_.end(), 10000);

  std::fill(md_.distance_buffer_.begin(), md_.distance_buffer_.end(),10000);
  std::fill(md_.distance_buffer_neg_.begin(), md_.distance_buffer_neg_.end(),10000);
}

// 这个和深蓝学院第一章学的SDF地图构建的抛物线有关
// 描述输入和输出：
template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

// 这个就是深蓝学院第一章的东西。那么2D距离场是不是也可以通过这样的方式来构建和更新呢？
// 只更新了local_bound的一块，具体范围由raycastProcess确定
void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] != 1) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

void SDFMap::updateESDF2d() {
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    fillESDF(
        [&](int y) {
          return md_.occupancy_buffer_inflate_2D_[toAddress2D(x, y)] == 1 ?
              0 :
              std::numeric_limits<double>::max();
        },
        [&](int y, double val) { md_.tmp_buffer3_[toAddress2D(x, y)] = val; }, min_esdf[1],
        max_esdf[1], 1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    fillESDF([&](int x) { return md_.tmp_buffer3_[toAddress2D(x, y)]; },
              [&](int x, double val) {
                md_.distance_buffer_pos_2D_[toAddress2D(x, y)] = mp_.resolution_ * std::sqrt(val);
              },
              min_esdf[0], max_esdf[0], 0);
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y){

        int idx = toAddress2D(x, y);
        if (md_.occupancy_buffer_inflate_2D_[idx] != 1) {
          md_.occupancy_buffer_neg_2D_[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_2D_[idx] == 1) {
          md_.occupancy_buffer_neg_2D_[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    fillESDF(
        [&](int y) {
          return md_.occupancy_buffer_neg_2D_[toAddress2D(x, y)] == 1 ?
              0 :
              std::numeric_limits<double>::max();
        },
        [&](int y, double val) { md_.tmp_buffer3_[toAddress2D(x, y)] = val; }, min_esdf[1],
        max_esdf[1], 1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    fillESDF([&](int x) { return md_.tmp_buffer3_[toAddress2D(x, y)]; },
              [&](int x, double val) {
                md_.distance_buffer_neg_2D_[toAddress2D(x, y)] = mp_.resolution_ * std::sqrt(val);
              },
              min_esdf[0], max_esdf[0], 0);
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y){
      int idx = toAddress2D(x, y);
      md_.distance_buffer_all_2D_[idx] = md_.distance_buffer_pos_2D_[idx];

      if (md_.distance_buffer_neg_2D_[idx] > 0.0)
        md_.distance_buffer_all_2D_[idx] += (-md_.distance_buffer_neg_2D_[idx] + mp_.resolution_);
    }
}



//把当前更新需要的free和occpancy数据都储存起来
int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;//访问次数+1

  //如果是第一次访问这个节点，就把其放到cache_voxel_里面
  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }
  //如果是占据，则放到count_hit_，占据+1
  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage() {
  // md_.proj_points_.clear();
  ros::Time t0 = ros::Time::now();
  md_.proj_points_cnt = 0;

  uint16_t* row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  const int cols = md_.depth_image_.cols;
  const int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_depth(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ point_depth;
  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;
  bool novalid_points_num = 0;
  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; ++v) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; ++u) {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        // if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;
      double pt_dis;
      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_) {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          // depth = mp_.max_ray_length_ + 0.1这个处理非常重要
          // 这里可能会出现一定的错误
          bool exec_max = false;
          if (depth == 0) {
            depth = mp_.max_ray_length_ + 0.1;
            exec_max = true;
          } else if (depth < mp_.depth_filter_mindist_) {
            ++novalid_points_num ;
            continue;
          } 
          // else if (depth > mp_.depth_filter_maxdist_) {
          //   depth = mp_.max_ray_length_ + 0.1;
          //   exec_max = true;
          // }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;
          pt_dis = pt_cur.norm();
          if(pt_dis > (mp_.max_ray_length_ + 0.1))
          {
            pt_cur = (pt_cur) / pt_dis * (mp_.max_ray_length_ + 0.1);//缩放到raycast的范围内
          }

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          // CHANGE
          point_depth.x = pt_world.x();
          point_depth.y = pt_world.y();
          point_depth.z = pt_world.z();
          // cloud_depth->points.push_back(point_depth);
          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false) {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            } else {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud(cloud_depth);
  // sor.setMeanK(20); // 设置K值，即每个点的邻域内点的数量
  // sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
  // sor.filter(*cloud_depth);
  // md_.proj_points_.resize(cloud_depth->points.size());
  // md_.proj_points_cnt = cloud_depth->points.size();
  // for(int i = 0; i < md_.proj_points_cnt; ++i)
  // {
  //   md_.proj_points_[i].x() = cloud_depth->points[i].x;
  //   md_.proj_points_[i].y() = cloud_depth->points[i].y;
  //   md_.proj_points_[i].z() = cloud_depth->points[i].z;
  // }


  /* maintain camera pose for consistency check */
  ros::Time t1 = ros::Time::now();
  // ROS_WARN_STREAM("Project depth image time cost: " << (t1 - t0).toSec() * 1000 
  //                 << "ms. Point cloud size is: " << md_.proj_points_cnt <<", No-valide point num: " 
  //                 << novalid_points_num << ", Total point num: " << md_.depth_image_.cols * md_.depth_image_.rows);

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

//用深度图投影得到的地图点来更新占据栅格地图
//setCacheOccupancy里面存放要处理的voxel。先把投影点放进去，然后把射线上的点都放进去
//对于setCacheOccupancy里面每一个点，判断这一次更新中是occpancy or free，
//occpancy就加个数，否则就减一个数，如果数小于或大于权重，就可以直接判断应该是occpancy or free
void SDFMap::raycastProcess() {
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  // 注意这里min和max是反者赋值的，所以才能不断缩小范围
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // set flag for projected point
    // 这里只是设置了这一个point的占据状态，下面的while是在设置这一条射线上的点的占据状态
    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);//对于超出地图范围的点，找一个最近的点给它，否则在更新呀什么的时候，会超出地图范围而报错

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - md_.camera_pos_).norm();

      //如果这个深度点超出了raycast的范围，则说明要更新的范围内，这些点的ray line都是free的
      //如果length大于max_ray_length_，则把最远点设为free，意味着这一线上所有都是free；如果小于，则将最远点设为occpancy，线上其他点为free
      if (length > mp_.max_ray_length_ + 1e-3) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;//缩放到raycast的范围内
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    //在这条射线上，以half的距离向前遍历
    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // determine the local bounding box for updating ESDF
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  // std::cout << "local_bound: " << Eigen::Vector3d(max_x, max_y, max_z).transpose() << ", " << Eigen::Vector3d(min_x, min_y, min_z).transpose() << std::endl;
  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  // local_range_min这个和rviz里面那个方盒子有关吗？
  // 目前是让无人机在更新空间的正中间
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;
  // 懂了，原来栅格地图是这样更新的
  while (!md_.cache_voxel_.empty()) {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    //
    //如果是占据，则log_odds_update大于0；否则小于0
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? //判断这个voxel是占据orfree
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    //已经确认是占据or free，就直接跳到下一个点
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }
    //这里在进行占据概率更新
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

// 
void SDFMap::clearAndInflateLocalMap() {
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  // 这里只保留了local范围内的地图
  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range。
  // 清除。这里的操作是，将local range的外围vec_margin像素的区域都重置。注意三个方向都进行了



  // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {
  //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {
  //       for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {
  //           // 检查是否在有效范围外
  //           if (z < min_cut(2) || z > max_cut(2) || y < min_cut(1) || y > max_cut(1) || x < min_cut(0) || x > max_cut(0)) {
  //               int idx = toAddress(x, y, z);
  //               md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //               md_.distance_buffer_all_[idx] = 10000;
  //             }
  //         }
  //     }
  // }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts;
  if(mp_.need_map_2D_) inf_pts.resize(pow(2 * inf_step + 1, 2));
  else inf_pts.resize(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  // 只处理local range的occupancy_buffer_inflate_数据。把local range的都重置。
  // 但是这样会造成地图信息丢失，从而发生碰撞
  // occupancy_buffer_inflate_唯一的赋值就是在这下面部分
  // for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  //   for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
  //     for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
  //       md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
  //     }

  // 清除障碍物，是不是和这里有关呢？
  // NOTE: 后续修改要关注这里
  // for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  //   for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
  //     for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

  //       if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_) {
  //         continue;
  //         // 注意，这里，如果是未知的区域，则直接continue了
  //       } else if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_ 
  //                  && md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.clamp_min_log_ + 1e-3
  //                  ) {
  //         auto inx = toAddress(Eigen::Vector3i(x, y, z));
  //         md_.occupancy_buffer_inflate_[inx] = char(0);
  //         // md_.freed_idx_.push_back(Eigen::Vector3i(x, y, z));
  //       }
  //     }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        auto index = toAddress(Eigen::Vector3i(x, y, z));
        // if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) 
        // {
        //   inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
        //   inf_pts.resize(1); //先把膨胀关闭，看看会怎么样
        //   for (int k = 0; k < (int)inf_pts.size(); ++k) {
        //     inf_pt = inf_pts[k];
        //     int idx_inf = toAddress(inf_pt);
        //     if (idx_inf < 0 ||
        //         idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
        //       continue;
        //     }
        //     md_.occupancy_buffer_inflate_[idx_inf] = 1;
        //   }
        // }
        if (md_.occupancy_buffer_[index] > mp_.min_occupancy_log_) 
        {
          md_.occupancy_buffer_inflate_[index] = 1;
        }
        else if (md_.occupancy_buffer_[index] < mp_.min_occupancy_log_ 
                && md_.occupancy_buffer_[index] > mp_.clamp_min_log_ - 1e-3
                ) {
          md_.occupancy_buffer_inflate_[index] = char(0);
          // md_.freed_idx_.push_back(Eigen::Vector3i(x, y, z));
        }
  }
  // add virtual ceiling to limit flight height
  // if (mp_.virtual_ceil_height_ > -0.5) {
  //   int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
  //   for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  //     for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
  //       md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
  //     }
  // }
}

void SDFMap::clearAndInflateLocalMap2D() {
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  // 这里只保留了local范围内的地图
  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);


  // inflate occupied voxels to compensate robot size


  // clear outdated data
  // 只处理local range的occupancy_buffer_inflate_数据。把local range的都重置。
  // 但是这样会造成地图信息丢失，从而发生碰撞
  // occupancy_buffer_inflate_唯一的赋值就是在这下面部分
  // 同样的，这一部分也不要了
  // for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  //   for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y){
  //       md_.occupancy_buffer_inflate_2D_[toAddress2D(x, y)] = 0;
  // }

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts;
  if(inf_step == 0) inf_pts.resize(1);
  else if(mp_.need_map_2D_) inf_pts.resize(pow(2 * inf_step + 1, 2) - 2*2);
  else inf_pts.resize(pow(2 * inf_step + 1, 3));
  Eigen::Vector3i inf_pt;

  // for(const auto free_idx : md_.freed_idx_)
  // {
  //   inflatePoint(free_idx, inf_step, inf_pts);
  //   for (int k = 0; k < (int)inf_pts.size(); ++k) {
  //     inf_pt = inf_pts[k];
  //     int idx_inf = toAddress(inf_pt);
  //     if (idx_inf < 0 ||
  //         idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
  //       continue;
  //     }
  //     md_.occupancy_buffer_inflate_[idx_inf] = 0;
  //   }
  // }
  // md_.freed_idx_.resize(0);
  // std::cout << "clearAndInflateLocalMap2D: 1" << std::endl;
  int z_min = max((int)floor((mp_.camera_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_), md_.local_bound_min_(2));
  int z_max = min((int)floor((mp_.camera_height_ + mp_.height_max_2D_ - mp_.map_origin_(2)) * mp_.resolution_inv_), md_.local_bound_max_(2));
  if(mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    z_max = min(ceil_id, z_max);
  }
  // std::cout << "clearAndInflateLocalMap2D: 2" << std::endl;
  bool has_free = 1;
  // 这里是为了清除被膨胀后的2D地图，清除膨胀地部分
  // 很多地方使用了mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)来判断一个点是否在地图内
  // 但是这样地判断并不能保证点在地图内，也不能保证在3D地图内就会在2D地图内吧
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
    {
      // 0.5是相机高度
      for (int z = (mp_.camera_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_; 
           z <= (mp_.camera_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_; ++z) 
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0
            // && md_.occupancy_buffer_inflate_2D_[toAddress2D(x, y)] != 1
        ) 
        {
          md_.occupancy_buffer_inflate_2D_[toAddress2D(x, y)] = 0;   
        }
      }
      // if(has_free == 1) 
      //   md_.occupancy_buffer_inflate_2D_[toAddress2D(x, y)] = 0;      
    }
  // std::cout << "clearAndInflateLocalMap2D: 3" << std::endl;
  // inflate obstacles
  int flate_num = 0;
  int no_flate_num = 0;
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = z_min; z <= z_max; ++z) 
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1) 
        {
          // no_flate_num ++;
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
          for (int k = 0; k < (int)inf_pts.size(); ++k) 
          {
            inf_pt = inf_pts[k];
            // int idx_inf = toAddress(inf_pt);

            // if (idx_inf >= 0 && idx_inf < mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)
            //     && !this->isInMap(inf_pt)) {
            //   ROS_ERROR("error!!!!!");
            // }
            if(!this->isInMap2D(Eigen::Vector2i(inf_pt.x(), inf_pt.y())))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_2D_[toAddress2D(inf_pt.x(), inf_pt.y())] = 1;
            // flate_num ++;
          }
          break;
        }
      }
  // std::cout << "clearAndInflateLocalMap2D: 4" << std::endl;
  clearMapByRobot(md_.camera_pos_);
  // std::cout << "clearAndInflateLocalMap2D: 5" << std::endl;
    // std::cout << "occupancy_buffer_inflate_2D_ num: " << no_flate_num << ", " << flate_num << std::endl;

}



void SDFMap::visCallback(const ros::TimerEvent& /*event*/) {
  if(!has_costmap_) return;
  // 要注意，这个地方的都只是可视化，真正使用的信息不完全是可视化的结果
  // 这里要说明的是，在md_里面储存的是整张地图的占据data，可以选择把整个占据信息都发布出来
  // 当前程序里面只选择发布了local range的部分

  if(mp_.need_map_2D_)
  {
    publishMap2D();
    publishESDF2D();
    // if(!topo_test_) publishUnknown2D();
  }
  if(topo_test_) return;
  publishMap();//发布local地图的占据栅格地图.
  // // publishMapInflate(true);//发布膨胀后的local地图的占据栅格地图。不知道all_info是什么
  publishUpdateRange();//local map的范围
  // publishESDF();//是膨胀了一个local_map_margin的ESDF，可视化比较好看？

  publishUnknown();//unknown是指occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_的区域。表示了感知的边界
  publishDepth();

}

// 非常关键的函数
void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.occ_need_update_) return;

  /* update occupancy */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  if (!md_.sub_cloudpoint_) projectDepthImage();
  raycastProcess();

  if (md_.local_updated_) clearAndInflateLocalMap();
  if (md_.local_updated_ && mp_.need_map_2D_) clearAndInflateLocalMap2D();

  t2 = ros::Time::now();

  md_.fuse_time_ += (t2 - t1).toSec();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  if (mp_.show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;
  md_.local_updated_ = false;
}

void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();
  if(mp_.need_map_2D_)
    updateESDF2d();
  t2 = ros::Time::now();

  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}

// 现在用的这个
void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // ROS_WARN_STREAM("depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows);

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  if (md_.has_first_depth_) return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

// 为什么要有这个callback函数？不是已经有东西可以接收了吗？
void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;
  md_.proj_points_cnt = 0;
  if (!md_.has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0) return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;

  // this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
  //                   md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (devi.squaredNorm() < 64) md_.proj_points_[md_.proj_points_cnt++] = p3d;
    // if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
    //     fabs(devi(2)) < mp_.local_update_range_(2)) {

      /* inflate the point */
      // for (int x = -inf_step; x <= inf_step; ++x)
      //   for (int y = -inf_step; y <= inf_step; ++y)
      //     for (int z = -inf_step_z; z <= inf_step_z; ++z) {

      //       p3d_inf(0) = pt.x + x * mp_.resolution_;
      //       p3d_inf(1) = pt.y + y * mp_.resolution_;
      //       p3d_inf(2) = pt.z + z * mp_.resolution_;

      //       max_x = max(max_x, p3d_inf(0));
      //       max_y = max(max_y, p3d_inf(1));
      //       max_z = max(max_z, p3d_inf(2));

      //       min_x = min(min_x, p3d_inf(0));
      //       min_y = min(min_y, p3d_inf(1));
      //       min_z = min(min_z, p3d_inf(2));

      //       posToIndex(p3d_inf, inf_pt);

      //       if (!isInMap(inf_pt)) continue;

      //       int idx_inf = toAddress(inf_pt);

      //       // md_.occupancy_buffer_inflate_[idx_inf] = 1;
      //       md_.proj_points_[md_.proj_points_cnt++] = p3d_inf;
      //     }
    // }
  }

  // min_x = min(min_x, md_.camera_pos_(0));
  // min_y = min(min_y, md_.camera_pos_(1));
  // min_z = min(min_z, md_.camera_pos_(2));

  // max_x = max(max_x, md_.camera_pos_(0));
  // max_y = max(max_y, md_.camera_pos_(1));
  // max_z = max(max_z, md_.camera_pos_(2));

  // max_z = max(max_z, mp_.ground_height_);

  // posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  // posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  // boundIndex(md_.local_bound_min_);
  // boundIndex(md_.local_bound_max_);

  // md_.esdf_need_update_ = true;
  md_.sub_cloudpoint_ = true;
  md_.occ_need_update_ = true;
}

//将当前的？占据的栅格发布出来
void SDFMap::publishMap() {
  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Eigen::Vector3i min_cut = md_.local_bound_min_ -
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // Eigen::Vector3i max_cut = md_.local_bound_max_ +
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);

  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // for (int x = min_cut(0); x <= max_cut(0); ++x)
  //   for (int y = min_cut(1); y <= max_cut(1); ++y)
  //     for (int z = min_cut(2); z <= max_cut(2); ++z) {

  //       if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

  //       Eigen::Vector3d pos;
  //       indexToPos(Eigen::Vector3i(x, y, z), pos);
  //       if (pos(2) > mp_.visualization_truncate_height_) continue;

  //       pt.x = pos(0);
  //       pt.y = pos(1);
  //       pt.z = pos(2);
  //       cloud.points.push_back(pt);
  //     }

  // cloud.width = cloud.points.size();
  // cloud.height = 1;
  // cloud.is_dense = true;
  // cloud.header.frame_id = mp_.frame_id_;

  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(cloud, cloud_msg);
  // map_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  if(pub_all_map_)
  {
    min_cut = Eigen::Vector3i::Zero();
    max_cut = mp_.map_voxel_num_;    
  }
  max_cut.z() = min((int)floor((2.5 - mp_.map_origin_(2)) * mp_.resolution_inv_), md_.local_bound_max_(2));
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] != 1) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void SDFMap::publishMap2D() {


  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  if(pub_all_map_)
  {
    min_cut = Eigen::Vector3i::Zero();
    max_cut = mp_.map_voxel_num_;    
  }
  static Eigen::Vector2i check_coord;
  posToIndex2D(check_point_, check_coord);
  static const int check_idx = toAddress2D(check_coord);
  char map_value_last = md_.occpancy_map_2D_.data[check_idx];

  boundIndex(min_cut);  
  boundIndex(max_cut);
  for (int y = min_cut(1); y <= max_cut(1); ++y)
    for (int x = min_cut(0); x <= max_cut(0); ++x)
    {
      int map_idx = toAddress2D(x, y);
      // int map_idx_occp = y *  
      if (md_.occupancy_buffer_inflate_2D_[map_idx] == 0) //free
      {
        md_.occpancy_map_2D_.data[map_idx] = 0;
      }
      else if(md_.occupancy_buffer_inflate_2D_[map_idx] == 1) //occ
      {
        md_.occpancy_map_2D_.data[map_idx] = 255;
        md_.occpancy_map_2D_binary_.data[map_idx] = 100;
        // std::cout << "设置了occpancy_map?" << std::endl;
      }
      else     //unknown, md_.occupancy_buffer_inflate_2D_[map_idx] == -1
      {
        md_.occpancy_map_2D_.data[map_idx] = 125;
      }
    }
  md_.occpancy_map_2D_.header.stamp = ros::Time::now();
  md_.occpancy_map_2D_binary_.header.stamp = ros::Time::now();
  map_pub_2D_.publish(md_.occpancy_map_2D_);
  map_pub_2D_binary_.publish(md_.occpancy_map_2D_binary_);

  char map_value_curr = md_.occpancy_map_2D_.data[check_idx];
  // std::cout << "map_value_last: " << (char)map_value_last <<  ", map_value_curr: " << (char)map_value_curr << std::endl; 
  // 这时，check_point刚被发现占据，发布一个信号出来。
  if((map_value_last == (char)0 && map_value_curr == (char)255) || 
     (map_value_last == (char)125 && map_value_curr == (char)255))
  {
    std_msgs::Empty emt;
    uknown_check_pub_.publish(emt);
    // ROS_ERROR_STREAM("Pub !!!!!!");
  }

}



void SDFMap::publishMapInflate(bool all_info) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info) {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] != 1) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub local map inflate, size is %i.", cloud.points.size());
}

void SDFMap::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        // if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == -1) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void SDFMap::publishUnknown2D() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  int z_min = max((int)floor((mp_.ground_height_2D_ - mp_.map_origin_(2)) * mp_.resolution_inv_), md_.local_bound_min_(2));
  int z_max = min((int)floor((mp_.ground_height_2D_ + mp_.height_max_2D_ - mp_.map_origin_(2)) * mp_.resolution_inv_), md_.local_bound_max_(2));
  if(mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    z_max = min(ceil_id, z_max);
  }
  z_min = z_max = (mp_.camera_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_;//直接设置为相机的高度的扇形.这个是int型的
  int state = 0; // 0: unknown, 1: occupy, 2: free;

  // NOTE: 后续修改要关注这里
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
    {
      for (int z = z_min; z <= z_max; ++z)
      {
        // if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == -1) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, mp_.ground_height_2D_), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_2D_.publish(cloud_msg);
}


void SDFMap::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double max_dis = mp_.max_ray_length_ * mp_.max_ray_length_;
  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    // if((md_.proj_points_[i] - md_.camera_pos_).squaredNorm() > max_dis) continue;
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  // Eigen::Quaterniond camera_q = md_.camera_q_;
  // Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();
  // Eigen::Vector3d camera_p = md_.camera_pos_;
  // Eigen::Matrix4d camera_pose = Eigen::Matrix4d::Identity();
  // camera_pose.block<3, 3>(0, 0) = camera_r;
  // camera_pose.block<3, 1>(0, 3) = camera_p;
  // pcl::transformPointCloud(cloud, cloud, camera_pose.inverse().cast<float>());

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void SDFMap::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void SDFMap::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void SDFMap::publishESDF2D() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // min_cut = Eigen::Vector3i::Zero();
  // max_cut = mp_.map_voxel_num_;    
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector2i posidx(x, y);
      boundIndex2D(posidx);

      dist = getDistance2D(posidx);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      Eigen::Vector2d pos;
      indexToPos2D(posidx, pos);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = mp_.ground_height_2D_;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_2D_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}


void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                          vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad, int sign) {
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {

      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist() {
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0) {
        }
      }
}

bool SDFMap::odomValid() { return md_.has_odom_; }

bool SDFMap::hasDepthObservation() { return md_.has_first_depth_; }

double SDFMap::getResolution() { return mp_.resolution_; }

Eigen::Vector3d SDFMap::getOrigin() { return mp_.map_origin_; }

int SDFMap::getVoxelNum() {
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                            Eigen::Vector3d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::getSurroundPts2D(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2],
                            Eigen::Vector3d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d(1, 1, 0);
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, 0);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y] = current_pos;
      
    }
  }
}

void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) {
  /* get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  // md_.camera_pos_(2) = 0.0;
  // ROS_INFO("----------");

  // 这里其实是lidar的姿态
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << 0, 0, 1,
                     -1, 0, 0,
                     0, -1, 0;

  // 使用Eigen的内置函数将旋转矩阵转换为四元数
  Eigen::Quaterniond q_rot(rotation_matrix);              

  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z) * q_rot;

  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

}

void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img) {
  std::cout << "depth: " << img->header.stamp << std::endl;
}

void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
  std::cout << "pose: " << pose->header.stamp << std::endl;

  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
}

// SDFMap
