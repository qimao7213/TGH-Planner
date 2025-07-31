
#include "dvr/voronoi_layer.h"
// #define VERBOSE
bool debug_flag_ = false;
using namespace gvg;
namespace DynaVoro
{
VoronoiLayer::VoronoiLayer(ros::NodeHandle& nh)
{
  double resolutiond, clearance_lowd, clearance_highd;
  nh.param("voro_map_resolution", resolutiond, 0.1);
  nh.param("obs_clearance", clearance_lowd, 0.3);
  nh.param("obs_clearance_high", clearance_highd, 5.0);
  resolution_ = static_cast<float>(resolutiond);
  clearance_low_ = static_cast<float>(clearance_lowd);   //小于这个值的维诺点不要
  clearance_high_ = static_cast<float>(clearance_highd); //大于这个值的维诺点不要,包络线
  ROS_WARN_STREAM("VoronoiLayer: resolution: " << resolution_ << ", clearance_low: " << clearance_low_ << ", clearance_high: " << clearance_high_);
  // visualization_.reset(new fast_planner::PlanningVisualization(nh));
  clearance_low_thr_ = clearance_low_ * clearance_low_ / resolution_ / resolution_;
  clearance_high_thr_ = clearance_high_ * clearance_high_ / resolution_ / resolution_;

  gvg_ = std::make_shared<GVG>();
  gvg_->setClearanceThresholdSq(clearance_low_thr_, clearance_high_thr_);
  gvg_planner_ = std::make_shared<gvg::Planner>();
  gvg_planner_->init(gvg_);

  costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D", 1, &VoronoiLayer::costmapCB, this);
  odometry_sub_ = nh.subscribe<nav_msgs::Odometry>("/car_odom", 1, &VoronoiLayer::odometryCallback, this);
  
  voronoi_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi_grid", 10);
  voronoi_grid_origin_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi_grid_origin", 10);
  voronoi_occupy_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi_grid_occupy", 10);
  distance_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voronoi_esdf", 10);
  gvg_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/voronoi_gvg_markers", 10);
  path_pub_ = nh.advertise<nav_msgs::Path>("/voronoi_path", 10);
  path_pub2_ = nh.advertise<nav_msgs::Path>("/voronoi_path_short", 10);
  path_pub_test_1_ = nh.advertise<nav_msgs::Path>("/voronoi_path_test1", 10);
  path_pub_test_2_ = nh.advertise<nav_msgs::Path>("/voronoi_path_test2", 10);
  path_pub_test_3_ = nh.advertise<nav_msgs::Path>("/voronoi_path_test3", 10);
  
  // rviz_goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &VoronoiLayer::rvizGoalCallback, this);
  // plan_timer_ = nh.createTimer(ros::Duration(0.2), &VoronoiLayer::planTimerCallback, this);
  voronoi_update_timer_ = nh.createTimer(ros::Duration(2), &VoronoiLayer::voronoiUpdateTimerCallback, this);
}


const DynamicVoronoi& VoronoiLayer::getVoronoi() const
{
  return voronoi_;
}

boost::mutex& VoronoiLayer::getMutex()
{
  return mutex_;
}

void VoronoiLayer::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
  unsigned char* pc = costarr;
  for (int i = 0; i < nx; ++i)
  {
    *pc++ = value;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; ++i)
  {
    *pc++ = value;
  }
  pc = costarr;
  for (int i = 0; i < ny; ++i, pc += nx)
  {
    *pc = value;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; ++i, pc += nx)
  {
    *pc = value;
  }
}

void VoronoiLayer::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{

  boost::unique_lock<boost::mutex> lock(mutex_);

  unsigned int size_x = map_msg->info.width;
  unsigned int size_y = map_msg->info.height;

  if (last_size_x_ != size_x || last_size_y_ != size_y)
  {
    voronoi_.initializeEmpty(size_x, size_y);
    for (unsigned int i = 0; i < size_x; ++i) {
      voronoi_.occupyCell(i, 0);           // 上边界
      voronoi_.occupyCell(i, size_y - 1); // 下边界
    }
    for (unsigned int j = 0; j < size_y; ++j) {
        voronoi_.occupyCell(0, j);           // 左边界
        voronoi_.occupyCell(size_x - 1, j); // 右边界
    }
    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  std::vector<IntPoint> new_free_cells, new_occupied_cells;
  for (unsigned int i = 1; i < size_x - 2; ++i)
  {  
    for (unsigned int j = 1; j < size_y - 2; ++j)
    {
      int index = j * size_x + i;
      if (voronoi_.isOccupied(i, j) && map_msg->data[index] != -1)
      {
        new_free_cells.emplace_back(i, j);
      }

      if (!voronoi_.isOccupied(i, j) && map_msg->data[index] == -1)
      // if (!voronoi_.isOccupied(i, j) && map_msg->getCost(i, j) >= 128)
      {
        new_occupied_cells.emplace_back(i, j);
      }
    }
  }
  if (new_free_cells.empty() && new_occupied_cells.empty())
  {
    return; // 没有变化
  }
  for (size_t i = 0; i < new_free_cells.size(); ++i)
  {
    voronoi_.clearCell(new_free_cells[i].x, new_free_cells[i].y);
  }

  for (size_t i = 0; i < new_occupied_cells.size(); ++i)
  {
    voronoi_.occupyCell(new_occupied_cells[i].x, new_occupied_cells[i].y);
  }

  // start timing
  SimpleTimer timer;
  voronoi_.update();
  voronoi_.prune();
  // 不要使用这个，反而效果不好
  // voronoi_.updateAlternativePrunedDiagram();

  #ifdef VERBOSE
  timer.print("Voronoi Update");    
  #endif
  timer.reset();
  gvg_->createGraph(voronoi_);
  gvg_updated_ = true;
  #ifdef VERBOSE
  timer.print("GVG Create"); 
  #endif
  // ------------- Drawing ----------------------
  // 将找到的图结构画到cv::Mat上
  // cv::Mat voronoiImg = cv::Mat::zeros(size_y, size_x, CV_8UC3);
  // voronoiImg.setTo(cv::Scalar(255, 255, 255));
  // cv::Mat voronoiImg2 = voronoiImg.clone();
  // cv::Mat voronoiImg3 = voronoiImg.clone();
  // for (int y = 0; y < size_y; ++y)
  // {
  //   for (int x = 0; x < size_x; ++x)
  //   {
  //     if (voronoi_.isVoronoiWithDisThr(x, y, clearance_low_thr_))
  //     {
  //       voronoiImg.at<cv::Vec3b>(size_y - y - 1, x)[0] = 255;   // 蓝色通道
  //       voronoiImg.at<cv::Vec3b>(size_y - y - 1, x)[1] = 255;   // 绿色通道
  //       voronoiImg.at<cv::Vec3b>(size_y - y - 1, x)[2] = 0;   // 红色通道
  //     }
  //   }
  // }                     
  // auto gvg_graphs = gvg_->getGraphs();
  // for(int i = 0; i < gvg_graphs.size(); ++i)
  // {
  //   auto graph = gvg_graphs[i];
  //   int index = 0;
  //   for(auto& node : graph)
  //   {
  //     auto node_ptr = node.second;
  //     IntPoint node_pos = node_ptr->pos;
  //     if (node_ptr->type == GraphNode::Strong) // 绘制强节点
  //     {
  //       // cv::circle(voronoiImg, cv::Point(node_pos.x, size_y - node_pos.y - 1), 1, cv::Scalar(0, 0, 255), -1);
  //       voronoiImg.at<cv::Vec3b>(size_y - node_pos.y - 1, node_pos.x) = cv::Vec3b(0, 0, 255);   
  //     }
  //     else if (node_ptr->type == GraphNode::Weak) // 绘制弱节点
  //     {
  //       // cv::circle(voronoiImg, cv::Point(node_pos.x, size_y - node_pos.y - 1), 1, cv::Scalar(255, 0, 0), -1);
  //       voronoiImg.at<cv::Vec3b>(size_y - node_pos.y - 1, node_pos.x) = cv::Vec3b(255, 0, 0);   
  //     }
  //     std::vector<Path> paths = node_ptr->neighbor_paths;
  //     for (const auto& path : paths)
  //     {
  //       if (path.path.empty())
  //       {
  //         continue;
  //       }
  //       for (size_t j = 0; j < path.path.size(); ++j)
  //       {
  //         // std::cout << "x, y: " << path.path[j].x << ", " << size_y - path.path[j].y - 1 << std::endl;
  //         voronoiImg.at<cv::Vec3b>(size_y - path.path[j].y - 1, path.path[j].x)[0] = 0;   // 蓝色通道
  //         voronoiImg.at<cv::Vec3b>(size_y - path.path[j].y - 1, path.path[j].x)[1] = 255; // 绿色通道
  //         voronoiImg.at<cv::Vec3b>(size_y - path.path[j].y - 1, path.path[j].x)[2] = 0;   // 红色通道
  //       }
  //     }
  //     int debug = 0;
  //     ++index;
  //   }
  //   int debug = 0;
  // }

  // auto truncated_points = gvg_->getTruncatedPoints();
  // for (const auto& point : truncated_points)
  // {
  //   // cv::circle(voronoiImg, cv::Point(point.x, size_y - point.y - 1), 1, cv::Scalar(0, 255, 255), -1);
  //   // std::cout << "Truncated Point: " << point.x << ", " << point.y;
  //   // std::cout << ", Distance: " << voronoi_.getDistanceSq(point.x, point.y) << std::endl;
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[0] = 100;   // 蓝色通道
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[1] = 20; // 绿色通道
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[2] = 150;   // 红色通道
  //   int debug = 0;
  // }
  // auto completeCoonction_points = gvg_->getCompleteConnectionPoints();
  // for (const auto& point : completeCoonction_points)
  // {
  //   // cv::circle(voronoiImg, cv::Point(point.x, size_y - point.y - 1), 1, cv::Scalar(255, 0, 255), -1);
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[0] = 255;   // 蓝色通道
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[1] = 0; // 绿色通道
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[2] = 255;   // 红色通道
  //   int debug = 0;
  // }
  // cv::Mat esdf_img = cv::Mat::zeros(size_y, size_x, CV_8UC3);
  // drawESDFToMat(esdf_img);       
  // int debug = 0;

  // 将占据地图、voronoi的is_voronoi和gvg里面的is_voronoi都存放下来
  // cv::Mat voronoiMix = cv::Mat::zeros(size_y, size_x, CV_8UC3);
  // static int idx = 0;
  // for (int y = 0; y < size_y; ++y)
  // {
  //   for (int x = 0; x < size_x; ++x)
  //   {
  //     if (voronoi_.isVoronoi(x, y))
  //     {
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[0] = 0;   // 蓝色通道
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[1] = 0;   // 绿色通道
  //       voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[2] = 255;   // 红色通道
  //     }
  //     if (gvg_->isVoronoi(x, y))
  //     {
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[0] = 0;   // 蓝色通道
  //       voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[1] = 255;   // 绿色通道
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[2] = 0;   // 红色通道
  //     }      
  //     if (voronoi_.isOccupied(x, y))
  //     {
  //       voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[0] = 255;   // 蓝色通道
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[1] = 0;   // 绿色通道
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[2] = 0;   // 红色通道
  //     }
  //   }
  // } 
  // cv::imwrite("/home/bhrqhb/catkin_ws/planner/catkin_ws_TGH_Planner/src/TGH-Planner/uav_simulator/dynamicvoronoi/data/" + std::to_string(idx) + ".png", voronoiMix);
  // ++idx;


  // ------------- Drawing End ----------------------

  publishGVG(gvg_->getGraphs());
  publishVoronoiGrid();
  // publishDistanceCloud();
}

void VoronoiLayer::publishVoronoiGrid()
{
  nav_msgs::OccupancyGrid grid, grid_occupy;
  // Publish Whole Grid
  grid.header.frame_id = "world";
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution_;

  grid.info.width = last_size_x_;
  grid.info.height = last_size_y_;

  grid.info.origin.position.x = -0.5 * last_size_x_ * resolution_;
  grid.info.origin.position.y = -0.5 * last_size_y_ * resolution_;
  grid.info.origin.position.z = 0.1;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(last_size_x_ * last_size_y_, 0);
  grid_occupy = grid;
  // SimpleTimer timer;
  // timer.reset();
  for (unsigned int x = 0; x < last_size_x_; ++x)
  {
    for (unsigned int y = 0; y < last_size_y_; ++y)
    {
      // XX
      // if (voronoi_.isVoronoiAlternative(x, y))
      if (voronoi_.isVoronoi(x, y))
      {
        grid.data[x + y * last_size_x_] = 128;
      }
      if (voronoi_.isOccupied(x, y))
      {
        grid_occupy.data[x + y * last_size_x_] = -1;
      }
    }
  }
  // timer.print("XXXXXXXXXXXXXXXXXXXXXXXX");
  voronoi_grid_origin_pub_.publish(grid);
  voronoi_occupy_pub_.publish(grid_occupy);
}


void VoronoiLayer::publishGVG(const std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>& gvg)
{
    size_t previous_marker_count = 0; // 静态变量，记录上一次发布的标记数量
    visualization_msgs::MarkerArray marker_array;
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = "world";
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution_;
  
    grid.info.width = last_size_x_;
    grid.info.height = last_size_y_;
  
    grid.info.origin.position.x = -0.5 * last_size_x_ * resolution_;
    grid.info.origin.position.y = -0.5 * last_size_y_ * resolution_;
    grid.info.origin.position.z = -0.1;
    grid.info.origin.orientation.w = 1.0;
  
    grid.data.resize(last_size_x_ * last_size_y_, 0);
    // 1. 添加所有当前需要的节点marker
    size_t marker_count = 0;
    for (size_t graph_idx = 0; graph_idx < gvg.size(); ++graph_idx)
    {
        const auto& graph = gvg[graph_idx];
        for (const auto& node_pair : graph)
        {
            const auto& node_ptr = node_pair.second;
            const IntPoint& node_pos = node_ptr->pos;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "gvg_vectices";
            marker.id = marker_count++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = coordToWorldX(node_pos.x);
            marker.pose.position.y = coordToWorldY(node_pos.y);
            marker.pose.position.z = 0.1;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            if (node_ptr->type == GraphNode::Strong)
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            }
            else if (node_ptr->type == GraphNode::Weak)
            {
                // continue; // 不发布弱节点
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;
            }
            else
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            }

            marker_array.markers.emplace_back(marker);
            std::vector<Path> paths = node_ptr->neighbor_paths;
            grid.data[node_ptr->pos.x + node_ptr->pos.y * last_size_x_] = 128; // 标记为Voronoi点
            for (int i = 0; i < paths.size(); ++i)
            {
              const auto& path = paths[i];
              if(node_ptr->neighbors[i].lock()->type == GraphNode::Weak)
              {
                // continue; // 不发布弱节点的路径
              }
              // grid.date
              for (size_t j = 0; j < path.path.size(); ++j)
              {
                grid.data[path.path[j].x + path.path[j].y * last_size_x_] = 128; // 标记为Voronoi点
              }
            }
        }
    }

    // 2. 删除多余的旧marker
    for (size_t i = marker_count; i < previous_marker_count; ++i)
    {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "gvg_vectices";
        delete_marker.id = i;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.emplace_back(delete_marker);
    }

    previous_marker_count = marker_count;
    gvg_marker_pub_.publish(marker_array);
    voronoi_grid_pub_.publish(grid);
}

void VoronoiLayer::publishDistanceCloud()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.clear();

  const float min_dist = 0.0;
  const float max_dist = 3.0;
  for (int x = 0; x < last_size_x_; ++x)
  {
    for (int y = 0; y < last_size_y_; ++y)
    {
      // if(x != 120) continue;
      float dist = (voronoi_.getDistance(x, y)) * resolution_;

      dist = std::min(dist, (max_dist));
      dist = std::max(dist, (min_dist));

      pcl::PointXYZI pt;
      pt.x = coordToWorldX(x);
      pt.y = coordToWorldX(y);
      pt.z = 0.1; // 或根据需要设置高度
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.emplace_back(pt);
      // if(x == 120) std::cout << "x, y: " << pt.x << ", " << pt.y << ", dist: " << dist << std::endl;      
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();

  distance_cloud_pub_.publish(cloud_msg);
}


// 返回As中距离B最近的K个点
std::vector<IntPoint> findKNearestPoints( const std::vector<IntPoint>& As, 
                      const IntPoint& B, int K)
{
    // 计算所有A到B的距离
    std::vector<std::pair<double, int>> dist_idx;
    for (int i = 0; i < As.size(); ++i) {
        double dist = std::hypot(As[i].x - B.x, As[i].y - B.y);
        dist_idx.emplace_back(dist, i);
    }
    // 排序
    std::sort(dist_idx.begin(), dist_idx.end());
    // 取前K个
    std::vector<IntPoint> result;
    for (int k = 0; k < std::min(K, (int)dist_idx.size()); ++k) {
        result.push_back(As[dist_idx[k].second]);
    }
    return result;
}


double pathLength(const vector<Eigen::Vector2d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}
double pathLength2(const vector<IntPoint>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < path.size() - 1; ++i) {
    length += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  }
  return length;
}

bool VoronoiLayer::plan(Eigen::Vector3d start, Eigen::Vector3d goal, double start_yaw)
{

  if ((goal - goal_last_).norm() > 5.0)
  {
    last_best_path_.clear();
    goal_last_ = goal;
  }
  voro_paths_raw_world_.clear();
  voro_paths_shortcut_.clear();
  // start.x() = 18.7321;
  // start.y() = -7.11596;
  // start.z() = 1.25864;
  // goal.x() = -6.66841;
  // goal.y() = -4.21227;
  // goal.z() = 2.32283;
  // if (!gvg_updated_)
  // {
  //   ROS_WARN_STREAM("GVG is not updated, can not plan path."); 
  //   debug_flag_ = true;
  //   // return false;
  // }
  // else
  // {
  //   debug_flag_ = false;
  // }
  std::cout << "plan start: " << start.transpose() << ", goal: " << goal.transpose() << std::endl;
  int KdNeighborNum = 8; // KD-Tree最近邻搜索的数量
  //先将start和goal转换为int类型，在map的坐标系下
  Eigen::Vector2i start_coord, goal_coord;
  start_coord.x() = WorldToMapX(start.x());
  start_coord.y() = WorldToMapY(start.y());
  goal_coord.x() = WorldToMapX(goal.x());
  goal_coord.y() = WorldToMapY(goal.y());

  Eigen::Vector2d colli_pt, start_pt, goal_pt;
  start_pt.x() = start_coord.x();
  start_pt.y() = start_coord.y();
  goal_pt.x() = goal_coord.x();
  goal_pt.y() = goal_coord.y();
  if (lineVisib(start_pt, goal_pt, (clearance_low_ - 0.05) / resolution_, colli_pt)) {
    publishPath({});
    publishPath2({start_pt, goal_pt});
    ROS_WARN_STREAM("Start and goal are visible, return straight line path.");
    last_best_path_ = discretizePath({start_pt, goal_pt});

    std::vector<Eigen::Vector2d> voro_path_shortcut;
    voro_path_shortcut.emplace_back(start.head<2>());
    voro_path_shortcut.emplace_back(goal.head<2>());
    voro_paths_shortcut_.emplace_back(voro_path_shortcut);
    return true;
  }

  vector<IntPoint> strong_nodes;
  SimpleTimer timer;
  gvg_->getStrongNodes(strong_nodes);
  ROS_WARN_STREAM("There are " << gvg_->getGraphsSize() << " GVG graphs. Strong nodes: " 
                      << strong_nodes.size());
  KdNeighborNum = std::min(KdNeighborNum, static_cast<int>(strong_nodes.size()));
  if (strong_nodes.empty())
  {
    publishPath({});
    publishPath2({start_pt, goal_pt});
    ROS_WARN_STREAM("There is no graph. Return the stright of start and end.");
    last_best_path_ = discretizePath({start_pt, goal_pt});

    std::vector<Eigen::Vector2d> voro_path_shortcut;
    voro_path_shortcut.emplace_back(start.head<2>());
    voro_path_shortcut.emplace_back(goal.head<2>());
    voro_paths_shortcut_.emplace_back(voro_path_shortcut);
    return true;
  }

  // 先判断起点和终点是否在图中，如果不在，则将其初始化为GraphNode
  // 找到离它num个最近的节点，然后从当前位置raycast到每个节点，如果路径无碰撞，则将最近节点作为其neibor，并修改原graph的结构
  // 如果所有路径都有碰撞，则将最近的那个节点作为StartNode或者GoalNode
  // TODO：这里不应该只考虑强节点，而是把弱节点也考虑进去。
  // TODO: 在cmu_local_planner的terrian map里面，应该把小车身后点云去除掉
  // TODO: 在Indoor这个rosbag上面，最后的时刻不能很好地规划轨迹，看看是什么问题。
  // TODO: 这个上图的方式可能还是有点问题， 比如在zigzag的时候，会找到一条往后的初始路径，然后short的时候就很不容易成功
  int start_graph_id = -1;
  int end_graph_id = -1;
  bool start_on_graph = false;
  bool end_on_graph = false;
  GraphNode::Ptr start_tmp_node_ptr = std::make_shared<GraphNode>();
  GraphNode::Ptr end_tmp_node_ptr = std::make_shared<GraphNode>();
  start_tmp_node_ptr->pos = IntPoint(-1, -1);
  end_tmp_node_ptr->pos = IntPoint(-1, -1);

  auto it_start = creatPathNode(
      start_coord, start_graph_id, start_on_graph, strong_nodes, KdNeighborNum, true, start_tmp_node_ptr);
  // #ifdef VERBOSE
    timer.print("Plan KD-Tree Start Node");
  // #endif
  timer.reset();
  if (!start_on_graph) 
  {
    gvg_->insertNodeToGraph(it_start, start_graph_id);
    strong_nodes.push_back(it_start->pos);
  }
  if (!(start_tmp_node_ptr->pos == IntPoint(-1, -1)))
  {
    gvg_->insertNodeToGraph(start_tmp_node_ptr, start_graph_id);
    strong_nodes.push_back(start_tmp_node_ptr->pos);
  }
   
  auto it_goal = creatPathNode(
      goal_coord, end_graph_id, end_on_graph, strong_nodes, KdNeighborNum, false, end_tmp_node_ptr);
  // #ifdef VERBOSE
    timer.print("Plan KD-Tree End Node");
  // #endif
  timer.reset();  

  ROS_WARN_STREAM("The [start] node connect to graph: " << start_graph_id);
  ROS_WARN_STREAM("The [ end ] node connect to graph: " << end_graph_id);
  if(start_graph_id < 0)
  {
    ROS_ERROR_STREAM("The start node is not connected to any grpgh, can not find path");
  }
  if(end_graph_id < 0)
  {
    ROS_ERROR_STREAM("The end node is not connected to any grpgh, can not find path");
  }
  if(end_graph_id != start_graph_id)
  {
    ROS_ERROR_STREAM("The start and end nodes are connected to different grpgh, can not find path");
    gvg_updated_ = false;
    if (!start_on_graph) removeNodeAndConnections(it_start);
    if (!end_on_graph) removeNodeAndConnections(it_goal);
    if (!(start_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndRepairStrongConnection(start_tmp_node_ptr);
    if (!(end_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndRepairStrongConnection(end_tmp_node_ptr);
    return false;
  }
  // timer.reset();
  // auto topo_path = gvg_planner_->searchTopoPaths(it_start, it_goal, 50);
  // timer.print("Plan topo paths");
  // for (const auto& path : topo_path)
  // {
  //   publishPath(path);
  //   ros::Duration(0.03).sleep();
  //   int debug = 0;
  // }

  // 在这里进行10次重复规划
  // 轨迹的发布和可视化也在另外的地方去进行
  std::vector<std::vector<Eigen::Vector2d>> voro_paths_raw;      //这个是在图像坐标系下的

  int path_found_num = 0;
  int path_search_times = 3;
  double min_path_length = 10000000;
  for (int iter = 0; iter < path_search_times; ++iter)
  {
    timer.reset();
    if(gvg_planner_->serachPath(it_start, it_goal))
    {
      // #ifdef VERBOSE
        timer.print("Plan One A* Search"); 
      // #endif
      std::vector<IntPoint> path_nodes_sparse = gvg_planner_->getFullPathNodes();
      std::vector<IntPoint> path_nodes = gvg_planner_->getFullPath();
      if (path_nodes.size() > 4 * min_path_length && path_nodes.size() > 3 * last_size_x_)
      {
        ROS_WARN_STREAM("Break because current's raw path is more than 5 times longer of min_path_length.");
        break;
      }      
      // publishPath(path_nodes);
      timer.reset();
      vector<Eigen::Vector2d> short_path = shortcutPath(path_nodes, 0, 2);
      // #ifdef VERBOSE
        timer.print("Plan Short Path");
      // #endif
      // publishPath2(short_path);
      voro_paths_raw.emplace_back(short_path);
      gvg_updated_ = false;
      path_found_num ++;

      // 这都是在世界坐标系下的，要传递出去的
      std::vector<Eigen::Vector2d> voro_path_raw;
      std::vector<Eigen::Vector2d> voro_path_shortcut;
      // for(const auto& pt : path_nodes) voro_path_raw.emplace_back(coordToWorldX(pt.x), coordToWorldY(pt.y));
      // voro_paths_raw_world_.emplace_back(voro_path_raw);
      for(const auto& pt : short_path) 
        voro_path_shortcut.emplace_back(coordToWorldNoOffsetX(pt.x()), coordToWorldNoOffsetY(pt.y()));
      voro_paths_shortcut_.emplace_back(voro_path_shortcut);

      if (path_nodes_sparse.size() <= 4) {
          // 不足5个点，直接不再继续规划
          ROS_WARN("Break because current path has less than 5 path nodes.");
          break;
      }
      double current_path_length = pathLength(short_path);
      int short_times;
      if (min_path_length * resolution_ < 20.0 ) short_times = 4;
      else if (20.0 <= min_path_length * resolution_  && min_path_length * resolution_  < 80.0) short_times = 3;
      else if (80.0 <= min_path_length * resolution_ ) short_times = 2.2;

      if (current_path_length > short_times * min_path_length)
      {
          ROS_WARN_STREAM("Break because current path is more than" << std::to_string(short_times) << "times longer of min_path_length.");
          break;
      }
      min_path_length = std::min(min_path_length, current_path_length);

      path_nodes_sparse = std::vector<IntPoint>(path_nodes_sparse.begin() + 2, path_nodes_sparse.end() - 2);
      int K_nearest = 5;
      Eigen::Vector2d check_point_last;
      check_point_last = short_path[1];
      for (size_t i = 1; i + 1 < short_path.size(); ++i) {
          K_nearest = 5;
          // 距离检查
          if(i != 1) 
            if ((short_path[i] - check_point_last).norm() < 5.0) 
              continue;
          std::vector<IntPoint> k_nearest_points = findKNearestPoints
                                (path_nodes_sparse, IntPoint(short_path[i].x(), short_path[i].y()), K_nearest); 
          K_nearest = k_nearest_points.size();
          
          // 标记最近未被移除的path_nodes_sparse
          for(int j = 0; j < K_nearest; ++j)
          {
            auto it_neighbor = gvg_->findNodeInGraphs(k_nearest_points[j]);
            if (it_neighbor && it_neighbor->type == GraphNode::Strong && !it_neighbor->RemovedByPVS)
            {
              it_neighbor->RemovedByPVS = true;
              // std::cout << "Marking node (" << it_neighbor->pos.x << ", " << it_neighbor->pos.y 
              //           << ") as removed by shortcut point: " << short_path[i].transpose() << std::endl;
              break;
            }
          }
          check_point_last = short_path[i];
      }
      int debug = 0;
    }
    else
    {
      std::cout << "No path found!" << std::endl;
      break;
      gvg_updated_ = false;
    }
  }
  if (!start_on_graph) removeNodeAndConnections(it_start);
  if (!end_on_graph) removeNodeAndConnections(it_goal);  
  if (!(start_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndRepairStrongConnection(start_tmp_node_ptr);
  if (!(end_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndRepairStrongConnection(end_tmp_node_ptr);
  // 遍历整个graph，将标记为RemovedByXX的恢复
  for (auto& graph : gvg_->getGraphs())
  {
    for(auto& node_pair : graph)
    {
      node_pair.second->RemovedByPVS = false;
    }
  }
  std::cout << "Path search finished, found " << path_found_num << " paths." << std::endl;
  // if (path_found_num == 0)
  // {
  //   std::cout << "No path found after 10 iterations." << std::endl;
  //   gvg_updated_ = false;
  //   return false;
  // }
  // else // 选择最优的一条
  // {
  //   std::vector<std::vector<Eigen::Vector3d>> paths_for_publish;
  //   std::vector<Eigen::Vector3d> path_for_publish;
  //   std::vector<double> path_lengths;
  //   std::vector<double> path_angle_diffs;
  //   std::vector<double> path_angle_penalties;
  //   std::vector<double> path_sim_to_last;
  //   std::vector<std::vector<Eigen::Vector2d>> disected_paths = discretizePaths(voro_paths_raw);
  //   int idx = -1;
  //   for (const auto& path : voro_paths_raw)
  //   {
  //     ++idx;
  //     path_for_publish.clear();
  //     for (const auto& pt : path)
  //     {   
  //       path_for_publish.emplace_back(Eigen::Vector3d(
  //                                      (double)coordToWorldNoOffsetX(pt.x()), 
  //                                      (double)coordToWorldNoOffsetX(pt.y()), 
  //                                      0.1));
  //     }

  //     paths_for_publish.emplace_back(path_for_publish);
  //     path_lengths.push_back(pathLength(path));      
  //     double path_angle = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
  //     double angle_diff = path_angle - start_yaw;
  //     // 归一化到 [-pi, pi]
  //     while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  //     while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
  //     path_angle_diffs.emplace_back(std::abs(angle_diff));
  //     path_angle_penalties.emplace_back(std::abs(1.0 - std::cos(path_angle_diffs.back())));
  //     if (last_best_path_.empty()) 
  //     {
  //       path_sim_to_last.push_back(0.0); // 如果没有上一个最优路径，则相似度为0
  //       continue;
  //     }
  //     // 计算与上一个最优路径的相似度
  //     double sim = 0.0;
  //     int check_num = std::min(std::min(disected_paths[idx].size(), last_best_path_.size()), (size_t)(10.0/resolution_));
  //     for (int i = 0; i < check_num; ++i)
  //     {
  //       double dist = (disected_paths[idx][i] - last_best_path_[i]).norm();
  //       sim += dist;
  //     }
  //     path_sim_to_last.push_back(sim / check_num);
  //   }
  //   // visualization_->drawTopoVoronoiPaths(paths_for_publish, 0.2);  
  //   std::cout << "Found " << path_found_num << " paths." << std::endl;

  //   int best_path_idx = 0;
  //   double min_penalty = 1000000.0;
  //   double w_angle = 0.5;    // 权重，控制角度惩罚和长度惩罚的比例
  //   double w_sim_last = 1.0; // 权重，控制与上一个最优路径的相似度的影响
  //   for (int i = 0; i < path_found_num; ++i)
  //   {
  //     double penalty = path_lengths[i] + 
  //                      (path_angle_penalties[i]) * std::min(path_lengths[i], (10.0/resolution_)) * w_angle + 
  //                       path_sim_to_last[i] * w_sim_last;
  //     if (penalty < min_penalty)
  //     {
  //       min_penalty = penalty;
  //       best_path_idx = i;
  //     }
  //   }
  //   last_best_path_ = disected_paths[best_path_idx];
  //   std::cout << "Best path index: " << best_path_idx << ", length: " 
  //             << path_lengths[best_path_idx] << ", angle diff: " 
  //             << path_angle_diffs[best_path_idx] <<  ", sim to last: " 
  //             << path_sim_to_last[best_path_idx] << ", penalty: " << std::endl;
  //   // 发布最优路径
  //   publishPath2(voro_paths_raw[best_path_idx]);

  //   gvg_updated_ = false;
  //   return true;
  // }

  int debug = 0;
}

// 1.把所有强节点都找出来
// 2.对于起点or终点，如果就在图中，则使用该节点；如果不在图中，则新建节点。
// 3.先在强节点中找出离它最近的K个点NearPts。先尝试直线和NearPts相连，找到一个即可退出；
// 4.如果都没有找到，则判断在向着NearPts的方向上，是否有Voronoi点，如果有，则连接到Voronoi点
//   将Voronoi点初始化为GraphNode，并将其和邻居的Strong节点连接起来
// 5.如果正向连接失败，则尝试反向连接到Voronoi点；
// 6.最终，要清除在这里建立的节点和邻居连接
GraphNode::Ptr VoronoiLayer::creatPathNode(
    const Eigen::Vector2i& node_idx,
    int& start_graph_id,
    bool& node_on_graph,
    const vector<IntPoint>& node_strong_cloud,
    int KdNeighborNum,
    bool startNode,
    GraphNode::Ptr node_tmp_ptr)
{
  GraphNode::Ptr node_ptr = std::make_shared<GraphNode>();
  node_ptr->pos = IntPoint(node_idx(0), node_idx(1));
  auto iter_node = gvg_->findNodeInGraphs(node_ptr->pos, start_graph_id);
  if (iter_node) {
      std::cout << "The " << (startNode ? "start" : "end") << " node on the Graph." << std::endl;
      node_ptr = iter_node; // 如果找到了，就使用已有的节点
      node_on_graph = true;
      return node_ptr;
    } 
      std::cout << "The " << (startNode ? "start" : "end") << " node not found in the Graph, creating a new one." << std::endl;  
      std::vector<IntPoint> k_nearest_points = findKNearestPoints(node_strong_cloud, node_ptr->pos, KdNeighborNum); 
      KdNeighborNum = k_nearest_points.size();

      std::vector<IntPoint> raycast_points; //raycast得到的就是四邻域连接
      std::vector<IntPoint> raycast_line_hit_voronoi(KdNeighborNum, IntPoint(-1, -1));
      std::vector<int> neighborGraphIdx(KdNeighborNum, start_graph_id);
      std::vector<bool> neighborVisConnect(KdNeighborNum, false);
      std::vector<std::vector<IntPoint>> raycast_line_to_voronoi(KdNeighborNum);
      Eigen::Vector2i pc;
      Eigen::Vector2i raycastStart(node_idx(0), node_idx(1));
      int valid_neighbors = 0;
      for (int i = 0; i < KdNeighborNum; ++i)
      {
        Eigen::Vector2i raycastEnd(k_nearest_points[i].x, k_nearest_points[i].y);
        // std::cout << "raycastStart: (" << raycastStart.x() << ", " << raycastStart.y() << "), "
        //           << "raycastEnd: (" << raycastEnd.x() << ", " << raycastEnd.y() << ")" << std::endl;
        bool no_collision = lineVisib(raycastStart, raycastEnd, raycast_points, (clearance_low_ - 0.1) / resolution_, pc);
        auto it_neighbor = gvg_->findNodeInGraphs(IntPoint(raycastEnd.x(), raycastEnd.y()), neighborGraphIdx[i]);
        if (no_collision)
        {
          node_ptr->type = GraphNode::Strong; // 设置为强节点
          if (it_neighbor) {
              node_ptr->addNeighbor(it_neighbor, raycast_points);
              std::reverse(raycast_points.begin(), raycast_points.end());
              it_neighbor->addNeighbor(node_ptr, raycast_points);
              neighborVisConnect[i] = true;
              ++ valid_neighbors;
              // break; // x
          }
          else
          {
            ROS_ERROR("Error: Neighbor node not found in the graph.");
          }
          
        }
        else
        {
          //从start或end向最近邻节点
          for(const auto& pt : raycast_points)
          {
            if(gvg_->isVoronoi(pt.x, pt.y))
            {
              raycast_line_hit_voronoi[i] = pt;
              break;
            }
            raycast_line_to_voronoi[i].emplace_back(pt);
          }
        }
      }
      if (valid_neighbors == 0)
      {
        bool connect_find = false; 
        for(int i = 0; i < KdNeighborNum; ++i)
        {
          if(raycast_line_hit_voronoi[i].x == -1 || raycast_line_hit_voronoi[i].y == -1) 
            continue;
          
          {
            // connect_paths是不包含头的
            auto connect_paths = gvg_planner_->expand_voronoi_grid(
                raycast_line_hit_voronoi[i], node_strong_cloud, last_size_x_, last_size_y_);
            int debug = 0;
            if(connect_paths.empty()) continue;
            node_tmp_ptr->pos = raycast_line_hit_voronoi[i];
            node_tmp_ptr->type = GraphNode::Strong; // 设置为强节点
            std::vector<GraphNode::Ptr> two_strong_nodes;
            for (int i = 0; i < connect_paths.size(); ++i)
            {
              auto it_neighbor = gvg_->findNodeInGraphs(connect_paths[i].back(), neighborGraphIdx[i]);
              if (!it_neighbor) continue;         // 其实如果出现这个，是应该要报错的
              connect_paths[i].pop_back();        // 去掉最后一个点，因为是连接到Voronoi点的
              node_tmp_ptr->addNeighbor(it_neighbor, connect_paths[i]);
              std::reverse(connect_paths[i].begin(), connect_paths[i].end());
              it_neighbor->addNeighbor(node_tmp_ptr, connect_paths[i]);
              two_strong_nodes.emplace_back(it_neighbor);
            }
            if (two_strong_nodes.size() == 2)
            {
              auto node_a = two_strong_nodes[0];
              auto node_b = two_strong_nodes[1];

              // 从node_a的邻居中移除node_b
              for (size_t i = 0; i < node_a->neighbors.size(); ++i) {
                  auto nb_ptr = node_a->neighbors[i].lock();
                  if (nb_ptr && nb_ptr->pos == node_b->pos) {
                      node_a->neighbors.erase(node_a->neighbors.begin() + i);
                      node_a->neighbor_paths.erase(node_a->neighbor_paths.begin() + i);
                      break;
                  }
              }
              // 从node_b的邻居中移除node_a
              for (size_t i = 0; i < node_b->neighbors.size(); ++i) {
                  auto nb_ptr = node_b->neighbors[i].lock();
                  if (nb_ptr && nb_ptr->pos == node_a->pos) {
                      node_b->neighbors.erase(node_b->neighbors.begin() + i);
                      node_b->neighbor_paths.erase(node_b->neighbor_paths.begin() + i);
                      break;
                  }
              }
            }
            std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
                  " node, using the A* to search node. 正向" << std::endl;
            node_ptr->type = GraphNode::Strong; // 设置为强节点
            node_ptr->addNeighbor(node_tmp_ptr, raycast_line_to_voronoi[i]);
            std::reverse(raycast_line_to_voronoi[i].begin(), raycast_line_to_voronoi[i].end());
            node_tmp_ptr->addNeighbor(node_ptr, raycast_line_to_voronoi[i]);

            start_graph_id = neighborGraphIdx[i];            
            return node_ptr;
            // 我是不是也不需要把这两个图里面的Strong节点给断开啊？
          }

          IntPoint neighbor_pos(k_nearest_points[i].x, k_nearest_points[i].y);
          auto connect_path = gvg_planner_->AstarOnVoronoi(raycast_line_hit_voronoi[i], 
                  neighbor_pos, last_size_x_, last_size_y_, neighborGraphIdx[i]);
          if(connect_path.empty()) continue;
          connect_find = true;
          raycast_line_to_voronoi[i].insert(raycast_line_to_voronoi[i].end(), connect_path.begin(), connect_path.end());
          node_ptr->type = GraphNode::Strong; // 设置为强节点
          auto it_neighbor = gvg_->findNodeInGraphs(connect_path.back(), neighborGraphIdx[i]);
          if (!it_neighbor) continue;         // 其实如果出现这个，是应该要报错的
          node_ptr->addNeighbor(it_neighbor, raycast_line_to_voronoi[i]);
          // TODO：这里可能会出错。如果这里直接连接上的是其他graph的节点，直接退出了会有问题。
          std::reverse(raycast_line_to_voronoi[i].begin(), raycast_line_to_voronoi[i].end());
          it_neighbor->addNeighbor(node_ptr, raycast_line_to_voronoi[i]); 
          std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
                  " node, using the A* to search node. 正向" << std::endl;
          start_graph_id = neighborGraphIdx[i];
          return node_ptr;
        }
        // 沿着raycast_line_to_voronoi[i]的反方向去raycast，直到碰到第一个voronoi点，然后从这个点搜索
        std::vector<IntPoint> raycast_points; //raycast得到的就是四邻域连接
        Eigen::Vector2i pc;
        Eigen::Vector2i raycastStart(node_idx(0), node_idx(1));

        for (int i = 0; i < KdNeighborNum; ++i)
        {
            // 反向方向：从最近邻指向当前点
            IntPoint neighbor_pos(k_nearest_points[i].x, k_nearest_points[i].y);
            Eigen::Vector2f dir = Eigen::Vector2f(node_idx(0) - neighbor_pos.x,
                                   node_idx(1) - neighbor_pos.y);
            if (dir.norm() == 0) continue;
            dir = dir.normalized();
            Eigen::Vector2i raycastEnd = raycastStart + Eigen::Vector2i(1000 * dir.x() , 1000 * dir.y()); // 远离当前点的方向
            // raycast_points里面包含了维诺图上的那个点
            bool connect_to_voronoi = lineVisib2(raycastStart, raycastEnd, raycast_points, 1.0, pc);
            if (!connect_to_voronoi) continue;

            {
              // connect_paths是不包含头的
              auto connect_paths = gvg_planner_->expand_voronoi_grid(
                  raycast_points.back(), node_strong_cloud, last_size_x_, last_size_y_);
              int debug = 0;
              if(connect_paths.empty()) continue;
              node_tmp_ptr->pos = raycast_points.back();
              raycast_points.pop_back(); // 去掉最后一个点，因为是连接到Voronoi点的
              node_tmp_ptr->type = GraphNode::Strong; // 设置为强节点
              std::vector<GraphNode::Ptr> two_strong_nodes;
              for (int i = 0; i < connect_paths.size(); ++i)
              {
                auto it_neighbor = gvg_->findNodeInGraphs(connect_paths[i].back(), neighborGraphIdx[i]);
                if (!it_neighbor) continue;         // 其实如果出现这个，是应该要报错的
                connect_paths[i].pop_back();        // 去掉最后一个点，因为是连接到Voronoi点的
                node_tmp_ptr->addNeighbor(it_neighbor, connect_paths[i]);
                std::reverse(connect_paths[i].begin(), connect_paths[i].end());
                it_neighbor->addNeighbor(node_tmp_ptr, connect_paths[i]);
                two_strong_nodes.emplace_back(it_neighbor);
              }
              if (two_strong_nodes.size() == 2)
              {
                auto node_a = two_strong_nodes[0];
                auto node_b = two_strong_nodes[1];

                // 从node_a的邻居中移除node_b
                for (size_t i = 0; i < node_a->neighbors.size(); ++i) {
                    auto nb_ptr = node_a->neighbors[i].lock();
                    if (nb_ptr && nb_ptr->pos == node_b->pos) {
                        node_a->neighbors.erase(node_a->neighbors.begin() + i);
                        node_a->neighbor_paths.erase(node_a->neighbor_paths.begin() + i);
                        break;
                    }
                }
                // 从node_b的邻居中移除node_a
                for (size_t i = 0; i < node_b->neighbors.size(); ++i) {
                    auto nb_ptr = node_b->neighbors[i].lock();
                    if (nb_ptr && nb_ptr->pos == node_a->pos) {
                        node_b->neighbors.erase(node_b->neighbors.begin() + i);
                        node_b->neighbor_paths.erase(node_b->neighbor_paths.begin() + i);
                        break;
                    }
                }
              }
              std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
                    " node, using the A* to search node. 反向" << std::endl;
              node_ptr->type = GraphNode::Strong; // 设置为强节点
              node_ptr->addNeighbor(node_tmp_ptr, raycast_points);
              std::reverse(raycast_points.begin(), raycast_points.end());
              node_tmp_ptr->addNeighbor(node_ptr, raycast_points);

              start_graph_id = neighborGraphIdx[i];            
              return node_ptr;
              // 我是不是也不需要把这两个图里面的Strong节点给断开啊？
            }

            auto connect_path = gvg_planner_->AstarOnVoronoi(raycast_points.back(), 
                    neighbor_pos, last_size_x_, last_size_y_, neighborGraphIdx[i]);
            if(connect_path.empty()) continue;
            connect_find = true;
            raycast_points.insert(raycast_points.end(), connect_path.begin(), connect_path.end());
            node_ptr->type = GraphNode::Strong; // 设置为强节点
            auto it_neighbor = gvg_->findNodeInGraphs(connect_path.back(), neighborGraphIdx[i]);
            if (!it_neighbor) continue;         // 其实如果出现这个，是应该要报错的
            node_ptr->addNeighbor(it_neighbor, raycast_points);
            // TODO: 或许也不用反向添加，因为一定是从start_node向外扩展，或向goal_node扩展
            std::reverse(raycast_points.begin(), raycast_points.end());
            it_neighbor->addNeighbor(node_ptr, raycast_points);
            std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
                      " node, using the A* to search node. 反向" << std::endl;        
            start_graph_id = neighborGraphIdx[i];
            return node_ptr;
        }


        std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
                  " node, using the closet node." << std::endl;
        // 如果没有有效的邻居，则使用最近的节点
        IntPoint closest_node = IntPoint(k_nearest_points[0].x, k_nearest_points[0].y);
        auto iter_node = gvg_->findNodeInGraphs(closest_node, start_graph_id);
        if (iter_node) {
            std::cout << "Closest node found: (" << closest_node.x << ", " << closest_node.y << ")" << std::endl;
            node_ptr = iter_node; 
        }
      }
      else
      {
        // 统计neighborGraphIdx中出现次数最多的id
        std::unordered_map<int, int> id_count;
        for (int i = 0; i < neighborGraphIdx.size(); ++i)
        {
          if(neighborVisConnect[i]) ++id_count[neighborGraphIdx[i]];
        }
        int max_count = 0;
        int most_id = -1;
        for (const auto& kv : id_count) {
            if (kv.second > max_count) {
                max_count = kv.second;
                most_id = kv.first;
            }
        }
        start_graph_id = most_id;
      }
  return node_ptr;
}

void VoronoiLayer::removeNodeAndConnections(gvg::GraphNode::Ptr node) {
    if (!node) return;
    IntPoint node_pos = node->pos;
    for (auto& nb : node->neighbors) {
        auto nb_ptr = nb.lock();
        if (!nb_ptr) continue;
        for (size_t i = nb_ptr->neighbors.size(); i-- > 0;) {
            auto back_ptr = nb_ptr->neighbors[i].lock();
            if (back_ptr && back_ptr->pos == node_pos) {
                nb_ptr->neighbors.erase(nb_ptr->neighbors.begin() + i);
                nb_ptr->neighbor_paths.erase(nb_ptr->neighbor_paths.begin() + i);
                // std::cout << "!!!!!!!!!!!!!!" << std::endl;
            }
        }
    }
    gvg_->removeNodeFromGraph(node->pos);
    // std::cout << "---------------" << std::endl;
}

void VoronoiLayer::removeNodeAndRepairStrongConnection(gvg::GraphNode::Ptr node_tmp_ptr) {
    if (!node_tmp_ptr) return;
    if (node_tmp_ptr->neighbors.size() != 2) {
        // 不是两个邻居，不修复
        removeNodeAndConnections(node_tmp_ptr);
        return;
    }

    // 获取两个邻居
    auto nb_ptr1 = node_tmp_ptr->neighbors[0].lock();
    auto nb_ptr2 = node_tmp_ptr->neighbors[1].lock();
    if (!nb_ptr1 || !nb_ptr2) {
        removeNodeAndConnections(node_tmp_ptr);
        return;
    }

    // 取出两条路径
    auto path1 = node_tmp_ptr->neighbor_paths[0].path;
    auto path2 = node_tmp_ptr->neighbor_paths[1].path;

    // 构造完整路径：path1 + node_tmp_ptr->pos + 反转path2
    std::vector<IntPoint> full_path = path1;
    full_path.push_back(node_tmp_ptr->pos);
    std::vector<IntPoint> path2_rev = path2;
    std::reverse(path2_rev.begin(), path2_rev.end());
    full_path.insert(full_path.end(), path2_rev.begin(), path2_rev.end());

    // 先删除node_tmp_ptr和邻居的连接
    removeNodeAndConnections(node_tmp_ptr);

    // 互相添加邻居关系
    nb_ptr1->addNeighbor(nb_ptr2, full_path);
    std::vector<IntPoint> full_path_rev = full_path;
    std::reverse(full_path_rev.begin(), full_path_rev.end());
    nb_ptr2->addNeighbor(nb_ptr1, full_path_rev);
}


void VoronoiLayer::publishPath(const std::vector<IntPoint>& path_nodes)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& node : path_nodes)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        // 将栅格坐标转换为世界坐标
        pose.pose.position.x = coordToWorldX(node.x);
        pose.pose.position.y = coordToWorldY(node.y);
        pose.pose.position.z = 0.1;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.emplace_back(pose);
    }

    path_pub_.publish(path_msg);
}

void VoronoiLayer::publishPath2(const std::vector<Eigen::Vector2d>& path_nodes)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& node : path_nodes)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        // 将栅格坐标转换为世界坐标
        pose.pose.position.x = coordToWorldNoOffsetX(node.x());
        pose.pose.position.y = coordToWorldNoOffsetY(node.y());
        pose.pose.position.z = 0.1;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.emplace_back(pose);
    }

    path_pub2_.publish(path_msg);
}

void VoronoiLayer::publishTestPath(const vector<Eigen::Vector2d>& path, const int& pub_num)
{
  nav_msgs::Path path_tmp;
  path_tmp.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose_stamped;  
  path_tmp.header.frame_id = "world";    
  pose_stamped.header.frame_id = "world"; 
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose.orientation.w = 1.0;   
  pose_stamped.pose.position.z = 0.1;                        
  for (const auto& pt : path) 
  {
      pose_stamped.pose.position.x = coordToWorldX(pt.x() - 0.5);
      pose_stamped.pose.position.y = coordToWorldY(pt.y() - 0.5);
      path_tmp.poses.emplace_back(pose_stamped);
  }
  if(pub_num == 1)
    path_pub_test_1_.publish(path_tmp);
  else if(pub_num == 2)
    path_pub_test_2_.publish(path_tmp);
  else 
    path_pub_test_3_.publish(path_tmp);
  ros::Duration(0.0001).sleep();
}
void VoronoiLayer::publishTestPath(const vector<Eigen::Vector2i>& path, const int& pub_num)
{
  nav_msgs::Path path_tmp;
  path_tmp.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose_stamped;  
  path_tmp.header.frame_id = "world";    
  pose_stamped.header.frame_id = "world"; 
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose.orientation.w = 1.0;   
  pose_stamped.pose.position.z = 0.1;                        
  for (const auto& pt : path) 
  {
      pose_stamped.pose.position.x = coordToWorldX(pt.x());
      pose_stamped.pose.position.y = coordToWorldY(pt.y());
      path_tmp.poses.emplace_back(pose_stamped);
  }
  if(pub_num == 1)
    path_pub_test_1_.publish(path_tmp);
  else if(pub_num == 2)
    path_pub_test_2_.publish(path_tmp);
  else 
    path_pub_test_3_.publish(path_tmp);
  ros::Duration(0.0001).sleep();
}

void VoronoiLayer::drawESDFToMat(cv::Mat& esdf_img)
{
    esdf_img = cv::Mat::zeros(last_size_y_, last_size_x_, CV_8UC3);
    const float min_dist = 0.0;
    const float max_dist = 3.2;
    const float scale = 255.0 / 3.2;
    for (int x = 0; x < last_size_x_; ++x)
    {    
      for (int y = 0; y < last_size_y_; ++y)
      {
            float dist = voronoi_.getDistance(x, y) * resolution_;
            dist = std::min(std::max(dist, min_dist), max_dist); // 限制在[min_dist, max_dist]
            // 归一化到0-255
            char gray = static_cast<char>(255.0 * (dist - min_dist) / (max_dist - min_dist));
            esdf_img.at<cv::Vec3b>(last_size_y_ - y - 1, x) = cv::Vec3b(gray, gray, gray);
            if(dist >= 3.0 && dist <= 3.0 + 1e-2)
            {
              esdf_img.at<cv::Vec3b>(last_size_y_ - y - 1, x) = cv::Vec3b(gray, 0, 0);
            }
            else if (dist > 3.0 + 1e-2  && dist <= 3.1)
            {
              esdf_img.at<cv::Vec3b>(last_size_y_ - y - 1, x) = cv::Vec3b(0, gray, 0);
            }
            // else if (dist > 3.1 && dist <= 3.141)
            // {
            //   esdf_img.at<cv::Vec3b>(last_size_y_ - y, x) = cv::Vec3b(0, 0, gray);
            // }
          }
    }
    int debug = 0;
 }

 // 注意，这里的单位都是像素单位
 // 如果是从网格的索引要转换到连续空间，有0.5的偏置
 // 网格索引->连续空间，加0.5
 // 连续空间->网格索引，然后取floor
 vector<Eigen::Vector2d> VoronoiLayer::shortcutPath(const std::vector<IntPoint>& path, int path_id, int iter_num)
 {
  vector<Eigen::Vector2d> short_path, last_path, process_path;
  for (const auto& pt : path) {
    short_path.emplace_back(pt.x + 0.5, pt.y + 0.5); 
  }

  // if(1) publishTestPath(short_path, 1);
  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;
    if (k == 0)
      process_path = short_path;
    else
      process_path = discretizePath(short_path);
    
    if (last_path.size() < 2) {
      return last_path;
    }

    /* visibility path shortening */
    Eigen::Vector2d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.emplace_back(process_path.front());
    for (int i = 1; i < process_path.size(); ++i) {
      // std::cout << "---------" << std::endl;
      if (lineVisib(short_path.back(), process_path[i], (clearance_low_ - 0.05) / resolution_, colli_pt, path_id, -1)) continue;
      // std::cout << "---------" << std::endl;
      // 如果此时有一个点和short_path最后一个点碰撞了，则把这个碰撞点外推
      evaluateEDTWithGrad(colli_pt, dist, grad);
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (process_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        colli_pt = colli_pt + 1.3 * push_dir; // 改了这个地方，多推一点
        // --i;
      }
      // if(1) publishTestPath(std::vector<Eigen::Vector2d>{short_path.back(), process_path[i]}, 2);      
      short_path.emplace_back(colli_pt);
      // if(1) publishTestPath(short_path, 1);
      // std::cout << "short_path: " << short_path.back().x() << ", " << short_path.back().y() << std::endl;
      // int  debug = 0;
    }
    short_path.emplace_back(process_path.back());
    // if(1) publishTestPath(short_path, 2);
    // int debug = 0;
    /* break if no shortcut */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if (len2 > len1) {
      // ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k +
      // 1);
      short_path = last_path;
      break;
    }
  }
  return short_path;
  // if(1) publishTestPath(short_path, 2);
  // short_paths_[path_id] = short_path;
 }

double VoronoiLayer::pathLength(const vector<Eigen::Vector2d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

bool VoronoiLayer::lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double thresh,
                             Eigen::Vector2d& pc, int caster_id, int skip_mode) {
    Eigen::Vector3d ray_pt;
    Eigen::Vector2i pt_id;
    double dist;

    thresh = std::max(thresh, 1e-3);
    Eigen::Vector2d dir = p2 - p1;
    // double dis_pt = dir.norm();
    // std::cout << "dis_pt: " << dis_pt << std::endl;
    // if (std::abs(dis_pt - 132.85) < 1e-3) {
    //   int debug = 0;
    // }
    Eigen::Vector3d p1_3d(p1.x(), p1.y(), 0.0);
    Eigen::Vector3d p2_3d(p2.x(), p2.y(), 0.0);
    RayCaster raycaster;      
    raycaster.setInput(p1_3d, p2_3d);

    // while会跳过最后一个点
    while (raycaster.step(ray_pt)) {
      pt_id(0) = ray_pt(0) + offset_(0);
      pt_id(1) = ray_pt(1) + offset_(1);
      dist = voronoi_.getDistanceNoBoundCheck(pt_id(0), pt_id(1));
      // std::cout << "pt: " << pt_id.transpose() << ", dist: " << dist << ", thre: " <<thresh << std::endl;
      if (dist <= (thresh)) {
        pc(0) = pt_id(0) + 0.5;
        pc(1) = pt_id(1) + 0.5;
        return false;
      }
    }
    return true;
  }

bool VoronoiLayer::lineVisib(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2, std::vector<IntPoint>& rayline_list,
                             double thresh, Eigen::Vector2i& pc, int caster_id, int skip_mode) {
    rayline_list.clear();
    Eigen::Vector3d ray_pt;
    Eigen::Vector2i pt_id;
    double dist;
    thresh = std::max(thresh, 1e-3);
    Eigen::Vector3d p1_3d(p1.x() + 0.5, p1.y() + 0.5, 0.0);
    Eigen::Vector3d p2_3d(p2.x() + 0.5, p2.y() + 0.5, 0.0);
    RayCaster raycaster;      
    raycaster.setInput(p1_3d, p2_3d);

    // while会跳过最后一个点
    int iter = 0;
    while (raycaster.step(ray_pt)) {
      pt_id(0) = ray_pt(0) + 0.5;
      pt_id(1) = ray_pt(1) + 0.5;
      dist = voronoi_.getDistanceNoBoundCheck(pt_id(0), pt_id(1));
      // std::cout << "dist: " << dist << std::endl;
      if (dist <= (thresh)) {
        pc(0) = pt_id(0);
        pc(1) = pt_id(1);
        return false;
      }
      else {
        if(iter > 0) //跳过第一个点，因为这个是起点、我不需要
          rayline_list.emplace_back(pt_id(0), pt_id(1));
      }
      ++iter;
    }
    // rayline_list.emplace_back(p2(0), p2(1)); // 添加终点
    return true;
  }

bool VoronoiLayer::lineVisib2(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2, std::vector<IntPoint>& rayline_list,
                             double thresh, Eigen::Vector2i& pc, int caster_id, int skip_mode) {
    rayline_list.clear();
    Eigen::Vector3d ray_pt;
    Eigen::Vector2i pt_id;
    double dist;
    thresh = std::max(thresh, 1e-3);
    Eigen::Vector3d p1_3d(p1.x() + 0.5, p1.y() + 0.5, 0.0);
    Eigen::Vector3d p2_3d(p2.x() + 0.5, p2.y() + 0.5, 0.0);
    RayCaster raycaster;      
    raycaster.setInput(p1_3d, p2_3d);

    // while会跳过最后一个点
    int iter = 0;
    while (raycaster.step(ray_pt)) {
      pt_id(0) = ray_pt(0) + 0.5;
      pt_id(1) = ray_pt(1) + 0.5;
      if (pt_id(0) < 0 || pt_id(0) >= last_size_x_ || 
          pt_id(1) < 0 || pt_id(1) >= last_size_y_) {
        return false;
      }

      if (gvg_->isVoronoi(pt_id(0), pt_id(1))) {
        rayline_list.emplace_back(pt_id(0), pt_id(1));
        return true;
      }

      dist = voronoi_.getDistance(pt_id(0), pt_id(1));
      // std::cout << "dist: " << dist << std::endl;
      if (dist <= (thresh)) {
        pc(0) = pt_id(0);
        pc(1) = pt_id(1);
        return false;
      }
      else {
        if(iter > 0) //跳过第一个点，因为这个是起点、我不需要
          rayline_list.emplace_back(pt_id(0), pt_id(1));
      }
      ++iter;
    }
    // rayline_list.emplace_back(p2(0), p2(1)); // 添加终点
    return false;
  }

  void VoronoiLayer::evaluateEDTWithGrad(const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d& grad)
  {
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    double dists[2][2];    
    getSurroundPts(pos, sur_pts, dists, diff);
    interpolateBilinear(dists, diff, dist, grad);
  }

  void VoronoiLayer::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], double dists[2][2],
                              Eigen::Vector2d& diff) {
    if (!isInMap(pos)) {
      // cout << "pos invalid for interpolation." << endl;
    }
    /* interpolation position */
    Eigen::Vector2d pos_m = pos - offset_;
    Eigen::Vector2i idx(std::floor(pos_m.x()), std::floor(pos_m.y()));
    Eigen::Vector2d idx_pos(idx.x() + 0.5, idx.y() + 0.5);
    diff = (pos - idx_pos);
    for (int x = 0; x < 2; ++x) {
      for (int y = 0; y < 2; ++y) {
          Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
          Eigen::Vector2d current_pos(current_idx.x() + 0.5, current_idx.y() + 0.5);
          pts[x][y] = current_pos;
          boundIndex(current_idx);
          dists[x][y] = voronoi_.getDistance(current_idx.x(), current_idx.y());
      }
    }
  }
void VoronoiLayer::interpolateBilinear(
    double values[2][2],
    const Eigen::Vector2d& diff,
    double& value,
    Eigen::Vector2d& grad)
{
    // values: [x][y]
    // diff: [dx, dy]，dx/dy ∈ [0,1)
    double x = diff(0), y = diff(1);
    // 双线性插值
    value = (1 - x) * (1 - y) * values[0][0]
          + x * (1 - y) * values[1][0]
          + (1 - x) * y * values[0][1]
          + x * y * values[1][1];
    // 对x求导
    grad(0) = -(1 - y) * values[0][0]
              + (1 - y) * values[1][0]
              - y * values[0][1]
              + y * values[1][1];
    // 对y求导
    grad(1) = -(1 - x) * values[0][0]
              - x * values[1][0]
              + (1 - x) * values[0][1]
              + x * values[1][1];
}
vector<Eigen::Vector2d> VoronoiLayer::discretizeLine(Eigen::Vector2d p1, Eigen::Vector2d p2) {
  Eigen::Vector2d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len);

  vector<Eigen::Vector2d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  for (int i = 0; i <= seg_num; ++i) line_pts.emplace_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}
vector<Eigen::Vector2d> VoronoiLayer::discretizePath(const vector<Eigen::Vector2d>& path) {
  vector<Eigen::Vector2d> dis_path, segment;

  if (path.size() < 2) {
    ROS_ERROR("what path? ");
    return dis_path;
  }

  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine(path[i], path[i + 1]);

    if (segment.size() < 1) continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    if (i != path.size() - 2) dis_path.pop_back();
  }
  return dis_path;
}

vector<vector<Eigen::Vector2d>> VoronoiLayer::discretizePaths(vector<vector<Eigen::Vector2d>>& path) {
  vector<vector<Eigen::Vector2d>> dis_paths;
  vector<Eigen::Vector2d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.emplace_back(dis_path);
  }

  return dis_paths;
}


void VoronoiLayer::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;
  // std::cout << "current yaw: " << Eigen::Quaterniond(odom_orient_).toRotationMatrix().eulerAngles(0, 1, 2)[2] << std::endl;

  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
  odom_yaw_         = atan2(rot_x(1), rot_x(0));
  has_odom_ = true;
}

void VoronoiLayer::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!has_odom_) {
    // ROS_WARN("No odometry data received yet, cannot set goal.");
    return;
  }

  Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  lastest_goal_ = goal;
  has_goal_ = true;
  this->plan(odom_pos_, lastest_goal_, odom_yaw_);
}

void VoronoiLayer::planTimerCallback(const ros::TimerEvent& event)
{
  if (!has_odom_) {
    // ROS_WARN("No odometry data received yet, cannot plan.");
    return;
  }
  if (!has_goal_){
    return;
  }
  // 判断当前位置和goal的直线之间是否小于2m且都是free的
  // 如果是，则将has_goal_设置为false，表示已经到达目标点，停止持续的规划
  double dist = (odom_pos_ - lastest_goal_).head<2>().norm(); // 只考虑x,y
  if (dist < 3.0) {
    // 判断直线上所有点是否都是free
    Eigen::Vector2d start(odom_pos_.x(), odom_pos_.y());
    Eigen::Vector2d goal(lastest_goal_.x(), lastest_goal_.y());
    Eigen::Vector2i start_coord, goal_coord;
    start_coord.x() = WorldToMapX(start.x());
    start_coord.y() = WorldToMapY(start.y());
    goal_coord.x() = WorldToMapX(goal.x());
    goal_coord.y() = WorldToMapY(goal.y());

    Eigen::Vector2d start_pt, goal_pt;
    start_pt.x() = start_coord.x();
    start_pt.y() = start_coord.y();
    goal_pt.x() = goal_coord.x();
    goal_pt.y() = goal_coord.y();

    std::vector<Eigen::Vector2d> line_pts = discretizeLine(start_pt, goal_pt);
    bool all_free = true;
    for (const auto& pt : line_pts) {
      if (voronoi_.isOccupied(pt.x(), pt.y())) {
        all_free = false;
        break;
      }
    }
    if (all_free) {
      has_goal_ = false;
      ROS_INFO("Arrived at goal, stop planning.");
      return;
    }
  }

  this->plan(odom_pos_, lastest_goal_, odom_yaw_);
}

void VoronoiLayer::voronoiUpdateTimerCallback(const ros::TimerEvent& event)
{
  boost::unique_lock<boost::mutex> lock(mutex_);

  std::vector<IntPoint> new_free_cells, new_occupied_cells;
  for (unsigned int i = 0; i < last_size_x_; ++i)
  {  
    for (unsigned int j = 0; j < last_size_y_ ; ++j)
    {
      int index = j * last_size_x_ + i;
      if (voronoi_.isOccupied(i, j))
      {
        new_occupied_cells.emplace_back(i, j);
      }

      if (!voronoi_.isOccupied(i, j))
      // if (!voronoi_.isOccupied(i, j) && map_msg->getCost(i, j) >= 128)
      {
        new_free_cells.emplace_back(i, j);
      }
    }
  }

  voronoi_.initializeEmpty(last_size_x_, last_size_y_);

  for (size_t i = 0; i < new_free_cells.size(); ++i)
  {
    voronoi_.clearCell(new_free_cells[i].x, new_free_cells[i].y);
  }

  for (size_t i = 0; i < new_occupied_cells.size(); ++i)
  {
    voronoi_.occupyCell(new_occupied_cells[i].x, new_occupied_cells[i].y);
  }
  voronoi_.update();
  voronoi_.prune();
}

}  // namespace costmap_2d
