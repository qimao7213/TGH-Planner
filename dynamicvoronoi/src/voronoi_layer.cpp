
#include "dvr/voronoi_layer.h"
// #define VERBOSE
// #define SavePathData
bool debug_flag_ = false;
using namespace gvg;
namespace DynaVoro
{
VoronoiLayer::VoronoiLayer(ros::NodeHandle& nh)
{
  double resolutiond, clearance_lowd, clearance_highd;
  nh.param("voronoi_layer/voro_map_resolution", resolutiond, 0.1);
  nh.param("voronoi_layer/obs_clearance", clearance_lowd, 0.3);
  nh.param("voronoi_layer/obs_clearance_high", clearance_highd, 8.0);
  resolution_ = static_cast<float>(resolutiond);
  resolutuon_inv_ = 1.0f / resolution_;
  clearance_low_ = static_cast<float>(clearance_lowd);   //小于这个值的维诺点不要
  clearance_high_ = static_cast<float>(clearance_highd); //大于这个值的维诺点不要,包络线
  ROS_WARN_STREAM("VoronoiLayer: resolution: " << resolution_ << ", clearance_low: " << clearance_low_ << ", clearance_high: " << clearance_high_);
  // visualization_.reset(new fast_planner::PlanningVisualization(nh));
  clearance_low_thr_ = clearance_low_ * clearance_low_ / resolution_ / resolution_;
  clearance_high_thr_ = clearance_high_ * clearance_high_ / resolution_ / resolution_;

  gvg_ = std::make_shared<GVG>();
  gvg_->setClearanceThresholdSq(clearance_low_thr_, clearance_high_thr_);
  // gvg_->set_use_EGVG(false);
  gvg_planner_ = std::make_shared<gvg::Planner>();
  gvg_planner_->init(gvg_);

  odometry_sub_ = nh.subscribe<nav_msgs::Odometry>("/car_odom", 1, &VoronoiLayer::odometryCallback, this);
  
  #ifdef SavePathData
  raw_path_sub_ = nh.subscribe<nav_msgs::Path>("/reuse_graph_node/topo_graph_path", 1, &VoronoiLayer::rawPathCallback, this);
  costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D", 1, &VoronoiLayer::costmapCB, this);
  #endif
  voronoi_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi/gridmap", 10);
  voronoi_grid_origin_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi/gridmap_origin", 10);
  voronoi_occupy_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi/gridmap_occupy", 10);
  distance_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voronoi/esdfmap", 10);
  gvg_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/voronoi/gvg_markers", 10);
  path_pub_ = nh.advertise<nav_msgs::Path>("/voronoi/path_topo", 10);
  path_pub2_ = nh.advertise<nav_msgs::Path>("/voronoi/path_short", 10);
  path_pub_test_1_ = nh.advertise<nav_msgs::Path>("/voronoi/path_test1", 10);
  path_pub_test_2_ = nh.advertise<nav_msgs::Path>("/voronoi/path_test2", 10);
  path_pub_test_3_ = nh.advertise<nav_msgs::Path>("/voronoi/path_test3", 10);
  
  egvg_runtime_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/egvg_runtime", 10);

  // rviz_goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &VoronoiLayer::rvizGoalCallback, this);
  // plan_timer_ = nh.createTimer(ros::Duration(0.2), &VoronoiLayer::planTimerCallback, this);
  voronoi_update_timer_ = nh.createTimer(ros::Duration(2), &VoronoiLayer::voronoiUpdateTimerCallback, this);

  #ifdef SavePathData
  std::string save_file;
  std::string default_path = "ChangeToYourPath/";
  nh.param("FilePath", save_file, default_path);
  save_file += "Utils/global_plan_record/Scene_6/plan_data.txt";
  save_file = "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/global_plan_record/Scene6/plan_data.txt";
  path_data_.save_file_stream.open(save_file.c_str(), std::ios::out);
  if(!path_data_.save_file_stream.is_open())
  {
    ROS_ERROR_STREAM("Unable to open file to save traj info! at: " << save_file);
  }
  #endif
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

bool VoronoiLayer::update_by_occupancy_map(const std::vector<char>& occupancy_map, int map_size_x, int map_size_y)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  // std::cout << "map_size_x: " << map_size_x << ", map_size_y: " << map_size_y << std::endl;
  if (last_size_x_ != map_size_x || last_size_y_ != map_size_y)
  {
    voronoi_.initializeEmpty(map_size_x, map_size_y);
    for (unsigned int i = 0; i < map_size_x; ++i) {
      voronoi_.occupyCell(i, 0);           // 上边界
      voronoi_.occupyCell(i, map_size_y - 1); // 下边界
    }
    for (unsigned int j = 0; j < map_size_y; ++j) {
        voronoi_.occupyCell(0, j);           // 左边界
        voronoi_.occupyCell(map_size_x - 1, j); // 右边界
    }
    last_size_x_ = map_size_x;
    last_size_y_ = map_size_y;
    esdf_data_.resize(last_size_x_ * last_size_y_, 1000.0);
  }
  std::vector<IntPoint> new_free_cells, new_occupied_cells;
  for (unsigned int i = 1; i < last_size_x_ - 2; ++i)
  {  
    for (unsigned int j = 1; j < last_size_y_ - 2; ++j)
    {
      int index = j * last_size_x_ + i;
      if (voronoi_.isOccupied(i, j) && occupancy_map[index] != 1)
      {
        new_free_cells.emplace_back(i, j);
      }

      if (!voronoi_.isOccupied(i, j) && occupancy_map[index] == 1)
      {
        new_occupied_cells.emplace_back(i, j);
      }
    }
  }
  if (new_free_cells.empty() && new_occupied_cells.empty())
  {
    publishGVG(gvg_->getGraphs());
    publishVoronoiGrid();
    return false; // 没有变化
  }
  for (size_t i = 0; i < new_free_cells.size(); ++i)
  {
    voronoi_.clearCell(new_free_cells[i].x, new_free_cells[i].y);
  }

  for (size_t i = 0; i < new_occupied_cells.size(); ++i)
  {
    voronoi_.occupyCell(new_occupied_cells[i].x, new_occupied_cells[i].y);
  }

  SimpleTimer timer;
  voronoi_.update();
  voronoi_.prune();

  for (int x = 0; x < last_size_x_; ++x)
  {
    for (int y = 0; y < last_size_y_; ++y)
    {
      int idx = y * last_size_x_ + x;
      esdf_data_[idx] = (voronoi_.getDistance(x, y)) * resolution_;    
    }
  }
  double gvd_runtime, egvg_runtime;
  #ifdef VERBOSE
  gvd_runtime = timer.print("Voronoi Update");    
  #endif
  timer.reset();
  gvg_->createGraph(voronoi_);
  gvg_updated_ = true;
  #ifdef VERBOSE
  egvg_runtime = timer.print("GVG Create"); 
  #endif

  publishRuntime(gvd_runtime, egvg_runtime);
  publishGVG(gvg_->getGraphs());
  publishVoronoiGrid();
  // publishDistanceCloud();
  return true;
}

void VoronoiLayer::publishRuntime(double gvd_runtime, double egvg_runtime)
{
  std_msgs::Float32MultiArray runtime_msg;
  runtime_msg.data.resize(2);
  runtime_msg.data[0] = static_cast<float>(gvd_runtime);
  runtime_msg.data[1] = static_cast<float>(egvg_runtime);
  egvg_runtime_pub_.publish(runtime_msg);
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
    esdf_data_.resize(last_size_x_ * last_size_y_, 1000.0);
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
    publishGVG(gvg_->getGraphs());
    publishVoronoiGrid();
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

  for (int x = 0; x < last_size_x_; ++x)
  {
    for (int y = 0; y < last_size_y_; ++y)
    {
      int idx = y * last_size_x_ + x;
      esdf_data_[idx] = (voronoi_.getDistance(x, y)) * resolution_;    
    }
  }

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
  // cv::Mat voronoiImg2 = voronoiImg.clone();     // 原始的voronoi图
  // cv::Mat voronoiImg3 = voronoiImg.clone();     
  // for (int y = 0; y < size_y; ++y)
  // {
  //   for (int x = 0; x < size_x; ++x)
  //   {
  //     if (voronoi_.isVoronoiWithDisThr(x, y, clearance_low_thr_))
  //     {
  //       voronoiImg2.at<cv::Vec3b>(size_y - y - 1, x)[0] = 255;   // 蓝色通道
  //       voronoiImg2.at<cv::Vec3b>(size_y - y - 1, x)[1] = 255;   // 绿色通道
  //       voronoiImg2.at<cv::Vec3b>(size_y - y - 1, x)[2] = 0;   // 红色通道
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
  //   cv::Vec3b pixel = voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x);
  //   if (pixel == cv::Vec3b(0, 0, 255) || pixel == cv::Vec3b(255, 0, 0)) continue;   // 不画在强弱节点上
  //   if (pixel == cv::Vec3b(255, 255, 255)) continue; // 原本是白色的点不画，不展示已经被冗余去除的点
  //   // if (voronoiImg2.at<cv::Vec3b>(size_y - point.y - 1, point.x) == cv::Vec3b(255, 255, 0)) continue; 
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[0] = 100;   // 蓝色通道
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[1] = 20; // 绿色通道
  //   voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x)[2] = 150;   // 红色通道
  //   int debug = 0;
  // }
  // auto completeCoonction_points = gvg_->getCompleteConnectionPoints();
  // for (const auto& point : completeCoonction_points)
  // {
  //   // cv::circle(voronoiImg, cv::Point(point.x, size_y - point.y - 1), 1, cv::Scalar(255, 0, 255), -1);
  //   cv::Vec3b pixel = voronoiImg.at<cv::Vec3b>(size_y - point.y - 1, point.x);
  //   if (pixel == cv::Vec3b(0, 0, 255) || pixel == cv::Vec3b(255, 0, 0)) continue;
  //   if (pixel == cv::Vec3b(255, 255, 255)) continue; 
  //   // if (voronoiImg2.at<cv::Vec3b>(size_y - point.y - 1, point.x) == cv::Vec3b(255, 255, 0)) continue; 
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
  //       // voronoiMix.at<cv::Vec3b>(size_y - y - 1, x)[2] = 255;   // 红色通道
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
  // std::string save_path = "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/save_map_point/egvg_analysis/";
  // std::string map_name = save_path + "occupancy_map.png";
  // std::string voronoi_name = save_path + "voronoi_map.png";
  // std::string voronoi_origin_name = save_path + "voronoi_map_origin.png";
  // cv::imwrite(voronoi_name, voronoiImg);
  // cv::imwrite(voronoi_origin_name, voronoiImg2);


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
        grid_occupy.data[x + y * last_size_x_] = 128;
      }
      if (voronoi_.isOccupied(x, y))
      {
        grid_occupy.data[x + y * last_size_x_] = -1;
      }
    }
  }
  // timer.print("XXXXXXXXXXXXXXXXXXXXXXXX");
  // voronoi_grid_origin_pub_.publish(grid);
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
                if(node_ptr->neighbor_paths.empty()) continue; // 不发布没有路径的强节点
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            }
            else if (node_ptr->type == GraphNode::Weak)
            {
                if(node_ptr->neighbor_paths.empty()) continue; // 不发布没有路径的强节点
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
      int idx = y * last_size_x_ + x;
      float dist = esdf_data_[ idx ];

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
  std::cout << "------------------- Voro Plan Start -------------------" << std::endl;
  if ((goal - goal_last_).norm() > 5.0)
  {
    last_best_path_.clear();
    goal_last_ = goal;
  }
  voro_paths_raw_world_.clear();
  voro_paths_shortcut_.clear();

  // GraphInfo curr_graph_info;
  // curr_graph_info.init(gvg_);
  // // curr_graph_info.print();
  // bool gvg_same = last_graph_info_ == curr_graph_info;
  // ROS_WARN_STREAM("GVG is same: " << gvg_same );
  // last_graph_info_ = curr_graph_info; // 记录当前图信息

  // start.x() = -16.04;
  // start.y() = -20.11;
  // start.z() = -1.06412;
  // goal.x() = 17.9;
  // goal.y() = 6.6;
  // goal.z() = -0.450835;

  #ifdef SavePathData
  path_data_.reset();
  path_data_.start_x = start.x();
  path_data_.start_y = start.y();
  path_data_.goal_x = goal.x();
  path_data_.goal_y = goal.y();
  #endif

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

  DynaVoro::SimpleTimer timer;
  #ifdef SavePathData
  float start_esdf = voronoi_.getDistanceNoBoundCheck(start_coord.x(), start_coord.y());
  float goal_esdf = voronoi_.getDistanceNoBoundCheck(goal_coord.x(), goal_coord.y());
  if (start_esdf < clearance_low_ / resolution_)
  {
    ROS_WARN_STREAM("The first node is occupied, please check the raw path. start_esdf: " << start_esdf);
    return false;
  }
  if (goal_esdf < clearance_low_ / resolution_)
  {
    ROS_WARN_STREAM("The last node is occupied, please check the raw path. goal_esdf: " << goal_esdf);
    return false;   
  }
  #endif
  // 这里使用SampleAstar来规划路径
  // {
  //   double t_search = 0.0, t_shortcut = 0.0;
  //   timer.reset();
  //   std::vector<IntPoint> astar_path = SampleAstar(IntPoint(WorldToMapX(start.x()), WorldToMapY(start.y())),
  //                                                  IntPoint(WorldToMapX(goal.x()), WorldToMapY(goal.y())));
  //   t_search = timer.print("Astar Search");
  //   double astar_path_length = pathLength2(astar_path) * resolution_;
  //   timer.reset();
  //   std::vector<Eigen::Vector2d> astar_path_smooth = shortcutPath(astar_path, 0, 2);
  //   t_shortcut = timer.print("Astar Shortcut");
  //   double astar_path_smooth_length = pathLength(astar_path_smooth) * resolution_;
  //   publishPath(astar_path);    
  //   publishPath2(astar_path_smooth);
  //   std::cout << "Astar path length: " << astar_path_length << ", smooth length: " << astar_path_smooth_length << std::endl;
  //   int debug = 0;
  //   #ifdef SavePathData
  //   path_data_.iter_num ++;
  //   path_data_.min_path_length = astar_path_smooth_length;
  //   path_data_.min_raw_path_length = astar_path_length;
  //   path_data_.t_search = t_search;
  //   path_data_.t_shortcut = t_shortcut;
  //   path_data_.saveToFile();
  //   #endif
  //   return true;
  // }

    vector<IntPoint> strong_nodes;
    gvg_->getStrongNodes(strong_nodes);
    // ROS_WARN_STREAM("There are " << gvg_->getGraphsSize() << " GVG graphs. Strong nodes: " << strong_nodes.size());
    KdNeighborNum = std::min(KdNeighborNum, static_cast<int>(strong_nodes.size()));
  if (lineVisib(start_pt, goal_pt, (clearance_low_ - 0.05) / resolution_, colli_pt) || KdNeighborNum == 0) {
    publishPath({});
    publishPath2({start_pt, goal_pt});
    ROS_WARN_STREAM("Start and goal are visible, return straight line path.");

    #ifdef SavePathData
    path_data_.reset();
    path_data_.iter_num ++;
    path_data_.min_path_length = (goal_pt - start_pt).norm() * resolution_;
    path_data_.min_raw_path_length = path_data_.min_path_length;
    path_data_.t_search = 0.0;
    path_data_.t_shortcut = 0.0;
    path_data_.saveToFile();
    #endif
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
  int start_graph_id = -1;
  int end_graph_id = -1;
  bool start_on_graph = false;//如果起点or终点就在图上，则说明不用添加新的连接。
  bool end_on_graph = false;
  GraphNode::Ptr start_tmp_node_ptr = std::make_shared<GraphNode>(); //如果起点or终点没有直接和图节点连接，而是和gvd的grid点连接，则需要创建一个临时节点
  GraphNode::Ptr end_tmp_node_ptr = std::make_shared<GraphNode>();
  start_tmp_node_ptr->pos = IntPoint(-1, -1);
  end_tmp_node_ptr->pos = IntPoint(-1, -1);

  auto it_start = creatPathNode(
      start_coord, start_graph_id, start_on_graph, strong_nodes, KdNeighborNum, true, start_tmp_node_ptr);
  // #ifdef VERBOSE
    // timer.print("Plan KD-Tree Start Node");
  // #endif
  timer.reset();
  // if (!start_on_graph) // 如果起点不在图上，则说明起点被初始化为一个新节点，需要添加到图里面？//不需要做这种处理
  // {
  //   gvg_->insertNodeToGraph(it_start, start_graph_id);
  //   strong_nodes.push_back(it_start->pos);
  // }
  // if (!(start_tmp_node_ptr->pos == IntPoint(-1, -1)))  // 如果初始化了一个临时连接起点的节点，则需要将其添加到图中
  // {
  //   gvg_->insertNodeToGraph(start_tmp_node_ptr, start_graph_id);
  //   strong_nodes.push_back(start_tmp_node_ptr->pos);
  // }
   
  auto it_goal = creatPathNode(
      goal_coord, end_graph_id, end_on_graph, strong_nodes, KdNeighborNum, false, end_tmp_node_ptr);
  // #ifdef VERBOSE
    // timer.print("Plan KD-Tree End Node");
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
    if (!(start_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndConnections(start_tmp_node_ptr);
    if (!(end_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndConnections(end_tmp_node_ptr);
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
  double t_search = 0.0, t_shortcut = 0.0;
  #ifdef SavePathData
  path_data_.reset();
  #endif
  for (int iter = 0; iter < path_search_times; ++iter)
  {
    timer.reset();
    if(gvg_planner_->serachPath(it_start, it_goal))
    {
      t_search += timer.print("Plan One A* Search"); 
      std::vector<IntPoint> path_nodes_sparse = gvg_planner_->getFullPathNodes();
      std::vector<IntPoint> path_nodes = gvg_planner_->getFullPath();
      if (path_nodes.size() > 4 * min_path_length && path_nodes.size() > 3 * last_size_x_)
      {
        ROS_WARN_STREAM("Break because current's raw path is more than 5 times longer of min_path_length.");
        break;
      }      
      
      timer.reset();
      vector<Eigen::Vector2d> short_path = shortcutPath(path_nodes, 0, 2);
      t_shortcut += timer.print("Plan Short Path Shortcut");
      #ifdef SavePathData
      {
        path_data_.t_search = t_search;
        path_data_.t_shortcut = t_shortcut;
        double current_path_length = pathLength(short_path) * resolution_;
        double current_raw_path_length = pathLength(path_nodes) * resolution_;
        std::cout << "Current path length: " << current_path_length 
                  << ", raw path length: " << current_raw_path_length << std::endl;
        if (current_path_length < path_data_.min_path_length)
        {
            path_data_.min_path_length = current_path_length;
            path_data_.min_raw_path_length = current_raw_path_length;
        }
      }
      #endif
      // publishPath(path_nodes);
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
          // 不足4个点，直接不再继续规划
          ROS_WARN("Break because current path has less than 4 path nodes.");
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
      // std::cout << "No path found!" << std::endl;
      break;
      gvg_updated_ = false;
    }
  }
  #ifdef SavePathData
  path_data_.iter_num ++;
  path_data_.saveToFile();
  #endif
  if (!start_on_graph) removeNodeAndConnections(it_start);
  if (!end_on_graph) removeNodeAndConnections(it_goal);  
  if (!(start_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndConnections(start_tmp_node_ptr);
  if (!(end_tmp_node_ptr->pos == IntPoint(-1, -1))) removeNodeAndConnections(end_tmp_node_ptr);
  // 遍历整个graph，将标记为RemovedByXX的恢复
  for (auto& graph : gvg_->getGraphs())
  {
    for(auto& node_pair : graph)
    {
      node_pair.second->RemovedByPVS = false;
    }
  }
  std::cout << "Path search finished, found " << path_found_num << " paths." << std::endl;

  bool vis_single_plan_result = false;
  #ifdef SavePathData
  vis_single_plan_result = true;
  #endif

  if (path_found_num == 0)
  {
    std::cout << "No path found after " << path_search_times << " iterations." << std::endl;
    gvg_updated_ = false;
    return false;
  }
  else if (vis_single_plan_result)// 选择最优的一条。现在不从这里选择了，而是到拓扑路径容器里面去选择
  {
    // std::vector<std::vector<Eigen::Vector3d>> paths_for_publish;
    // std::vector<Eigen::Vector3d> path_for_publish;
    std::vector<double> path_lengths;
    std::vector<double> path_angle_diffs, path_angle_penalties;
    std::vector<double> path_sim_to_last;
    std::vector<std::vector<Eigen::Vector2d>> disected_paths = discretizePaths(voro_paths_raw);
    int idx = -1;
    for (const auto& path : voro_paths_raw)
    {
      ++idx;
      path_lengths.push_back(pathLength(path));      
      double path_angle = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
      double angle_diff = path_angle - start_yaw;
      // 归一化到 [-pi, pi]
      while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
      path_angle_diffs.emplace_back(std::abs(angle_diff));
      path_angle_penalties.emplace_back(std::abs(1.0 - std::cos(path_angle_diffs.back())));
      if (last_best_path_.empty()) 
      {
        path_sim_to_last.push_back(0.0); // 如果没有上一个最优路径，则相似度为0
        continue;
      }
      // 计算与上一个最优路径的相似度
      double sim = 0.0;
      int check_num = std::min(std::min(disected_paths[idx].size(), last_best_path_.size()), (size_t)(10.0/resolution_));
      for (int i = 0; i < check_num; ++i)
      {
        double dist = (disected_paths[idx][i] - last_best_path_[i]).norm();
        sim += dist;
      }
      path_sim_to_last.push_back(sim / check_num);
    }

    int best_path_idx = 0;
    double min_penalty = 1000000.0;
    double w_angle = 0.0;    // 权重，控制角度惩罚和长度惩罚的比例
    double w_sim_last = 0.0; // 权重，控制与上一个最优路径的相似度的影响
    for (int i = 0; i < path_found_num; ++i)
    {
      double penalty = path_lengths[i] + 
                       (path_angle_penalties[i]) * std::min(path_lengths[i], (10.0/resolution_)) * w_angle + 
                        path_sim_to_last[i] * w_sim_last;
      if (penalty < min_penalty)
      {
        min_penalty = penalty;
        best_path_idx = i;
      }
    }
    last_best_path_ = disected_paths[best_path_idx];
    std::cout << "Best path index: " << best_path_idx << ", length: " 
              << path_lengths[best_path_idx] << ", angle diff: " 
              << path_angle_diffs[best_path_idx] <<  ", sim to last: " 
              << path_sim_to_last[best_path_idx] << ", penalty: " << min_penalty << std::endl;
    // 发布最优路径
    publishPath2(voro_paths_raw[best_path_idx]);

    gvg_updated_ = false;
    return true;
  }

  int debug = 0;
  return true;
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
      std::vector<std::vector<IntPoint>> raycast_line_to_voronoi(KdNeighborNum);      
      std::vector<int> neighborGraphIdx(KdNeighborNum, start_graph_id);
      std::vector<bool> neighborVisConnect(KdNeighborNum, false);
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
            // if (two_strong_nodes.size() == 2)
            // {
            //   auto node_a = two_strong_nodes[0];
            //   auto node_b = two_strong_nodes[1];
            //   // 从node_a的邻居中移除node_b
            //   for (size_t i = 0; i < node_a->neighbors.size(); ++i) {
            //       auto nb_ptr = node_a->neighbors[i].lock();
            //       if (nb_ptr && nb_ptr->pos == node_b->pos) {
            //           node_a->neighbors.erase(node_a->neighbors.begin() + i);
            //           node_a->neighbor_paths.erase(node_a->neighbor_paths.begin() + i);
            //           break;
            //       }
            //   }
            //   // 从node_b的邻居中移除node_a
            //   for (size_t i = 0; i < node_b->neighbors.size(); ++i) {
            //       auto nb_ptr = node_b->neighbors[i].lock();
            //       if (nb_ptr && nb_ptr->pos == node_a->pos) {
            //           node_b->neighbors.erase(node_b->neighbors.begin() + i);
            //           node_b->neighbor_paths.erase(node_b->neighbor_paths.begin() + i);
            //           break;
            //       }
            //   }
            // }
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

          // IntPoint neighbor_pos(k_nearest_points[i].x, k_nearest_points[i].y);
          // auto connect_path = gvg_planner_->AstarOnVoronoi(raycast_line_hit_voronoi[i], 
          //         neighbor_pos, last_size_x_, last_size_y_, neighborGraphIdx[i]);
          // if(connect_path.empty()) continue;
          // connect_find = true;
          // raycast_line_to_voronoi[i].insert(raycast_line_to_voronoi[i].end(), connect_path.begin(), connect_path.end());
          // node_ptr->type = GraphNode::Strong; // 设置为强节点
          // auto it_neighbor = gvg_->findNodeInGraphs(connect_path.back(), neighborGraphIdx[i]);
          // if (!it_neighbor) continue;         // 其实如果出现这个，是应该要报错的
          // node_ptr->addNeighbor(it_neighbor, raycast_line_to_voronoi[i]);
          // // TODO：这里可能会出错。如果这里直接连接上的是其他graph的节点，直接退出了会有问题。
          // std::reverse(raycast_line_to_voronoi[i].begin(), raycast_line_to_voronoi[i].end());
          // it_neighbor->addNeighbor(node_ptr, raycast_line_to_voronoi[i]); 
          // std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
          //         " node, using the A* to search node. 正向" << std::endl;
          // start_graph_id = neighborGraphIdx[i];
          // return node_ptr;
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
              // if (two_strong_nodes.size() == 2)
              // {
              //   auto node_a = two_strong_nodes[0];
              //   auto node_b = two_strong_nodes[1];
              //   // 从node_a的邻居中移除node_b
              //   for (size_t i = 0; i < node_a->neighbors.size(); ++i) {
              //       auto nb_ptr = node_a->neighbors[i].lock();
              //       if (nb_ptr && nb_ptr->pos == node_b->pos) {
              //           node_a->neighbors.erase(node_a->neighbors.begin() + i);
              //           node_a->neighbor_paths.erase(node_a->neighbor_paths.begin() + i);
              //           break;
              //       }
              //   }
              //   // 从node_b的邻居中移除node_a
              //   for (size_t i = 0; i < node_b->neighbors.size(); ++i) {
              //       auto nb_ptr = node_b->neighbors[i].lock();
              //       if (nb_ptr && nb_ptr->pos == node_a->pos) {
              //           node_b->neighbors.erase(node_b->neighbors.begin() + i);
              //           node_b->neighbor_paths.erase(node_b->neighbor_paths.begin() + i);
              //           break;
              //       }
              //   }
              // }
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

            // auto connect_path = gvg_planner_->AstarOnVoronoi(raycast_points.back(), 
            //         neighbor_pos, last_size_x_, last_size_y_, neighborGraphIdx[i]);
            // if(connect_path.empty()) continue;
            // connect_find = true;
            // raycast_points.insert(raycast_points.end(), connect_path.begin(), connect_path.end());
            // node_ptr->type = GraphNode::Strong; // 设置为强节点
            // auto it_neighbor = gvg_->findNodeInGraphs(connect_path.back(), neighborGraphIdx[i]);
            // if (!it_neighbor) continue;         // 其实如果出现这个，是应该要报错的
            // node_ptr->addNeighbor(it_neighbor, raycast_points);
            // // TODO: 或许也不用反向添加，因为一定是从start_node向外扩展，或向goal_node扩展
            // std::reverse(raycast_points.begin(), raycast_points.end());
            // it_neighbor->addNeighbor(node_ptr, raycast_points);
            // std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
            //           " node, using the A* to search node. 反向" << std::endl;        
            // start_graph_id = neighborGraphIdx[i];
            // return node_ptr;
        }


        std::cout << "No valid neighbors found for " << (startNode ? "start" : "end") << 
                  " node, using the closet node." << std::endl;
        // 如果没有有效的邻居，则使用最近的节点
        IntPoint closest_node = IntPoint(k_nearest_points[0].x, k_nearest_points[0].y);
        auto iter_node = gvg_->findNodeInGraphs(closest_node, start_graph_id);
        if (iter_node) {
            std::cout << "Closest node found: (" << closest_node.x << ", " << closest_node.y << ")" << std::endl;
            node_ptr = iter_node; 
            node_on_graph = true;
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
    ros::Duration(0.005).sleep();
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
    ros::Duration(0.005).sleep();
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
    const float max_dist = clearance_high_ + 0.2;
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
            if(dist >= clearance_high_ && dist <= clearance_high_ + 1e-2)
            {
              esdf_img.at<cv::Vec3b>(last_size_y_ - y - 1, x) = cv::Vec3b(gray, 0, 0);
            }
            else if (dist > clearance_high_ + 1e-2  && dist <= clearance_high_ + 0.1)
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

 vector<Eigen::Vector2d> VoronoiLayer::shortcutPath2(const std::vector<IntPoint>& path, int path_id, int iter_num)
 {
  vector<Eigen::Vector2d> short_path, last_path, process_path;
  for (const auto& pt : path) {
    short_path.emplace_back(pt.x + 0.5, pt.y + 0.5); 
  }

  // if(1) publishTestPath(short_path, 1);
  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

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

double VoronoiLayer::pathLength(const vector<IntPoint>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    Eigen::Vector2d diff = Eigen::Vector2d(path[i + 1].x, path[i + 1].y) - 
                           Eigen::Vector2d(path[i].x, path[i].y);
    length += diff.norm();
  }
  return length;
}

bool VoronoiLayer::lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double thresh,
                             Eigen::Vector2d& pc, int caster_id, int skip_mode) {
    Eigen::Vector3d ray_pt;
    Eigen::Vector2i pt_id;
    double dist;

    thresh = std::max(thresh, 1e-3);
    // Eigen::Vector2d dir = p2 - p1;
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

      // dist = voronoi_.getDistance(pt_id(0), pt_id(1));
      dist = esdf_data_[pt_id(1) * last_size_x_ + pt_id(0)] * resolutuon_inv_;
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
          // dists[x][y] = voronoi_.getDistance(current_idx.x(), current_idx.y());
          dists[x][y] = esdf_data_[current_idx.y() * last_size_x_ + current_idx.x()] * resolutuon_inv_;
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
  for (int x = 0; x < last_size_x_; ++x)
  {
    for (int y = 0; y < last_size_y_; ++y)
    {
      int idx = y * last_size_x_ + x;
      esdf_data_[idx] = (voronoi_.getDistance(x, y)) * resolution_;    
    }
  }

}

#ifdef SavePathData
void VoronoiLayer::rawPathCallback(const nav_msgs::PathConstPtr& msg)
{
    static int iter_num = 0;
    iter_num++;
    std::string iter_num_str = std::to_string(iter_num);
    path_data_.reset();    
    // 检查是否有路径数据
    if (msg->poses.empty()) {
        ROS_WARN("Received empty raw path. FAEL plan fail!");
        path_data_.iter_num ++;
        path_data_.t_search += 0.0;
        path_data_.t_shortcut += 0.0;
        path_data_.min_path_length = -1;
        path_data_.min_raw_path_length = -1;
        path_data_.saveToFile();
        return;
    }

    ROS_INFO("Received raw path with %zu poses.", msg->poses.size());
    if (msg->poses.size() == 3)
    {
      ROS_WARN("Raw path only has start and goal, no need to plan.");
      path_data_.iter_num ++;
      path_data_.t_search += 0.0;
      path_data_.t_shortcut += 0.0;
      path_data_.min_path_length = -1;
      path_data_.min_raw_path_length = -1;
      path_data_.saveToFile();
      return;
    }

    double offset_x = -50 / resolution_;
    double offset_y = -50 / resolution_;
    // 转换路径点到当前维诺图的坐标系
    std::vector<IntPoint> path_nodes;
    std::vector<Eigen::Vector2d> path_nodes2d;
    for (const auto& pose : msg->poses) {
        // 提取世界坐标
        double world_x = pose.pose.position.x;
        double world_y = pose.pose.position.y;

        // 转换到维诺图坐标系
        int map_x = WorldToMapX(world_x);
        int map_y = WorldToMapY(world_y);

        // 添加偏移处理
        int voronoi_x = map_x + offset_x; 
        int voronoi_y = map_y + offset_y;

        path_nodes.emplace_back(voronoi_x, voronoi_y);
        path_nodes2d.emplace_back(voronoi_x, voronoi_y);
    }
    if (voronoi_.getDistanceNoBoundCheck(path_nodes[0].x, path_nodes[0].y) < clearance_low_ / resolution_)
    {
      ROS_WARN("The first node is occupied, please check the raw path.");
      return;
    }
    if (voronoi_.getDistanceNoBoundCheck(path_nodes.back().x, path_nodes.back().y) < clearance_low_ / resolution_)
    {
      ROS_WARN("The last node is occupied, please check the raw path.");
      return;   
    }

    SimpleTimer timer;
    double t_search = 0.0, t_shortcut = 0.0;
    timer.reset();
    vector<Eigen::Vector2d> short_path = shortcutPath2(path_nodes, 0, 2);
    t_shortcut += timer.print(iter_num_str + ": Plan Short Path Shortcut");
    {
      path_data_.iter_num ++;
      path_data_.start_x = (path_nodes2d.front().x() + offset_x) * resolution_;
      path_data_.start_y = (path_nodes2d.front().y() + offset_y)* resolution_;
      path_data_.goal_x = (path_nodes2d.back().x() + offset_x) * resolution_;
      path_data_.goal_y = (path_nodes2d.back().y() + offset_y) * resolution_;
      path_data_.t_search += t_search;
      path_data_.t_shortcut += t_shortcut;
      double current_path_length = pathLength(short_path) * resolution_;
      if (current_path_length < path_data_.min_path_length)
      {
          path_data_.min_path_length = current_path_length;
          path_data_.min_raw_path_length = pathLength(path_nodes2d) * resolution_;
      }
      path_data_.saveToFile();
    }
    publishPath(path_nodes);
    publishPath2(short_path);
}
#endif

std::vector<IntPoint> VoronoiLayer::SampleAstar(IntPoint start, IntPoint goal) {
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    double tie_breaker = 1.0 + 1.0 / 10000;
    auto heuristic = [tie_breaker](int x1, int y1, int x2, int y2) {
        int dx = std::abs(x1 - x2);
        int dy = std::abs(y1 - y2);
        int diag = std::min(dx, dy);
        double h = std::sqrt(2.0) * diag + 1.0 * (std::max(dx, dy) - diag); // diagonal距离 + 直线距离
        return tie_breaker * h;

        // int dx = std::abs(x1 - x2);
        // int dy = std::abs(y1 - y2);
        // double h = std::sqrt(dx * dx + dy * dy); // 欧式距离
        // return tie_breaker * h;

    };

    float esdf_start = voronoi_.getDistance(start.x, start.y);
    float esdf_goal = voronoi_.getDistance(goal.x, goal.y);
    if (esdf_start < clearance_low_ / resolution_ || esdf_goal < clearance_low_ / resolution_) {
        ROS_WARN("Start or goal position is occupied, cannot find path.");
        return {};
    }
    // 使用一维数组代替二维数组
    std::vector<double> gScore(last_size_x_ * last_size_y_, std::numeric_limits<double>::infinity());
    std::vector<double> accumX(last_size_x_ * last_size_y_, 0.0);
    std::vector<double> accumY(last_size_x_ * last_size_y_, 0.0);
    std::vector<IntPoint> cameFrom(last_size_x_ * last_size_y_, {-1, -1});
    auto index = [&](int x, int y) { return y * last_size_x_ + x; };

    // 优先队列替代 openList
    using Node = std::pair<double, IntPoint>; // {f_score, point}
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

    gScore[index(start.x, start.y)] = 0;
    openSet.push({heuristic(start.x, start.y, goal.x, goal.y), start});

    while (!openSet.empty()) {
        auto [_, current] = openSet.top();
        openSet.pop();

        if (current == goal) {
            // 回溯路径
            std::vector<IntPoint> path;
            while (!(current == start)) {
                path.push_back(current);
                current = cameFrom[index(current.x, current.y)];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int dir = 0; dir < 8; ++dir) {
            int nx = current.x + dx[dir];
            int ny = current.y + dy[dir];
            if (nx < 0 || ny < 0 || nx >= last_size_x_ || ny >= last_size_y_) continue;

            if (voronoi_.getDistance(nx, ny) < clearance_low_ / resolution_) continue;

            double cost = std::hypot(dx[dir], dy[dir]);
            double tentative_accumX = accumX[index(current.x, current.y)] + std::abs(dx[dir]);
            double tentative_accumY = accumY[index(current.x, current.y)] + std::abs(dy[dir]);
            double tentative_g = std::sqrt(tentative_accumX * tentative_accumX + tentative_accumY * tentative_accumY);
            tentative_g = gScore[index(current.x, current.y)] + cost; // 加上这一行的话，就是cost是用的最原始的。

            if (tentative_g < gScore[index(nx, ny)]) {
                gScore[index(nx, ny)] = tentative_g;
                accumX[index(nx, ny)] = tentative_accumX;
                accumY[index(nx, ny)] = tentative_accumY;
                cameFrom[index(nx, ny)] = current;
                openSet.push({tentative_g + heuristic(nx, ny, goal.x, goal.y), {nx, ny}});
            }
        }
    }

    return {}; // 无路径
}
}  // namespace costmap_2d