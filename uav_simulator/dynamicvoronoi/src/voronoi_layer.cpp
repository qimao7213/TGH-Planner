
#include "dvr/voronoi_layer.h"
// #define VERBOSE
namespace DynaVoro
{
VoronoiLayer::VoronoiLayer(ros::NodeHandle& nh)
{
  resolution_ = nh.param("/map_resolution", 0.1);
  clearance_low_ = nh.param("/obs_clearance", 0.3);
  clearance_high_ = nh.param("/obs_clearance_high", 3.0);
  clearance_low_thr_ = clearance_low_ * clearance_low_ / resolution_ / resolution_;
  clearance_high_thr_ = clearance_high_ * clearance_high_ / resolution_ / resolution_;

  gvg_ = std::make_shared<GVG>();
  gvg_->setClearanceThresholdSq(clearance_low_thr_, clearance_high_thr_);
  gvg_planner_ = std::make_shared<gvg::Planner>();
  gvg_planner_->init(gvg_);

  costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D", 1, &VoronoiLayer::costmapCB, this);
  odometry_sub_ = nh.subscribe<nav_msgs::Odometry>("/car/odom", 1, &VoronoiLayer::odometryCallback, this);
  rviz_goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &VoronoiLayer::rvizGoalCallback, this);
  
  voronoi_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi_grid", 10);
  voronoi_occupy_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi_grid_occupy", 10);
  distance_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voronoi_esdf", 10);
  gvg_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/voronoi_gvg_markers", 10);
  path_pub_ = nh.advertise<nav_msgs::Path>("/voronoi_path", 10);
  path_pub2_ = nh.advertise<nav_msgs::Path>("/voronoi_path_short", 10);
  path_pub_test_1_ = nh.advertise<nav_msgs::Path>("/voronoi_path_test1", 10);
  path_pub_test_2_ = nh.advertise<nav_msgs::Path>("/voronoi_path_test2", 10);
  path_pub_test_3_ = nh.advertise<nav_msgs::Path>("/voronoi_path_test3", 10);

  plan_timer_ = nh.createTimer(ros::Duration(0.5), &VoronoiLayer::planTimerCallback, this);
  voronoi_update_timer_ = nh.createTimer(ros::Duration(1), &VoronoiLayer::voronoiUpdateTimerCallback, this);
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
  for (unsigned int j = 1; j < size_y - 2; ++j)
  {
    for (unsigned int i = 1; i < size_x - 2; ++i)
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
  for (unsigned int x = 0; x < last_size_x_; ++x)
  {
    for (unsigned int y = 0; y < last_size_y_; ++y)
    {
      // XX
      // if (voronoi_.isVoronoiAlternative(x, y))
      // if (voronoi_.isVoronoiWithDisThr(x, y, clearance_low_thr_, clearance_high_thr_))
      // {
      //   grid.data[x + y * last_size_x_] = 128;
      // }
      if (voronoi_.isOccupied(x, y))
      {
        grid_occupy.data[x + y * last_size_x_] = -1;
      }
    }
  }
  // voronoi_grid_pub_.publish(grid);
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
    grid.info.origin.position.z = 0.1;
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
              if(node_ptr->neighbors[i]->type == GraphNode::Weak)
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

bool VoronoiLayer::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
{
  int KdNeighborNum = 5; // KD-Tree最近邻搜索的数量
  //先将start和goal转换为int类型，在map的坐标系下
  Eigen::Vector2i start_coord, goal_coord;
  start_coord.x() = WorldToMapX(start.x());
  start_coord.y() = WorldToMapY(start.y());
  goal_coord.x() = WorldToMapX(goal.x());
  goal_coord.y() = WorldToMapY(goal.y());

  auto gvg_graphs = gvg_->getGraphs();
  pcl::PointCloud<pcl::PointXY>::Ptr node_cloud(new pcl::PointCloud<pcl::PointXY>);
  pcl::PointXY tmpPoint;

  SimpleTimer timer;
  //TODO: 这里需要考虑多个图的情况
  if (gvg_graphs.size() >= 2)
  {
    ROS_WARN_STREAM("There are more than one GVG graphs, only the first one will be used for planning.");
  }
  auto& graph = gvg_graphs[0];
  std::cout << "graph size: " << graph.size() << std::endl;
  KdNeighborNum = std::min(KdNeighborNum, static_cast<int>(graph.size()));
  for (const auto& node_pair : graph)
  {
    const auto& node_ptr = node_pair.second;
    const IntPoint& node_pos = node_ptr->pos;
    if (node_ptr->type == GraphNode::Strong) 
    {
      tmpPoint.x = node_pos.x;
      tmpPoint.y = node_pos.y;
      node_cloud->points.emplace_back(tmpPoint);
    }
  }
  if (node_cloud->points.empty())
  {
    std::cout << "There is no graph. Return the stright of start and end." << std::endl;
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    // 将栅格坐标转换为世界坐标
    pose.pose.position.x = start.x();
    pose.pose.position.y = start.y();
    pose.pose.position.z = 0.1;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.emplace_back(pose);
    pose.pose.position.x = goal.x();
    pose.pose.position.y = goal.y();
    pose.pose.position.z = 0.1;
    path_msg.poses.emplace_back(pose);
    path_pub2_.publish(path_msg);
    return true;
  }
  GvgNodeKdTree_.setInputCloud(node_cloud);

  // 先判断起点和终点是否在图中，如果不在，则将其初始化为GraphNode
  // 找到离它num个最近的节点，然后从当前位置raycast到每个节点，如果路径无碰撞，则将最近节点作为其neibor，并修改原graph的结构
  // 如果所有路径都有碰撞，则将最近的那个节点作为StartNode或者GoalNode
  auto it_start = creatPathNode(
      start_coord, graph, node_cloud, KdNeighborNum, true);
  auto it_goal = creatPathNode(
      goal_coord, graph, node_cloud, KdNeighborNum, false);

  #ifdef VERBOSE
    timer.print("Plan KD-Tree");
  #endif
  timer.reset();
  if(gvg_planner_->serachPath(it_start, it_goal))
  {
    #ifdef VERBOSE
      timer.print("Plan A* Search"); 
    #endif
    auto path_nodes = gvg_planner_->getFullPath();
    // for (int i = 1; i < path_nodes.size(); ++i)
    // {
    //   if (path_nodes[i] == path_nodes[i - 1])
    //   {
    //     ROS_WARN_STREAM("Path contains duplicate nodes at index " << i << ": (" 
    //                     << path_nodes[i].x << ", " << path_nodes[i].y << ")");
    //   }
    //   std::cout << "path seg dis: " << std::sqrt(std::pow(path_nodes[i].x - path_nodes[i - 1].x, 2) + 
    //                std::pow(path_nodes[i].y - path_nodes[i - 1].y, 2)) << std::endl;
    // }
    publishPath(path_nodes);
    timer.reset();
    vector<Eigen::Vector2d> short_path = shortcutPath(path_nodes, 0, 3);
    #ifdef VERBOSE
      timer.print("Plan Short Path");
    #endif
    publishPath2(short_path);
    return true;
  }
  else
  {
    std::cout << "No path found!" << std::endl;
    return false;
  }
  int debug = 0;
}

// TODO：换成A*搜索
GraphNode::Ptr VoronoiLayer::creatPathNode(
    const Eigen::Vector2i& node_idx,
    std::unordered_map<IntPoint, GraphNode::Ptr>& graph,
    pcl::PointCloud<pcl::PointXY>::Ptr node_cloud,
    int KdNeighborNum,
    bool startNode)
{
  GraphNode::Ptr node_ptr = std::make_shared<GraphNode>();
  node_ptr->pos = IntPoint(node_idx(0), node_idx(1));
  auto iter_node = graph.find(node_ptr->pos);
  if (iter_node != graph.end()) {
      std::cout << "The node on the Graph." << std::endl;
      node_ptr = iter_node->second; // 如果找到了，就使用原来的节点
  } else {
      std::cout << "The node not found in the Graph, creating a new one." << std::endl;  
      std::vector<int> pointIdxNKNSearch(KdNeighborNum);
      std::vector<float> pointNKNSquaredDistance(KdNeighborNum);
      pcl::PointXY queryPoint;
      queryPoint.x = node_idx(0);
      queryPoint.y = node_idx(1);
      GvgNodeKdTree_.nearestKSearch(queryPoint, KdNeighborNum, pointIdxNKNSearch, pointNKNSquaredDistance);

      std::vector<IntPoint> raycast_points; //raycast得到的就是四邻域连接
      Eigen::Vector2i pc;
      Eigen::Vector2i raycastStart(node_idx(0), node_idx(1));
      int valid_neighbors = 0;
      for (int i = 0; i < KdNeighborNum; ++i)
      {
        Eigen::Vector2i raycastEnd(node_cloud->points[pointIdxNKNSearch[i]].x, node_cloud->points[pointIdxNKNSearch[i]].y);
        // std::cout << "raycastStart: (" << raycastStart.x() << ", " << raycastStart.y() << "), "
        //           << "raycastEnd: (" << raycastEnd.x() << ", " << raycastEnd.y() << ")" << std::endl;
        bool no_collision = lineVisib(raycastStart, raycastEnd, raycast_points, clearance_low_/resolution_, pc);
        if (no_collision)
        {
          node_ptr->type = GraphNode::Strong; // 设置为强节点
          auto it_neighbor = graph.find(IntPoint(raycastEnd.x(), raycastEnd.y()));
          if (it_neighbor != graph.end()) {
              node_ptr->addNeighbor(it_neighbor->second, raycast_points);
              // TODO: 或许也不用反向添加，因为一定是从start_node向外扩展，或向goal_node扩展
              std::reverse(raycast_points.begin(), raycast_points.end());
              it_neighbor->second->addNeighbor(node_ptr, raycast_points);
          }
          else
          {
            std::cout << "Error: Neighbor node not found in the graph." << std::endl;
          }
          ++ valid_neighbors;
        }
      }
      if (valid_neighbors == 0)
      {
        std::cout << "No valid neighbors found for start node, using the closest node." << std::endl;
        // 如果没有有效的邻居，则使用最近的节点
        IntPoint closest_node = IntPoint(node_cloud->points[pointIdxNKNSearch[0]].x, node_cloud->points[pointIdxNKNSearch[0]].y);
        auto iter_node = graph.find(closest_node);
        if (iter_node != graph.end()) {
            std::cout << "Closest node found: (" << closest_node.x << ", " << closest_node.y << ")" << std::endl;
            node_ptr = iter_node->second; 
        }
      }
  }
  return node_ptr;
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
    for (int y = 0; y < last_size_y_; ++y)
    {
        for (int x = 0; x < last_size_x_; ++x)
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
        colli_pt = colli_pt + push_dir;
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
      dist = voronoi_.getDistance(pt_id(0), pt_id(1));
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
      dist = voronoi_.getDistance(pt_id(0), pt_id(1));
      // std::cout << "dist: " << dist << std::endl;
      if (dist <= (thresh)) {
        pc(0) = pt_id(0);
        pc(1) = pt_id(1);
        return false;
      }
      else {
        if(iter > 0) //跳过第一个点，因为这个是起点
          rayline_list.emplace_back(pt_id(0), pt_id(1));
      }
      ++iter;
    }
    // rayline_list.emplace_back(p2(0), p2(1)); // 添加终点
    return true;
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

  has_odom_ = true;
}

void VoronoiLayer::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!has_odom_) {
    ROS_WARN("No odometry data received yet, cannot set goal.");
    return;
  }

  Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  lastest_goal_ = goal;
  has_goal_ = true;
  // this->plan(odom_pos_, goal);
}

void VoronoiLayer::planTimerCallback(const ros::TimerEvent& event)
{
  if (!has_odom_ || !has_goal_) {
    ROS_WARN("No odometry data received yet, cannot plan.");
    return;
  }
  this->plan(odom_pos_, lastest_goal_);
}

void VoronoiLayer::voronoiUpdateTimerCallback(const ros::TimerEvent& event)
{
  boost::unique_lock<boost::mutex> lock(mutex_);



  std::vector<IntPoint> new_free_cells, new_occupied_cells;
  for (unsigned int j = 0; j < last_size_y_ ; ++j)
  {
    for (unsigned int i = 0; i < last_size_x_; ++i)
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
