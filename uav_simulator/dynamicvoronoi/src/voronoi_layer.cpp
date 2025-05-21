
#include "dvr/voronoi_layer.h"

#include <chrono>  // NOLINT
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
namespace DynaVoro
{
VoronoiLayer::VoronoiLayer(ros::NodeHandle& nh)
{

  costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D_binary", 1, &VoronoiLayer::costmapCB, this);
  // costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &VoronoiLayer::costmapCB, this);
  voronoi_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/voronoi_grid", 1);
  gvg_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/voronoi_gvg_markers", 1);
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
  for (unsigned int j = 1; j < size_y - 1; ++j)
  {
    for (unsigned int i = 1; i < size_x - 1; ++i)
    {
      int index = j * size_x + i;
      if (voronoi_.isOccupied(i, j) && map_msg->data[index] != 100)
      {
        new_free_cells.push_back(IntPoint(i, j));
      }

      if (!voronoi_.isOccupied(i, j) && map_msg->data[index] == 100)
      // if (!voronoi_.isOccupied(i, j) && map_msg->getCost(i, j) >= 128)
      {
        new_occupied_cells.push_back(IntPoint(i, j));
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
  const auto start_timestamp = std::chrono::system_clock::now();

  voronoi_.update();
  voronoi_.prune();
  // XX
  // voronoi_.updateAlternativePrunedDiagram();

  // end timing
  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  // ROS_WARN_STREAM("Runtime=" <<  diff.count() * 1e3 << "ms, new_free size: " << new_free_cells.size() << 
  //                 ", new_occupy size: " << new_occupied_cells.size());

  const auto end_timestamp2 = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff2 = end_timestamp2 - end_timestamp;
  gvg_.createGraph(voronoi_);
  // ROS_WARN_STREAM("Runtime gvg=" <<  diff2.count() * 1e3 << " ms.");

  // // 将找到的图结构画到cv::Mat上
  // cv::Mat voronoiImg = cv::Mat::zeros(size_y, size_x, CV_8UC3);
  // voronoiImg.setTo(cv::Scalar(255, 255, 255));
  // cv::Mat voronoiImg2 = voronoiImg.clone();
  // cv::Mat voronoiImg3 = voronoiImg.clone();
  // for (int y = 0; y < size_y; ++y)
  // {
  //   for (int x = 0; x < size_x; ++x)
  //   {
  //     if (voronoi_.isVoronoi(x, y))
  //     {
  //       voronoiImg.at<cv::Vec3b>(size_y - y, x)[0] = 255;   // 蓝色通道
  //       voronoiImg.at<cv::Vec3b>(size_y - y, x)[1] = 255;   // 绿色通道
  //       voronoiImg.at<cv::Vec3b>(size_y - y, x)[2] = 0;   // 红色通道
  //     }
  //   }
  // }                  
  // auto gvg_graphs = gvg_.getGraphs();
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
  //       cv::circle(voronoiImg, cv::Point(node_pos.x, size_y - node_pos.y), 2, cv::Scalar(0, 0, 255), -1);
  //     }
  //     else if (node_ptr->type == GraphNode::Weak) // 绘制弱节点
  //     {
  //       cv::circle(voronoiImg, cv::Point(node_pos.x, size_y - node_pos.y), 2, cv::Scalar(255, 0, 0), -1);
  //     }
  //     // cv::putText(voronoiImg, std::to_string(index), cv::Point(node_pos.x, size_y - node_pos.y), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);
  //     std::vector<Path> paths = node_ptr->neighbor_paths;
  //     for (const auto& path : paths)
  //     {
  //       if (path.path.empty())
  //       {
  //         continue;
  //       }
  //       for (size_t j = 0; j < path.path.size(); ++j)
  //       {
  //         // std::cout << "x, y: " << path.path[j].x << ", " << size_y - path.path[j].y << std::endl;
  //         voronoiImg.at<cv::Vec3b>(size_y - path.path[j].y, path.path[j].x)[0] = 0;   // 蓝色通道
  //         voronoiImg.at<cv::Vec3b>(size_y - path.path[j].y, path.path[j].x)[1] = 255; // 绿色通道
  //         voronoiImg.at<cv::Vec3b>(size_y - path.path[j].y, path.path[j].x)[2] = 0;   // 红色通道
  //       }
  //     }
  //     int debug = 0;
  //     ++index;
  //   }
  //   int debug = 0;
  // }
  publishGVG(map_msg->info.resolution, gvg_.getGraphs());

  publishVoronoiGrid(size_x, size_x, map_msg->info.resolution);
}

void VoronoiLayer::publishVoronoiGrid(int size_x, int size_y ,float resolution)
{
  unsigned int nx = size_x;
  unsigned int ny = size_y;

  nav_msgs::OccupancyGrid grid;
  // Publish Whole Grid
  grid.header.frame_id = "world";
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution;

  grid.info.width = nx;
  grid.info.height = ny;

  grid.info.origin.position.x = -0.5 * size_x * resolution;
  grid.info.origin.position.y = -0.5 * size_y * resolution;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(nx * ny);

  for (unsigned int x = 0; x < nx; ++x)
  {
    for (unsigned int y = 0; y < ny; ++y)
    {
      // XX
      // if (voronoi_.isVoronoiAlternative(x, y))
      if (voronoi_.isVoronoi(x, y))
      {
        grid.data[x + y * nx] = 128;
      }
      else
      {
        grid.data[x + y * nx] = 0;
      }
    }
  }
  voronoi_grid_pub_.publish(grid);
}


void VoronoiLayer::publishGVG(float resolution, std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>> gvg)
{
    // 创建 MarkerArray 用于存储所有节点的标记
    visualization_msgs::MarkerArray marker_array;

    // 删除之前的标记
    static size_t previous_marker_count = 0; // 记录上一次发布的标记数量
    for (size_t i = 0; i < previous_marker_count; ++i)
    {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "gvg_vectices"; // 命名空间
        delete_marker.id = i;             // 唯一 ID
        delete_marker.action = visualization_msgs::Marker::DELETE; // 删除标记
        marker_array.markers.push_back(delete_marker);
    }

    // 遍历每个图
    size_t marker_count = 0; // 当前标记数量
    for (size_t graph_idx = 0; graph_idx < gvg.size(); ++graph_idx)
    {
        const auto& graph = gvg[graph_idx];

        // 遍历图中的每个节点
        for (const auto& node_pair : graph)
        {
            const auto& node_ptr = node_pair.second; // 获取节点指针
            const IntPoint& node_pos = node_ptr->pos; // 获取节点位置

            // 创建一个 Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "gvg_vectices"; // 命名空间
            marker.id = marker_count++; // 唯一 ID
            marker.type = visualization_msgs::Marker::SPHERE; // 类型为球体
            marker.action = visualization_msgs::Marker::ADD;

            // 设置球体的位置
            marker.pose.position.x = (node_pos.x + 0.5) * resolution - 0.5 * last_size_x_ * resolution;
            marker.pose.position.y = (node_pos.y + 0.5) * resolution - 0.5 * last_size_y_ * resolution;
            marker.pose.position.z = 0.0; // 假设 z 轴为 0
            marker.pose.orientation.w = 1.0;

            // 设置球体的大小
            marker.scale.x = 0.25; // 球体的直径
            marker.scale.y = 0.25;
            marker.scale.z = 0.25;

            // 根据节点类型设置颜色
            if (node_ptr->type == GraphNode::Strong)
            {
                marker.color.r = 0.0; // 红色
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0; // 不透明
            }
            else if (node_ptr->type == GraphNode::Weak)
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0; // 蓝色
                marker.color.b = 1.0;
                marker.color.a = 1.0; // 不透明
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0; // 绿色
                marker.color.b = 0.0;
                marker.color.a = 1.0; // 不透明
            }

            // 将 Marker 添加到 MarkerArray 中
            marker_array.markers.push_back(marker);
        }
    }

    // 更新上一次的标记数量
    previous_marker_count = marker_count;

    // 发布 MarkerArray
    gvg_marker_pub_.publish(marker_array);
}


}  // namespace costmap_2d
