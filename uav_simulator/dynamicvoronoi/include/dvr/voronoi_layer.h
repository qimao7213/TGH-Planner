
#pragma once

#include <memory>

#include <boost/thread.hpp>
#include "costmap_2d/GenericPluginConfig.h"
#include "costmap_2d/cost_values.h"
#include "costmap_2d/layer.h"
#include "costmap_2d/layered_costmap.h"
#include "dvr/dynamicvoronoi.h"
#include "dvr/GVG.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace DynaVoro
{
class VoronoiLayer
{
public:
  VoronoiLayer(ros::NodeHandle& nh);
  void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  const DynamicVoronoi& getVoronoi() const;
  boost::mutex& getMutex();

private:
  void publishVoronoiGrid(int size_x, int size_y ,float resolution);
  void publishGVG(float resolution, std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>> gvg);

  void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

  ros::Publisher voronoi_grid_pub_, gvg_marker_pub_;
  ros::Subscriber costmap_sub_;

  DynamicVoronoi voronoi_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  boost::mutex mutex_;
  GVG gvg_;
};

}  // namespace costmap_2d
