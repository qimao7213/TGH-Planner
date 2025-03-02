#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <Eigen/Eigen>
#include <random>

#include "common_srvs/load_or_save_global_map.h"
using namespace std;

// 判断点 p 是否在向量 AB 的右侧（返回正数）或左侧（返回负数）或在向量上（返回 0）
double crossProduct(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& P) {
    Eigen::Vector2d AB = B - A;
    Eigen::Vector2d AP = P - A;
    return AB.x() * AP.y() - AB.y() * AP.x();
}

bool isPointInRectangle(const Eigen::Vector2d corners[4], const Eigen::Vector2d& p) {
    // 对四条边进行叉积计算
    double cross1 = crossProduct(corners[0], corners[1], p);
    double cross2 = crossProduct(corners[1], corners[2], p);
    double cross3 = crossProduct(corners[2], corners[3], p);
    double cross4 = crossProduct(corners[3], corners[0], p);

    // 判断叉积符号是否相同
    if ((cross1 >= 0 && cross2 >= 0 && cross3 >= 0 && cross4 >= 0) ||
        (cross1 <= 0 && cross2 <= 0 && cross3 <= 0 && cross4 <= 0)) {
        return true;
    } else {
        return false;
    }
}

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Publisher global_map_change_pub_;
ros::Subscriber _odom_sub;
ros::ServiceServer change_map_srv_;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_, wall_num_, u_wall_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void generateWallCloud(const Eigen::Vector2d corners[4], 
                       double x, double y, double z, 
                       double width, double length, double height, 
                       const Eigen::Matrix2d& rotation) 
{
    pcl::PointXYZ pt_random;
    for (double dx = -length / 2; dx <= length / 2; dx += _resolution) {
        for (double dy = -width / 2; dy <= width / 2; dy += _resolution) {
            for (double dz = -height / 2; dz <= height / 2; dz += _resolution) {
                Eigen::Vector3d pt(dx, dy, dz);

                // 旋转并平移
                Eigen::Vector3d pt_transformed = Eigen::Vector3d((rotation * pt.head<2>()).x(), 
                                (rotation * pt.head<2>()).y(), pt.z()) + Eigen::Vector3d(x, y, z);
                pt_random.x = pt_transformed.x();
                pt_random.y = pt_transformed.y();
                pt_random.z = pt_transformed.z();

                // 确保点在地图范围内
                if (pt_random.x >= _x_l && pt_random.x <= _x_h &&
                    pt_random.y >= _y_l && pt_random.y <= _y_h) 
                    {
                      // 确保点不在起点周围
                      if((pt_random.x - _init_x) * (pt_random.x - _init_x) + 
                         (pt_random.y - _init_y) * (pt_random.y - _init_y) < 4)
                         continue;
                     cloudMap.push_back(pt_random);
                    }
            }
        }
    }
}



void RandomMapGenerate() {
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -30; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          if((pt_random.x - _init_x) * (pt_random.x - _init_x) + 
            (pt_random.y - _init_y) * (pt_random.y - _init_y) < 4)
            continue;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            if((pt_random.x - _init_x) * (pt_random.x - _init_x) + 
               (pt_random.y - _init_y) * (pt_random.y - _init_y) < 4)
                 continue;
            cloudMap.push_back(pt_random);
          }
    }
  }

  // generate wall obs
  // int wall_num_gen = 0;
  for (int i = 0; i < wall_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    // 将坐标对齐到栅格中心
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    // 随机生成yaw角
    double theta = rand_theta_(eng);

    // 设置长方体的宽和高度，长度是宽度的7倍
    double width = rand_w(eng);
    double height = rand_h(eng); height = 3.0;
    double length = 8 * width;
    // width = length;

    // 计算长方体的四个顶点
    Eigen::Matrix2d rotation;
    rotation << cos(theta), -sin(theta), sin(theta), cos(theta);

    Eigen::Vector2d half_width_length(width / 2.0, length / 2.0);

    Eigen::Vector2d corners[4];
    corners[0] = rotation * Eigen::Vector2d(-half_width_length.x(), -half_width_length.y()) + Eigen::Vector2d(x, y);
    corners[1] = rotation * Eigen::Vector2d(-half_width_length.x(), half_width_length.y()) + Eigen::Vector2d(x, y);
    corners[2] = rotation * Eigen::Vector2d(half_width_length.x(), half_width_length.y()) + Eigen::Vector2d(x, y);
    corners[3] = rotation * Eigen::Vector2d(half_width_length.x(), -half_width_length.y()) + Eigen::Vector2d(x, y);

    // 确保生成的长方体不在指定的初始位置附近
    if(isPointInRectangle(corners, Eigen::Vector2d(_init_x, _init_y))) 
    {
        i--;
        continue;      
    }

    // 检查所有顶点是否都在边界内
    bool in_bounds = true;
    for (const auto& corner : corners) {
        if (corner.x() < _x_l || corner.x() > _x_h || corner.y() < _y_l || corner.y() > _y_h) {
            in_bounds = false;
            break;
        }
    }

    // 如果任何一个顶点不在边界内，跳过这个长方体
    if (!in_bounds) {
        i--;
        continue;
    }

    // 生成长方体的点云
    for (double dx = -length / 2; dx <= length / 2; dx += _resolution) {
        for (double dy = -width / 2; dy <= width / 2; dy += _resolution) {
            for (double dz = -height / 2; dz <= height / 2; dz += _resolution) {
                Eigen::Vector3d pt(dx, dy, dz);

                // 旋转并平移
                Eigen::Vector3d pt_transformed = Eigen::Vector3d((rotation * pt.head<2>()).x(), 
                                (rotation * pt.head<2>()).y(), pt.z()) + Eigen::Vector3d(x, y, z);
                pt_random.x = pt_transformed.x();
                pt_random.y = pt_transformed.y();
                pt_random.z = pt_transformed.z();

                // 确保点在地图范围内
                if (pt_random.x >= _x_l && pt_random.x <= _x_h &&
                    pt_random.y >= _y_l && pt_random.y <= _y_h 
                    // pt_random.z >= _h_l && pt_random.z <= _h_h
                    ) 
                    {
                    if((pt_random.x - _init_x) * (pt_random.x - _init_x) + 
                    (pt_random.y - _init_y) * (pt_random.y - _init_y) < 4)
                        continue;
                     cloudMap.push_back(pt_random);
                    }
            }
        }
    }
  }

  // 生成U型墙的点云
  // U型墙由三个长方体组成：左右两侧和底部
  for (int i = 0; i < u_wall_num_; ++i) {
      double x, y, z;
      x = rand_x(eng);
      y = rand_y(eng);
      z = rand_z_(eng);

      // 将坐标对齐到栅格中心
      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;
      z = floor(z / _resolution) * _resolution + _resolution / 2.0;

      // 随机生成yaw角
      double theta = rand_theta_(eng);;
      Eigen::Matrix2d rotation;
      rotation << cos(theta), -sin(theta), sin(theta), cos(theta);

      // 设置长方体的宽和高度
      double width = rand_w(eng);
      double height = 3.0;
      double length = 10 * width;

      // 生成U型墙的左侧
      Eigen::Vector2d left_corners[4];
      left_corners[0] = rotation * Eigen::Vector2d(-width / 2.0, -length / 2.0) + Eigen::Vector2d(x + length / 2, y - length / 2);
      left_corners[1] = rotation * Eigen::Vector2d(-width / 2.0, length / 2.0) + Eigen::Vector2d(x + length / 2, y - length / 2);
      left_corners[2] = rotation * Eigen::Vector2d(width / 2.0, length / 2.0) + Eigen::Vector2d(x + length / 2, y - length / 2);
      left_corners[3] = rotation * Eigen::Vector2d(width / 2.0, -length / 2.0) + Eigen::Vector2d(x + length / 2, y - length / 2);

      // 生成U型墙的右侧
      Eigen::Vector2d right_corners[4];
      right_corners[0] = rotation * Eigen::Vector2d(-width / 2.0, -length / 2.0) + Eigen::Vector2d(x + length / 2, y);
      right_corners[1] = rotation * Eigen::Vector2d(-width / 2.0, length / 2.0) + Eigen::Vector2d(x + length / 2, y);
      right_corners[2] = rotation * Eigen::Vector2d(width / 2.0, length / 2.0) + Eigen::Vector2d(x + length / 2, y);
      right_corners[3] = rotation * Eigen::Vector2d(width / 2.0, -length / 2.0) + Eigen::Vector2d(x + length / 2, y);

      // 生成U型墙的底部
      Eigen::Vector2d bottom_corners[4];
      bottom_corners[0] = rotation * Eigen::Vector2d(-length / 2.0, -width / 2.0) + Eigen::Vector2d(x, y - length / 2);
      bottom_corners[1] = rotation * Eigen::Vector2d(-length / 2.0, width / 2.0) + Eigen::Vector2d(x, y - length / 2);
      bottom_corners[2] = rotation * Eigen::Vector2d(length / 2.0, width / 2.0) + Eigen::Vector2d(x, y - length / 2);
      bottom_corners[3] = rotation * Eigen::Vector2d(length / 2.0, -width / 2.0) + Eigen::Vector2d(x, y - length / 2);
      bool in_bounds = true;
      for (const auto& corner : left_corners) {
          if (corner.x() < _x_l || corner.x() > _x_h || corner.y() < _y_l || corner.y() > _y_h) {
              in_bounds = false;
              break;
          }
      }
      for (const auto& corner : right_corners) {
          if (corner.x() < _x_l || corner.x() > _x_h || corner.y() < _y_l || corner.y() > _y_h) {
              in_bounds = false;
              break;
          }
      }
      for (const auto& corner : bottom_corners) {
          if (corner.x() < _x_l || corner.x() > _x_h || corner.y() < _y_l || corner.y() > _y_h) {
              in_bounds = false;
              break;
          }
      }
      // 如果任何一个顶点不在边界内，跳过这个长方体
      if (!in_bounds) {
          i--;
          continue;
      }


      // 生成左侧的点云
      generateWallCloud(left_corners, x + length / 2, y - length, z, width, length, height, rotation);

      // 生成右侧的点云
      generateWallCloud(right_corners, x + length / 2, y, z, width, length, height, rotation);

      // 生成底部的点云
      generateWallCloud(bottom_corners, x, y - length / 2, z, length, width, height, rotation);
  }




  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
void pubSensedPoints() {
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
  // }

  return;

  /* ---------- only publish points around current position ---------- */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

//暂时没有使用
void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

bool global_map_change(common_srvs::load_or_save_global_map::Request &req, 
                       common_srvs::load_or_save_global_map::Response &res)
{
  string file_name = "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/uav_simulator/map_generator/global_map/"
                     + req.file_name + ".pcd";
  if(req.load_or_save == 2)
  {
    ROS_WARN_STREAM("Save Global Map to: " << (file_name));
    int save_success = pcl::io::savePCDFileASCII((file_name), cloudMap);
    if(save_success == 0) res.success = true;
    else res.success = false;
  }
  if(req.load_or_save == 1)
  {
    ROS_WARN_STREAM("Load Global Map from: " << file_name);
    int load_success = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloudMap);
    if(load_success == 0) 
    {
      cloudMap.width = cloudMap.points.size();
      cloudMap.height = 1;
      cloudMap.is_dense = true;
      ROS_WARN("Load Global Map from File.");
      kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
      std_msgs::Empty emt;
      global_map_change_pub_.publish(emt); // 发送一个信号给pcl_render，重新接收一次全局地图
      res.success = true;
    }
    else res.success = false;
  }
  return 1;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");
  //暂时没有使用
  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);//这里为什么分了local和全局呢？

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);
  change_map_srv_ = n.advertiseService("/map_generator/change_map", global_map_change);
  global_map_change_pub_  = n.advertise<std_msgs::Empty>("/map_generator/global_map_change", 10);

  //暂时没有使用
  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);//这个是在干什么呢？
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);
  n.param("map/wall_num", wall_num_, 3);
  n.param("map/u_wall_num", u_wall_num_, 2);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}