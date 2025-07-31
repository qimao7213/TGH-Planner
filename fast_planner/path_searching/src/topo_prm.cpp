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



#include <path_searching/topo_prm.h>
#include <thread>

void savePointsToFile(const std::vector<Eigen::Vector3d>& points, const std::string& filename) {
    // 创建输出文件流对象
    std::ofstream outFile(filename);
    // 检查文件是否成功打开
    if (!outFile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    // 将vector中的每个元素写入文件
    int iter = 0;
    for (const Eigen::Vector3d& pt : points) {
        outFile << pt(0) << " " << pt(1) << "\n";
        iter ++;
    }
    // 关闭文件
    outFile.close();
    // 检查文件是否成功关闭
    if (!outFile) {
        std::cerr << "Error closing file: " << filename << std::endl;
    }
}
void loadPointsFromFile(std::vector<Eigen::Vector3d>& points, const std::string& filename) {
    // 创建输入文件流对象
    std::ifstream inFile(filename);
    
    // 检查文件是否成功打开
    if (!inFile) {
        std::cerr << "Error opening file for reading: " << filename << std::endl;
        return;
    }
    
    // 读取文件中的每一行
    double x, y;
    while (inFile >> x >> y) {
        // 将文件中的每个点的x, y坐标读取并转换为Eigen::Vector3d
        Eigen::Vector3d pt(x, y, -0.5);  // 设置z坐标为 -0.5
        points.push_back(pt);  // 将点添加到 points 容器中
    }
    
    // 关闭文件
    inFile.close();
    
    // 检查文件是否成功关闭
    if (!inFile) {
        std::cerr << "Error closing file: " << filename << std::endl;
    }
}

bool isNonDecreasing(const std::vector<fast_planner::TopoPath>& path_container) {
  for (size_t i = 1; i < path_container.size(); ++i) {
      if (path_container[i].length < path_container[i - 1].length) {
          return false; // 找到降序的元素，直接返回 false
      }
  }
  return true; // 如果遍历完没有找到降序，返回 true
}
bool visLineStep_ = false;
std::string file_path_;

namespace fast_planner {
TopologyPRM::TopologyPRM(/* args */) {}

TopologyPRM::~TopologyPRM() {}

void TopologyPRM::init(ros::NodeHandle& nh) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // init parameter
  nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0);
  nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);
  nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);
  nh.param("topo_prm/clearance", clearance_, -1.0);
  nh.param("topo_prm/clearance_line", clearance_line_, 0.1); /// 
  nh.param("topo_prm/short_cut_num", short_cut_num_, -1);
  nh.param("topo_prm/reserve_num", reserve_num_, -1);
  nh.param("topo_prm/ratio_to_short", ratio_to_short_, -1.0);
  nh.param("topo_prm/max_sample_num", max_sample_num_, -1);
  nh.param("topo_prm/max_sample_time", max_sample_time_, -1.0);
  nh.param("topo_prm/max_raw_path", max_raw_path_, -1);
  nh.param("topo_prm/max_raw_path2", max_raw_path2_, -1);
  nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);
  nh.param("topo_prm/FilePath", file_path_);
  resolution_ = edt_environment_->sdf_map_->getResolution();
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;
  voronoi_layer_ = std::make_shared<DynaVoro::VoronoiLayer>(nh);
  grah_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/TopoPlan/graph", 20);
  sampled_points_.reserve(max_sample_num_);
  skip_scale_ = ((clearance_line_ * 1.414) / resolution_);
  use_skip_ = true;//是否使用EUVD
  threadPool_ = std::make_shared<ThreadPool>(std::thread::hardware_concurrency());
  for (int i = 0; i < max_raw_path_; ++i) {
    casters_.push_back(RayCaster());
  }
  path_1_pub_ = nh.advertise<nav_msgs::Path>("/TopoPlan/path_1", 10);
  path_2_pub_ = nh.advertise<nav_msgs::Path>("/TopoPlan/path_2", 10);
  path_3_pub_ = nh.advertise<nav_msgs::Path>("/TopoPlan/path_3", 10);
  astar2D_path_finder_.reset(new Astar2D);
  astar2D_path_finder_->setParam(nh);
  astar2D_path_finder_->setEnvironment(edt_environment_);
  astar2D_path_finder_->init();
  ROS_WARN("----Topo path finder init!------");
}

void TopologyPRM::initForTest(ros::NodeHandle& nh) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // init parameter
  nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), 1.0);
  nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), 3.5);
  nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), 0.1);
  nh.param("topo_prm/clearance", clearance_, 0.3); /// 
  nh.param("topo_prm/clearance_line", clearance_line_, 0.3); /// 
  nh.param("topo_prm/short_cut_num", short_cut_num_, 1);
  nh.param("topo_prm/reserve_num", reserve_num_, 6);
  nh.param("topo_prm/ratio_to_short", ratio_to_short_, 4.0);
  // 600, 0.01, 100, 50这一组参数是为了测试DFS和BDFS
  nh.param("topo_prm/max_sample_num", max_sample_num_, 2000);
  nh.param("topo_prm/max_sample_time", max_sample_time_, 0.004); //设置0.004
  nh.param("topo_prm/max_raw_path", max_raw_path_, 300);
  nh.param("topo_prm/max_raw_path2", max_raw_path2_, 100);
  nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, true);
  nh.param("topo_prm/FilePath", file_path_);
  sampled_points_.reserve(max_sample_num_);  
  resolution_ = edt_environment_->sdf_map_->getResolution();
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;
  voronoi_layer_ = std::make_shared<DynaVoro::VoronoiLayer>(nh);
  grah_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/TopoPlan/graph", 20);
  path_1_pub_ = nh.advertise<nav_msgs::Path>("/TopoPlan/path_1", 10);
  path_2_pub_ = nh.advertise<nav_msgs::Path>("/TopoPlan/path_2", 10);
  path_3_pub_ = nh.advertise<nav_msgs::Path>("/TopoPlan/path_3", 10);
  skip_scale_ = ((clearance_line_ * 1.414) / resolution_);
  use_skip_ = true; // 是否使用EUVD
  topo_test_ = true;
  for (int i = 0; i < max_raw_path_; ++i) {
    casters_.push_back(RayCaster());
  }
  threadPool_ = std::make_shared<ThreadPool>(std::thread::hardware_concurrency());
  record_data_.reset();

  astar2D_path_finder_.reset(new Astar2D);
  astar2D_path_finder_->setParam(nh);
  astar2D_path_finder_->setEnvironment(edt_environment_);
  ROS_WARN("----Topo path finder init!------");
}

void TopologyPRM::findVoroPaths(Eigen::Vector3d start, Eigen::Vector3d end,
                                vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& raw_paths,
                                vector<vector<Eigen::Vector3d>>& filtered_paths,
                                vector<vector<Eigen::Vector3d>>& select_paths)
{
  ros::Time t1, t2;
  double voro_plan_time, short_time, prune_time, select_time, preprocess_time;
  /* ---------- create the topo graph ---------- */
  t1 = ros::Time::now();
  start.z() = ground_height_;
  end.z() = ground_height_;
  if(topo_test_ && only2D_) //这里地方是因为测试topo的时候，地图要spin后才产生的. 这个地方想办法简化一下？
  {
    start.z() = ground_height_;
    end.z() = ground_height_;
    for(auto & pt : start_pts) pt.z() = ground_height_;
    for(auto & pt : end_pts) pt.z() = ground_height_;
    resolution_ = edt_environment_->sdf_map_->getResolution();
    offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;
    skip_scale_ = ((clearance_line_ * 1.414) / resolution_);
    astar2D_path_finder_->init();
    edt_environment_->sdf_map_->getRegion(map_origin_, map_size_);
  }
  
  std::cout << "[Topo Voro]: start: " << start.transpose() << ", end: " << end.transpose() << "\n";
  // std::cout << "skip_scale_: " << skip_scale_ << std::endl;
  start_pts_ = start_pts;
  end_pts_ = end_pts;
  record_data_.reset();
  
  // 关闭preprocess并清理掉path_container的话，就相当于关闭了TopoPathSet
  // path_container_front_.clear();  
  // path_container_back_.clear(); 
  preprocess();
  // if (!path_container_front_.empty() && !path_container_back_.empty()) 
  //   return;
  double sleep_time = 0.0;
  if(topo_test_) sleep_time = 0.00;
  ros::Duration(sleep_time).sleep();   
  preprocess_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();
  voronoi_layer_->plan(start, end, 0.0);
  std::cout <<"ssss" << std::endl;
  short_paths_ = voronoi_layer_->getVoroPaths(ground_height_);
  voro_plan_time = (ros::Time::now() - t1).toSec();
  /* ---------- prune equivalent paths ---------- */
  t1 = ros::Time::now();

  std::cout << "pruneEquivalent start!" << std::endl;
  filtered_paths = pruneEquivalent(short_paths_);
  std::cout << "pruneEquivalent end!" << std::endl;

  prune_time = (ros::Time::now() - t1).toSec();

  /* ---------- select N shortest paths ---------- */
  t1 = ros::Time::now();

  std::cout << "selectShortPathsV2 start!" << std::endl;
  selectShortPathsV2(filtered_paths);
  std::cout << "selectShortPathsV2 end!" << std::endl;
  // select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (ros::Time::now() - t1).toSec();
  double total_time = preprocess_time + voro_plan_time + short_time + prune_time + select_time;
  std::cout << "\n[Topo]: total time: " << total_time << ", preprocess: " << preprocess_time 
            << ", voro: " << voro_plan_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;
}


void TopologyPRM::findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end,
                                vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& raw_paths,
                                vector<vector<Eigen::Vector3d>>& filtered_paths,
                                vector<vector<Eigen::Vector3d>>& select_paths) {
  ros::Time t1, t2;
  double graph_time, search_time, short_time, prune_time, select_time, preprocess_time;
  /* ---------- create the topo graph ---------- */
  t1 = ros::Time::now();
  start.z() = ground_height_;
  end.z() = ground_height_;
  if(topo_test_ && only2D_) //这里地方是因为测试topo的时候，地图要spin后才产生的. 这个地方想办法简化一下？
  {
    start.z() = ground_height_;
    end.z() = ground_height_;
    for(auto & pt : start_pts) pt.z() = ground_height_;
    for(auto & pt : end_pts) pt.z() = ground_height_;
    resolution_ = edt_environment_->sdf_map_->getResolution();
    offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;
    skip_scale_ = ((clearance_line_ * 1.414) / resolution_);
    astar2D_path_finder_->init();
    edt_environment_->sdf_map_->getRegion(map_origin_, map_size_);
  }
  std::cout << "[Topo]: start: " << start.transpose() << ", end: " << end.transpose() << "\n";
  // std::cout << "skip_scale_: " << skip_scale_ << std::endl;
  start_pts_ = start_pts;
  end_pts_ = end_pts;
  record_data_.reset();
  
  // 关闭preprocess并清理掉path_container的话，就相当于关闭了TopoPathSet
  // path_container_front_.clear();  
  // path_container_back_.clear(); 
  preprocess();
  // if (!path_container_front_.empty() && !path_container_back_.empty()) 
  //   return;
  double sleep_time = 0.0;
  if(topo_test_) sleep_time = 0.00;
  ros::Duration(sleep_time).sleep();   
  preprocess_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();
  graph = createGraph(start, end);

  graph_time = (ros::Time::now() - t1).toSec();

  /* ---------- search paths in the graph ---------- */
  t1 = ros::Time::now();

  raw_paths = searchPaths();

  search_time = (ros::Time::now() - t1).toSec();

  /* ---------- path shortening ---------- */
  // for parallel, save result in short_paths_
  t1 = ros::Time::now();

  // 在short之后再检查一次同伦。
  // 因为没有short之前，路径可能歪歪扭扭的，用UVD检查的话可能会出问题。
  std::cout << "shortcutPaths start!" << std::endl;
  shortcutPaths();
  std::cout << "shortcutPaths end!" << std::endl;

  short_time = (ros::Time::now() - t1).toSec();

  /* ---------- prune equivalent paths ---------- */
  t1 = ros::Time::now();

  std::cout << "pruneEquivalent start!" << std::endl;
  filtered_paths = pruneEquivalent(short_paths_);
  std::cout << "pruneEquivalent end!" << std::endl;

  prune_time = (ros::Time::now() - t1).toSec();
  // cout << "prune: " << (t2 - t1).toSec() << endl;

  /* ---------- select N shortest paths ---------- */
  t1 = ros::Time::now();

  std::cout << "selectShortPathsV2 start!" << std::endl;
  selectShortPathsV2(filtered_paths);
  std::cout << "selectShortPathsV2 end!" << std::endl;
  // select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (ros::Time::now() - t1).toSec();

  // final_paths_ = select_paths;

  double total_time = preprocess_time + graph_time + search_time + short_time + prune_time + select_time;
  last_success_ = (raw_paths_.size() > 0);
  std::cout << "\n[Topo]: total time: " << total_time << ", preprocess: " << preprocess_time << ", graph: " << graph_time
            << ", search: " << search_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;
  record_data_.run_time = total_time - sleep_time;
  record_data_.preprocess_time = preprocess_time - sleep_time;
  record_data_.path_num = raw_paths_.size();
}

list<GraphNode::Ptr> TopologyPRM::createGraph(Eigen::Vector3d start, Eigen::Vector3d end) {
  std::cout << "[Topo]: searching----------------------" << std::endl;
  // std::cout << "start: " << start.transpose() << ", end: " << end.transpose() << "." << std::endl;
  /* init the start, end and sample region */
  graph_.clear();
  string points_file_name = file_path_ + "fast_planner/plan_manage/maps/sample_points.txt";

  GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // sample region
  // sample_r_(0) = 0.5 * min((end - start).norm(), 7.0) + sample_inflate_(0);
  // sample_r_(1) = 0.3 * min((end - start).norm(), 7.0) + sample_inflate_(1);
  sample_r_(0) = 0.5 * ((end - start).norm()) + sample_inflate_(0);
  sample_r_(1) = 0.3 * ((end - start).norm()) + sample_inflate_(1);
  sample_r_(2) = sample_inflate_(2);

  // transformation
  translation_ = 0.5 * (start + end);

  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  xtf = (end - translation_).normalized();
  ytf = xtf.cross(downward).normalized();
  ztf = xtf.cross(ytf);

  rotation_.col(0) = xtf;
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  creatSampleRegion();
  int node_id = 1;

  /* ---------- main loop ---------- */
  int sample_num = 0;
  double sample_time = 0.0;
  vector<vector<Eigen::Vector2d>> edgeSamplePts;
  // int edge_sample_num = sampleEdgePoints(sample_area_[0], sample_area_[1], sample_area_[2], sample_area_[3], edgeSamplePts);
  int edge_sample_num = 0;
 
  Eigen::Vector3d pt;
  pt.z() = ground_height_;
  ros::Time t1, t2;
  int a, b;
  sampled_points_.clear();
  
  // //计算每一个部分的耗时，看看到底是哪个部分
  // double time_1 = 0.0; double time_2 = 0.0; double time_3 = 0.0;
  static bool isFirstRun = true;
  if(topo_test_ && isFirstRun)  loadPointsFromFile(sampled_points_, points_file_name);
  while (
    sample_time < (last_success_ ? max_sample_time_ : 1.5 * max_sample_time_) && 
    sample_num < max_sample_num_) {
    t1 = ros::Time::now();

    // if(isFirstRun)
    // {
    //   if(sample_num < sampled_points_.size())
    //   {
    //     pt = sampled_points_[sample_num];
    //   }
    //   else
    //   {
    //     pt = getSample();
    //   }
    // }
    // else
    // {
    //   pt = getSample();
    // }
    pt = getSample();
    // pt.z() = ground_height_;
    // 下面这一段是用Edge的采样点。现在不使用这个方法
    // a = sample_num % 5;//mod
    // b = floor(sample_num / 5);
    // if(b < edge_sample_num)
    // {
    //   if(a < 4) 
    //   {
    //     pt.head(2) = edgeSamplePts[a][b];
    //     if(!edt_environment_->sdf_map_->isInMap2D(edgeSamplePts[a][b]))
    //     {
    //       ++sample_num;
    //       continue;
    //     } 
    //   }
    //   else pt = getSample();
    // }
    // else pt = getSample();
    // 上面这一段是用Edge的采样点。现在不使用这个方法

    if(!edt_environment_->sdf_map_->isInMap2D(Eigen::Vector2d(pt.x(), pt.y()), 0.5))
    {
      sample_time += (ros::Time::now() - t1).toSec();
      continue;
    }
    ++sample_num;
    double dist;
    Eigen::Vector3d grad;
    // edt_environment_->evaluateEDTWithGrad(pt, -1.0, dist, grad);
    dist = edt_environment_->evaluateCoarseEDT(pt, -1.0, only2D_);
    if (dist <= clearance_) {
      sample_time += (ros::Time::now() - t1).toSec();
      continue;
    }
    // sampled_points_.emplace_back(pt);
    /* find visible guard */
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    if (visib_guards.size() == 0) {
      GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    } else if (visib_guards.size() == 2) {
      /* try adding new connection between two guard */
      // vector<pair<GraphNode::Ptr, GraphNode::Ptr>> sort_guards =
      // sortVisibGuard(visib_guards);
      bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {
        sample_time += (ros::Time::now() - t1).toSec();
        continue;
      }
      // new useful connection needed, add new connector
      GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
      graph_.push_back(connector);

      // connect guards
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);

      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
    }

    sample_time += (ros::Time::now() - t1).toSec();
  }
  isFirstRun = false;
  /* print record */
  std::cout << "[Topo]: sample num: " << sample_num << ", sample time: " << sample_time * 1000;
  record_data_.sample_num = sample_num;
  pruneGraph();
  // publishGraph(sample_num);
  // savePointsToFile(sampled_points_, points_file_name);
  return graph_;
  // return searchPaths(start_node, end_node);
}

vector<GraphNode::Ptr> TopologyPRM::findVisibGuard(Eigen::Vector3d pt) {
  vector<GraphNode::Ptr> visib_guards;
  Eigen::Vector3d pc;

  int visib_num = 0;

  /* find visible GUARD from pt */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) continue;

    if (lineVisib(pt, (*iter)->pos_, clearance_, pc, 0, -1)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2) break;
    }
  }

  return visib_guards;
}

bool TopologyPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
  vector<Eigen::Vector3d> path1(3), path2(3);
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  vector<Eigen::Vector3d> connect_pts;
  bool has_connect = false;
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        path2[1] = g1->neighbors_[i]->pos_;
        bool same_topo = sameTopoPath(path1, path2, 0.0);
        if (same_topo) {
          // get shorter connection ?
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;
            // ROS_WARN("shorter!");
          }
          return false;
        }
      }
    }
  }
  return true;
}

Eigen::Vector3d TopologyPRM::getSample() {
  /* sampling */
  Eigen::Vector3d pt;
  if(topo_test_)
  {
    pt(0) = rand_pos_(eng_) * map_size_(0) * 0.5 ;
    pt(1) = rand_pos_(eng_) * map_size_(1) * 0.5 ;
    pt(2) = ground_height_;
    return pt;
  }
  pt(0) = rand_pos_(eng_) * sample_r_(0);
  pt(1) = rand_pos_(eng_) * sample_r_(1);
  pt(2) = rand_pos_(eng_) * sample_r_(2);

  pt = rotation_ * pt + translation_;
  pt(2) = ground_height_;
  return pt;
}

// skip_mode为-1，表示都不要earlycheck，用于shortPath()
// skip_mode为0，表示不要skip，用于findVisibGuard()；
// skip_mode为1，用skip_scale降采样
// skip_mode为2，用2*skip_scale降采样，用于sameTopoPath()
bool TopologyPRM::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                            Eigen::Vector3d& pc, int caster_id, int skip_mode) {
  Eigen::Vector3d ray_pt;
  Eigen::Vector3i pt_id;
  Eigen::Vector2i pt_id_2d;
  double dist;
  double skip_scale = 1.0;
  thresh = max(thresh, 1e-3);
  Eigen::Vector3d dir = p2 - p1;
  double dis_pt = dir.norm();
  // if(visLineStep_)
  // {
  //   vector<Eigen::Vector3d> line;
  //   line.emplace_back(p1);
  //   line.emplace_back(p2);
    // publishTestPath(vector<Eigen::Vector3d>{p1, p2}, 3);
  //   int debug = 0;
  // }

  // early check，这个肯定是需要的
  if(skip_mode >= 0 && use_skip_ && (dis_pt < 2 * clearance_line_)) return true;

  // edge_pt只在检查两个点是否free时使用，其他检测线free时不使用
  // 判断这个点是不是edge上的点，如果是，则使用0.5 * thresh来检测距离
  // 这里，我采用了新的skip方式，不再是把起点和终点先缩放，这样会引起很大的误差
  // 遍历raycast的时候，依然是step=1来遍历，只不过在check dis的时候，跳着check
  if(skip_mode == 1) skip_scale = skip_scale_;
  else if(skip_mode == 2) skip_scale = 2 * skip_scale_;
  casters_[caster_id].setInput(p1 / resolution_, p2 / resolution_);

  //TODO: 这一部分可以简化
  int step_num = 1;
  if(skip_mode <= 0) step_num = 1;
  else
  {
    step_num = ceil(skip_scale * (std::abs(dir.x()) + std::abs(dir.y())) / dis_pt);
    step_num = min(max(1, step_num), (int)floor(dis_pt / resolution_));     
  }

  // if(skip_mode > 0 && dis_pt > 0.7) 
  // {
  //   std::cout << step_num << ", " << dis_pt << ", " << skip_scale << std::endl;
  // }
 
  int iter = 0;
  // while会跳过最后一个点
  while (casters_[caster_id].step(ray_pt)) {
    if(skip_mode >= 1 && (iter == 0 || (iter % step_num) != 0)) 
    {
        ++iter;
        continue;
    }
    ++iter;
    // if(iter > 10000) 
    // {
    //   ROS_ERROR_STREAM("Raycast error! start: " << p1.transpose() << ", end: " << p2.transpose());
    //   ros::Duration(5.0).sleep();
    // }
    pt_id_2d(0) = ray_pt(0) + offset_(0);
    pt_id_2d(1) = ray_pt(1) + offset_(1);
    dist = only2D_ ? edt_environment_->sdf_map_->getDistance2D(pt_id_2d) : edt_environment_->sdf_map_->getDistance(pt_id);
    if (dist <= (thresh)) {
      edt_environment_->sdf_map_->indexToPos2D(pt_id_2d, pc);//pc是碰撞点
      pc.z() = ground_height_;
      return false;
    }
  }

  //  这里就是ray_pt_end，不能改
  // Eigen::Vector3d ray_pt_end = ray_pt + offset_;
  // dist = only2D_ ? edt_environment_->sdf_map_->getDistance2D(ray_pt_end) : edt_environment_->sdf_map_->getDistance(ray_pt_end);
  // if (dist <= (thresh)) {
  //   pc = ray_pt_end;
  //   return false;
  // }
  // return true;

  //  这里就是ray_pt_end，不能改
  // TODO，这里是有错的！！
  Eigen::Vector2i end_id;
  end_id(0) = ray_pt(0) + offset_(0);
  end_id(1) = ray_pt(1) + offset_(1);
  dist = only2D_ ? edt_environment_->sdf_map_->getDistance2D(end_id) : edt_environment_->sdf_map_->getDistance(pt_id);
  Eigen::Vector3d pt_tmp;
  edt_environment_->sdf_map_->indexToPos2D(end_id, pt_tmp);
  
  if (dist <= (thresh)) {
    edt_environment_->sdf_map_->indexToPos2D(end_id, pc);
    pc.z() = ground_height_;
    return false;
  }
  return true;
}

void TopologyPRM::pruneGraph() {
  /* prune useless node */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1) continue;

      /* core */
      // std::cout << "id: " << (*iter1)->id_ << std::endl;
      if ((*iter1)->neighbors_.size() <= 1) {
        // delete this node from others' neighbor
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

vector<vector<Eigen::Vector3d>> TopologyPRM::pruneEquivalent(const vector<vector<Eigen::Vector3d>>& paths) {
  vector<vector<Eigen::Vector3d>> pruned_paths;
  if (paths.size() < 1) return pruned_paths;

  /* ---------- prune topo equivalent path ---------- */
  // output: pruned_paths
  vector<int> exist_paths_id;
  // exist_paths_id.push_back(0);

  for (int i = 0; i < paths.size(); ++i) {
    // compare with exsit paths
    bool new_path = true;
    for(const auto& exist_path : path_container_front_)
    {
      bool same_topo = sameTopoPath(paths[i], exist_path.path, 0.0, true);
      if (same_topo) {
        new_path = false;
        break;
      }
    }
    // 如果和现在容器里的对比，都不是new_path，则不必再和其他的对比了。
    if(!new_path) continue;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // compare with one path
      bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0, true);
      // bool same_topo2 = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0, true);
      // if(same_topo != same_topo2)
      // { 
      //   sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0, false);
      //   sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0, true);
      //   int debug = 0;
      // }
      // std::cout << i << ", " << exist_paths_id[j] << ": same? " << same_topo << std::endl; //这里不需要查找表
      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      exist_paths_id.push_back(i);
    }
  }

  // save pruned paths
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  std::cout << ", pruned path num: " << pruned_paths.size();

  return pruned_paths;
}

// 选出前reserve_num条最短路径。每次选出一条就弹出，然后重新选下一条，而不是对整体排序
// 然后把起点和终点接上来，最后再操作一遍缩短、检查同伦等操作
// 在我之前的代码里，把topo_prm/reserve_num设置为了1，即选出最短的一条。
// 这里，没有传入const &，因为paths就是要传出去的filtered_paths，这里就把原filtered_paths里的select_paths剔出去了
// 后面，可视化的时候才不会重复
vector<vector<Eigen::Vector3d>> TopologyPRM::selectShortPaths(vector<vector<Eigen::Vector3d>>& paths,
                                                              int step) {
  /* ---------- only reserve top short path ---------- */
  vector<vector<Eigen::Vector3d>> short_paths;
  vector<Eigen::Vector3d> short_path;
  double min_len;

  // 这里还不如直接把全部都算出来，然后排序呢
  for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i) {
    int path_id = shortestPath(paths);
    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = pathLength(paths[path_id]);
      paths.erase(paths.begin() + path_id);
    } else {
      double rat = pathLength(paths[path_id]) / min_len;
      if (rat < ratio_to_short_) {
        short_paths.push_back(paths[path_id]);
        paths.erase(paths.begin() + path_id);
      } else {
        break;
      }
    }
  }
  std::cout << ", select path num: " << short_paths.size() << std::endl;

  // 我不用添加起点和终点，所以这部分我可以省略
  if(only2D_) return short_paths;
  /* ---------- merge with start and end segment ---------- */
  for (int i = 0; i < short_paths.size(); ++i) {
    short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());
    short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());
  }
  for (int i = 0; i < short_paths.size(); ++i) {
    shortcutPath(short_paths[i], i, 5);
    short_paths[i] = short_paths_[i];
  }

  short_paths = pruneEquivalent(short_paths);

  return short_paths;
}

void TopologyPRM::selectShortPathsV2(const vector<vector<Eigen::Vector3d>>& paths) {
  // 范围[first, last)
  // move是从first开始移动依次移动（从左到右），最后first到达result;
  // move_backward是从last-1开始依次移动（从右到坐），最后last到达result;

  double minLength = std::numeric_limits<double>::max();
  int new_insert_path_count = 0;
  for (const auto& path : paths) {
    double path_length = pathLength(path);
    TopoPath newPath(path, path_length);
    if(!path_container_front_.empty()) minLength = path_container_front_.front().length;

    // 插入到前部分
    if (path_container_front_.size() < path_container_size_half_ ||
        (newPath.length < path_container_front_.back().length && newPath.length < minLength * ratio_to_short_)) {
        auto insertPos = std::lower_bound(path_container_front_.begin(), path_container_front_.end(), newPath);
        path_container_front_.insert(insertPos, newPath);
        ++ new_insert_path_count;
        // 如果前部分超出容量，删除最长路径
        if (path_container_front_.size() > path_container_size_half_) {
            path_container_front_.pop_back();
        }
    }
    // 插入到后部分
    // 这里对ratio_to_short的判断，有可能造成back里面没有路径，得看看会不会发生
    else if (path_container_back_.size() < path_container_size_half_ ||
              (newPath.length > path_container_back_.front().length && newPath.length < minLength * ratio_to_short_)) {
        auto insertPos = std::lower_bound(path_container_back_.begin(), path_container_back_.end(), newPath);
        path_container_back_.insert(insertPos, newPath);

        // 如果后部分超出容量，删除最短路径
        if (path_container_back_.size() > path_container_size_half_) {
            path_container_back_.erase(path_container_back_.begin());
        }
    }
  }

  // std::cout << std::endl;
  // for(int i = 0; i < path_container_front_.size(); ++i) std::cout << path_container_front_[i].length << ", ";
  // std::cout << std::endl;  
  // 检查是否有降序
  if(!isNonDecreasing(path_container_front_)) ROS_ERROR("Wrong path_container_front_!");
  if(!isNonDecreasing(path_container_back_)) ROS_ERROR("Wrong path_container_back_!");
  // if(path_container_front_.back().length > path_container_back_.front().length) ROS_ERROR("Wrong path between front and back!");
  ROS_WARN_STREAM("Front size: " << path_container_front_.size() << ", Back size: " << path_container_back_.size() << 
                  ", New insert path count (close set): " << new_insert_path_count);
  // static int last_size = path_container_front_.size();
  // if(path_container_front_.size() == 1 && last_size > 1) ros::Duration(10).sleep();
  // last_size = path_container_front_.size();
  int path_num = 0;
  int total_paths = std::min(path_container_front_.size() + path_container_back_.size(), (size_t)5);
  for(int i = 0; i < path_container_front_.size() && path_num < total_paths; ++i, ++path_num)
  {
      record_data_.path_lengths[path_num] = path_container_front_[i].length;
  } 
  for(int i = 0; i < path_container_back_.size() && path_num < total_paths; ++i, ++path_num)
  {
      record_data_.path_lengths[path_num] = path_container_back_[i].length;
  }  
}


bool TopologyPRM::sameTopoPath(const vector<Eigen::Vector3d>& path1,
                               const vector<Eigen::Vector3d>& path2, double thresh, bool preprocess) {
  vector<Eigen::Vector3d> newPath1, newPath2;
  if(preprocess)
  {
    int sameStart, sameEnd;
    CommonStartEnd(path1, path2, sameStart, sameEnd);
    if((sameStart + sameEnd) > path1.size() || (sameStart + sameEnd) > path2.size())
    {
      return 1;
    } 
    newPath1.assign(path1.begin() + sameStart, path1.end() - sameEnd);
    newPath2.assign(path2.begin() + sameStart, path2.end() - sameEnd); 
    if(newPath1.size() < 2 || newPath2.size() < 2) return 1;
  }
  else
  {
    newPath1 = path1;
    newPath2 = path2;
  }

  // if(1) publishTestPath(newPath1, 1);
  // if(1) publishTestPath(newPath2, 2);   
  // calc the length
  double len1 = pathLength(newPath1);
  double len2 = pathLength(newPath2);

  double max_len = max(len1, len2);
  double min_len = min(len1, len2);
  if(min_len < resolution_ + 1e-3) return true;
  int pt_num;
  if(use_skip_) pt_num = ceil(max_len / resolution_ / (skip_scale_));
  else pt_num = ceil(max_len / resolution_);

  // std::cout << "pt num: " << pt_num << std::endl;

  vector<Eigen::Vector3d> pts1 = discretizePath(newPath1, pt_num);
  vector<Eigen::Vector3d> pts2 = discretizePath(newPath2, pt_num);

  Eigen::Vector3d pc;
  bool vis;
  int skip_mode = 0;
  if(thresh < 0.0) skip_mode = -1;
  else if(use_skip_) skip_mode = 1; // 这个，如果是2，降采样更加激进，但是有可能会造成错误，把本来是同伦的判断为不是，并一起保留下来
  for (int i = 0.5 * pt_num - 1; i >= 0; --i) {
    // auto t3 = ros::Time::now();  
    // bool vis = lineVisib(pts1[i], pts2[i], thresh, pc);
    // part_time_ += (ros::Time::now() - t3).toSec();
    // if (!vis) {
    //   return false;
    // }

    // TODO: 这里应该排除掉line上的那个点，不然会出错
    vis = lineVisib(pts1[i], 0.5 * (pts1[i] + pts2[i]), skip_mode * clearance_, pc, 0, skip_mode);
    if (!vis)return false;
    vis = lineVisib(pts2[i], 0.5 * (pts1[i] + pts2[i]), skip_mode * clearance_, pc, 0, skip_mode);
    if (!vis)return false;      
  }
  for (int i = 0.5 * pt_num; i < pt_num; ++i) {
    // auto t3 = ros::Time::now();  
    // bool vis = lineVisib(pts1[i], pts2[i], thresh, pc);
    // part_time_ += (ros::Time::now() - t3).toSec();
    // if (!vis) {
    //   return false;
    // }
    // TODO: 这里应该排除掉line上的那个点，不然会出错
    vis = lineVisib(pts1[i], 0.5 * (pts1[i] + pts2[i]), skip_mode * clearance_, pc, 0, skip_mode);
    if (!vis)return false;
    vis = lineVisib(pts2[i], 0.5 * (pts1[i] + pts2[i]), skip_mode * clearance_, pc, 0, skip_mode);
    if (!vis)return false;      
  }

  return true;
}

int TopologyPRM::shortestPath(vector<vector<Eigen::Vector3d>>& paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}
double TopologyPRM::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

vector<Eigen::Vector3d> TopologyPRM::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector3d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;
    if (i == pt_num - 1) {
        dis_path.push_back(path.back());
        continue;
    }
    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // 获取当前段长度
    double segment_length = len_list[idx + 1] - len_list[idx];

    // 如果段长为零，直接选取该点
    if (segment_length < 1e-8) {
        dis_path.push_back(path[idx]);
    } else {
        // 正常插值
        double lambda = (cur_l - len_list[idx]) / segment_length;
        Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
        dis_path.push_back(inter_pt);
    }
  }

  return dis_path;
}

vector<Eigen::Vector3d> TopologyPRM::pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num) {
  return discretizePath(path, pt_num);
}

// 这里iter_num不就是1吗？我这里这个好像没有生效
void TopologyPRM::shortcutPath(const vector<Eigen::Vector3d>& path, int path_id, int iter_num) {
  vector<Eigen::Vector3d> short_path = path;
  vector<Eigen::Vector3d> last_path;
  // if(1) publishTestPath(short_path, 1);
  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

    // 这里离散化后的点之间的距离是resolution
    vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

    if (dis_path.size() < 2) {
      short_paths_[path_id] = dis_path;
      return;
    }

    /* visibility path shortening */
    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());
    for (int i = 1; i < dis_path.size(); ++i) {
      if (lineVisib(short_path.back(), dis_path[i], clearance_, colli_pt, path_id, -1)) continue;
      // 如果此时有一个点和short_path最后一个点碰撞了，则把这个碰撞点外推
      if(only2D_) edt_environment_->evaluateEDTWithGrad2D(colli_pt, -1, dist, grad);
      else edt_environment_->evaluateEDTWithGrad(colli_pt, -1, dist, grad); 
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        colli_pt = colli_pt + resolution_ * 1.3 * push_dir;
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());

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
  // if(1) publishTestPath(short_path, 2);
  short_paths_[path_id] = short_path;
}

// 重载一个给path_container shortcut的
void TopologyPRM::shortcutPath(const int& path_id, const bool& inFront, const int& iter_num) {
  TopoPath& onePath = inFront ? path_container_front_[path_id] : path_container_back_[path_id];
  vector<Eigen::Vector3d> short_path = onePath.path;
  vector<Eigen::Vector3d> last_path;
  // if(1) publishTestPath(short_path, 1);
  for (int k = 0; k < iter_num; ++k) {
    // last_path = short_path;

    // 这里离散化后的点之间的距离是resolution
    vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

    if (dis_path.size() < 2) {
      onePath.path = dis_path;
      return;
    }

    /* visibility path shortening */
    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());
    for (int i = 1; i < dis_path.size(); ++i) {
      // 我这里设置了1.2 * clearance_，就是更加远离障碍物一点。
      // 不知道会不会避免short后路径点，由于亚像素误差依然小于clearance_的情况
      // 这里可能会出现问题，就是在一个很狭窄地地方，反复推，反复推，汇集很多点
      if (lineVisib(short_path.back(), dis_path[i], clearance_, colli_pt, path_id, -1)) continue;
      // 如果此时有一个点和short_path最后一个点碰撞了，则把这个碰撞点外推
      if(only2D_) edt_environment_->evaluateEDTWithGrad2D(colli_pt, -1, dist, grad);
      else edt_environment_->evaluateEDTWithGrad(colli_pt, -1, dist, grad); 
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        colli_pt = colli_pt + resolution_ * push_dir;
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());

    /* break if no shortcut */
    // double len1 = pathLength(last_path);
    // double len2 = pathLength(short_path);
    // if (len2 > len1) {
    //   // ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k +
    //   // 1);
    //   short_path = last_path;
    //   break;
    // }
  }
  // if(1) publishTestPath(short_path, 2);
  onePath.path = short_path;
}

void TopologyPRM::shortcutPaths() {
  short_paths_.resize(raw_paths_.size());

  // if (0) {
  if (parallel_shortcut_) {
    // vector<thread> short_threads;
    // for (int i = 0; i < raw_paths_.size(); ++i) {
    //   short_threads.push_back(thread(&TopologyPRM::shortcutPath, this, raw_paths_[i], i, 2));
    // }
    // for (int i = 0; i < raw_paths_.size(); ++i) {
    //   short_threads[i].join();
    // }

    std::vector<std::future<void>> results;
    for(int i = 0; i < raw_paths_.size(); ++i)
    {
      results.emplace_back(threadPool_->enqueue([this, i]{
                         shortcutPath(this->raw_paths_[i], i, 2);}));
    }
    for(auto& result : results)
    {
      result.get();
    }
  } else {
    for (int i = 0; i < raw_paths_.size(); ++i) shortcutPath(raw_paths_[i], i);
  }
}

vector<Eigen::Vector3d> TopologyPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  Eigen::Vector3d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);

  vector<Eigen::Vector3d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  for (int i = 0; i <= seg_num; ++i) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

vector<Eigen::Vector3d> TopologyPRM::discretizePath(const vector<Eigen::Vector3d>& path) {
  vector<Eigen::Vector3d> dis_path, segment;

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

vector<vector<Eigen::Vector3d>> TopologyPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& path) {
  vector<vector<Eigen::Vector3d>> dis_paths;
  vector<Eigen::Vector3d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.push_back(dis_path);
  }

  return dis_paths;
}

Eigen::Vector3d TopologyPRM::getOrthoPoint(const vector<Eigen::Vector3d>& path) {
  Eigen::Vector3d x1 = path.front();
  Eigen::Vector3d x2 = path.back();

  Eigen::Vector3d dir = (x2 - x1).normalized();
  Eigen::Vector3d mid = 0.5 * (x1 + x2);

  double min_cos = 1000.0;
  Eigen::Vector3d pdir;
  Eigen::Vector3d ortho_pt;

  for (int i = 1; i < path.size() - 1; ++i) {
    pdir = (path[i] - mid).normalized();
    double cos = fabs(pdir.dot(dir));

    if (cos < min_cos) {
      min_cos = cos;
      ortho_pt = path[i];
    }
  }

  return ortho_pt;
}

// search for useful path in the topo graph by DFS
// 用DFS搜索出最大max_1条路径，然后按节点数量升序排列，再选出前max_2个
vector<vector<Eigen::Vector3d>> TopologyPRM::searchPaths() {
  raw_paths_.clear();
  // //XXXX  xxx
  // vector<GraphNode::Ptr> visited;
  // visited.push_back(graph_.front());
  // depthFirstSearch(visited);
  // ROS_WARN_STREAM("raw path by DFS: " << raw_paths_.size()); 

  raw_paths_.clear();  
  unique_paths_.clear();
  unique_paths_.resize(path_node_num_max_);
  vector<GraphNode::Ptr> vis1, vis2;
  auto start_it = graph_.begin();
  vis1.push_back(*start_it);
  vis2.push_back(*std::next(start_it));
  depthFirstSearchBid(vis1, vis2, 0);
  ROS_WARN_STREAM("raw path by BiDFS: " << raw_paths_.size()); 

  // sort the path by node number
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num) max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num) min_node_num = raw_paths_[i].size();
    path_list[int(raw_paths_[i].size())].push_back(i);
  }

  // select paths with less nodes
  vector<vector<Eigen::Vector3d>> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path2_) {
        reach_max = true;
        break;
      }
    }
    if (reach_max) break;
  }
  std::cout << ", raw path num: " << raw_paths_.size() << ", " << filter_raw_paths.size();

  raw_paths_ = filter_raw_paths;

  return raw_paths_;
}

void TopologyPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
  GraphNode::Ptr cur = vis.back();

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // check reach goal
    if (cur->neighbors_[i]->id_ == 1) {
      // add this path to paths set
      vector<Eigen::Vector3d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_) return;

      break;
    }
  }

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // skip reach goal
    if (cur->neighbors_[i]->id_ == 1) continue;

    // skip already visited node
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit) continue;

    // recursive search
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_) return;

    vis.pop_back();
  }
}


void TopologyPRM::depthFirstSearchBid(vector<GraphNode::Ptr>& vis1, vector<GraphNode::Ptr>& vis2, 
                                      const bool& dirState) 
{
  // 切换不同的迭代分支. 0代表从起点来的，1代表从终点来的
  vector<GraphNode::Ptr>& vis = (dirState == 0) ? vis1 : vis2;
  vector<GraphNode::Ptr>& vis_other = (dirState == 0) ? vis2 : vis1;
  GraphNode::Ptr cur = vis.back(); 
  if(vis.size() >= path_node_num_max_ || vis_other.size() >= path_node_num_max_)
  {
    return;
  }

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    int curr_id = cur->neighbors_[i]->id_;
    auto vis_iter = std::find_if(vis_other.begin(), vis_other.end(),
                                [&curr_id](const GraphNode::Ptr& node){return node->id_ == curr_id;
                                });    
    // check reach goal
    // 找到了
    if (vis_iter != vis_other.end()) {
      // add this path to paths set
      vector<Eigen::Vector3d> path;
      vector<int> path_ids;
      if(dirState == 0) {// 从起点来的，先将vis顺序放入，然后将vis_iter反向接到后面
        for(int j = 0; j < vis.size(); ++j)
        {
          path.emplace_back(vis[j]->pos_);
          path_ids.emplace_back(vis[j]->id_);
        }
        for (auto it = vis_iter; it != vis_other.begin(); --it) {
          path.emplace_back((*it)->pos_); 
          path_ids.emplace_back((*it)->id_);      
        }
        path.emplace_back(vis_other.front()->pos_);
      } else {           // vis从终点来的，vis_iter是从起点来的，先顺序放入，然后将vis反向接到后面
        for (auto it = vis_other.begin(); it != vis_iter + 1; ++it) {
          path.emplace_back((*it)->pos_);   
          path_ids.emplace_back((*it)->id_);    
        }
        for(int j = vis.size() - 1; j >= 0; --j)
        {
          path.emplace_back(vis[j]->pos_);
          path_ids.emplace_back(vis[j]->id_);
        }
      }
      int node_size = path_ids.size();
      if(node_size < path_node_num_max_ && 
         unique_paths_[node_size].find(path_ids) == unique_paths_[node_size].end())
      {
        unique_paths_[node_size].insert(path_ids);
        raw_paths_.emplace_back(path);
        // publishTestPath(raw_paths_[raw_paths_.size() - 1], 3);   
      }

      if (raw_paths_.size() >= max_raw_path_) return;
      break;
    }
  }

  // 扩展搜索路径
  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    int curr_id = cur->neighbors_[i]->id_;
    auto vis_iter = std::find_if(vis_other.begin(), vis_other.end(),
                                [&curr_id](const GraphNode::Ptr& node){return node->id_ == curr_id;
                                });
    // skip reach goal
    if (vis_iter != vis_other.end()) continue;

    // skip already visited node
    bool revisit = std::find_if(vis.begin(), vis.end(),
                  [&curr_id](const GraphNode::Ptr& node){return node->id_ == curr_id;
                  }) != vis.end();
    //如果是访问过的，就跳过
    if (revisit) continue;

    // recursive search
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearchBid(vis1, vis2, !dirState);
    if (raw_paths_.size() >= max_raw_path_) return;
    vis.pop_back();
  }
}


void TopologyPRM::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }

bool TopologyPRM::triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2) {
  // get the traversing points along p1-p2
  vector<Eigen::Vector3d> pts;

  Eigen::Vector3d dir = p2 - p1;
  double length = dir.norm();
  int seg_num = ceil(length / resolution_);

  Eigen::Vector3d pt1;
  for (int i = 1; i < seg_num; ++i) {
    pt1 = p1 + dir * double(i) / double(seg_num);
    pts.push_back(pt1);
  }

  // test visibility
  for (int i = 0; i < pts.size(); ++i) {
    {
      return false;
    }
  }

  return true;
}

void TopologyPRM::publishTestPath(const vector<Eigen::Vector3d>& path, const int& pub_num)
{
  nav_msgs::Path path_tmp;
  path_tmp.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose_stamped;  
  if(topo_test_) 
  {
    path_tmp.header.frame_id = "map";    
    pose_stamped.header.frame_id = "map"; 
  }
  else
  {
    path_tmp.header.frame_id = "world";    
    pose_stamped.header.frame_id = "world"; 
  }

  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose.orientation.w = 1.0;   
  pose_stamped.pose.position.z = ground_height_;                        
  for (const auto& pt : path) 
  {
      pose_stamped.pose.position.x = pt.x();
      pose_stamped.pose.position.y = pt.y();
      path_tmp.poses.push_back(pose_stamped);
  }
  if(pub_num == 1)
    path_1_pub_.publish(path_tmp);
  else if(pub_num == 2)
    path_2_pub_.publish(path_tmp);
  else 
    path_3_pub_.publish(path_tmp);
  ros::Duration(0.0001).sleep();
}

// 这个是专门给画traj的esdf值用的
void TopologyPRM::publishTestPath(const vector<Eigen::Vector2i>& path, const int& pub_num)
{
  nav_msgs::Path path_tmp;
  path_tmp.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose_stamped;  
  if(topo_test_) 
  {
    path_tmp.header.frame_id = "map";    
    pose_stamped.header.frame_id = "map"; 
  }
  else
  {
    path_tmp.header.frame_id = "world";    
    pose_stamped.header.frame_id = "world"; 
  }

  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose.orientation.w = 1.0;   
  pose_stamped.pose.position.z = 0.7;  
  Eigen::Vector2d pt_d;                      
  for (const auto& pt : path) 
  {
      edt_environment_->sdf_map_->indexToPos2D(pt, pt_d);
      pose_stamped.pose.position.x = pt_d.x();
      pose_stamped.pose.position.y = pt_d.y();
      path_tmp.poses.push_back(pose_stamped);
  }
  if(pub_num == 1)
    path_1_pub_.publish(path_tmp);
  else if(pub_num == 2)
    path_2_pub_.publish(path_tmp);
  else 
    path_3_pub_.publish(path_tmp);
  ros::Duration(0.0001).sleep();
}

void TopologyPRM::publishGraph(int sample_num)
{
  visualization_msgs::Marker mk3;
  mk3.header.frame_id = "world";
  mk3.header.stamp    = ros::Time::now();
  mk3.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk3.action          = visualization_msgs::Marker::DELETE;
  mk3.id              = 3;
  grah_vis_pub_.publish(mk3);

  mk3.action             = visualization_msgs::Marker::ADD;
  mk3.pose.orientation.x = 0.0;
  mk3.pose.orientation.y = 0.0;
  mk3.pose.orientation.z = 0.0;
  mk3.pose.orientation.w = 1.0;

  mk3.scale.x = 0.2;
  mk3.scale.y = 0.2;
  mk3.scale.z = 0.2;

  mk3.color.r = 1;
  mk3.color.g = 1;
  mk3.color.b = 0.1;
  mk3.color.a = 0.8;

  geometry_msgs::Point pt;
  for(int i = 0; i < sampled_points_.size(); ++i)
  {
    pt.x = sampled_points_[i](0);
    pt.y = sampled_points_[i](1);
    pt.z = ground_height_;
    mk3.points.push_back(pt);
  }
  grah_vis_pub_.publish(mk3);

  ros::Duration(0.0001).sleep();
}

void TopologyPRM::creatSampleRegion()
{
  sample_area_.resize(0);
  Eigen::Vector3d pt;
  pt(0) = 1 * sample_r_(0);
  pt(1) = 1 * sample_r_(1);
  pt(2) = ground_height_;
  pt = rotation_ * pt + translation_;
  pt(2) = ground_height_;
  sample_area_.push_back(pt);

  pt(0) = 1 * sample_r_(0);
  pt(1) = -1 * sample_r_(1);
  pt(2) = ground_height_;
  pt = rotation_ * pt + translation_;
  pt(2) = ground_height_;
  sample_area_.push_back(pt);

  pt(0) = -1 * sample_r_(0);
  pt(1) = -1 * sample_r_(1);
  pt(2) = ground_height_;
  pt = rotation_ * pt + translation_;
  pt(2) = ground_height_;
  sample_area_.push_back(pt);

  pt(0) = -1 * sample_r_(0);
  pt(1) = 1 * sample_r_(1);
  pt(2) = ground_height_;
  pt = rotation_ * pt + translation_;
  pt(2) = ground_height_;
  sample_area_.push_back(pt); 
}

void TopologyPRM::generatePoints(const Eigen::Vector2d& pt, 
                    std::vector<Eigen::Vector2d>& points,
                    double l) {
    const int numPoints = 8;
    const double angleStep = M_PI / 4; // 45度转换为弧度

    for (int i = 0; i < numPoints; ++i) {
        double angle = i * angleStep;
        double dx = l * cos(angle);
        double dy = l * sin(angle);
        points.push_back(Eigen::Vector2d(pt.x() + dx, pt.y() + dy));
    }
}

void TopologyPRM::sampleEdge(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                             const int& numSample, const double& numSample_inv,
                             std::vector<Eigen::Vector2d>& samples) {
    double dx = end.x() - start.x();
    double dy = end.y() - start.y();
    for (int i = 0; i < numSample; ++i) {
        double t = i * numSample_inv;
        Eigen::Vector2d pt_tmp(start.x() + t * dx, start.y() + t * dy);
        double dis_tmp = this->edt_environment_->sdf_map_->getDistance2D(pt_tmp);
        if (dis_tmp < this->clearance_) {
            this->generatePoints(pt_tmp, samples, 4 * abs(dis_tmp));
        } else {
            samples.emplace_back(pt_tmp);
        }
    }
}


int TopologyPRM::sampleEdgePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                   const Eigen::Vector3d& p3, const Eigen::Vector3d& p4,
                                   vector<vector<Eigen::Vector2d>>& samples) 
{
    
    samples.resize(4);
    for(int i = 0; i < 4; ++i) samples[i].reserve(numSamples_ * 4);

    const double numSample_inv = 1.0/numSamples_;
    double dx, dy;
    sampleEdge(p1, p2, numSamples_, numSample_inv, samples[0]);
    sampleEdge(p2, p3, numSamples_, numSample_inv, samples[1]);
    sampleEdge(p3, p4, numSamples_, numSample_inv, samples[2]);
    sampleEdge(p4, p1, numSamples_, numSample_inv, samples[3]);

    int minSize = samples[0].size();
    for(const auto& sample : samples)
    {
      minSize = std::min(minSize, (int)sample.size());
    }

    return minSize;

}

void TopologyPRM::CommonStartEnd(const std::vector<Eigen::Vector3d>& vec1, 
                                 const std::vector<Eigen::Vector3d>& vec2,
                                 int& sameStart, int& sameEnd) {
    // 找到两个vector开头和结尾部分相同的最长前缀和后缀
    int start = 0;
    int end = 0;
    
    // 找出前缀相同的部分
    while (start < vec1.size() && start < vec2.size() && (vec1[start].head(2) - vec2[start].head(2)).squaredNorm() < 4e-2) {
        ++start;
    }
    
    // 找出后缀相同的部分
    while (end < vec1.size() && end < vec2.size() 
           && (vec1[vec1.size() - 1 - end].head(2) - vec2[vec2.size() - 1 - end].head(2)).squaredNorm() < 4e-2) {
        ++end;
    }
    sameStart = max(start-1, 0);
    sameEnd = max(end-1, 0);
}

// 从path_container里面选择shot的路径 
vector<Eigen::Vector3d> TopologyPRM::findDubinsShots(const Eigen::Vector3d& start_state,
                        const double& radius) {
  if(path_container_front_.empty() && path_container_back_.empty()) return {};
  
  dubins_shot_paths_.resize(path_container_front_.size());
  dubins_shot_succ_.resize(path_container_front_.size());
  // 这里开不开好像耗时都是差不多的啊。不知道这个线程池会不会有什么问题啊
  // if (0) {
  if (parallel_shortcut_) {
    std::vector<std::future<void>> results;
    for(int i = 0; i < path_container_front_.size(); ++i)
    {
      results.emplace_back(threadPool_->enqueue([this, i, &start_state, radius]{
                         findDubinsShot(path_container_front_[i].path, i, start_state, radius);}));
    }
    for(auto& result : results)
    {
      result.get();
    }
  } else {
    for (int i = 0; i < path_container_front_.size(); ++i) 
      findDubinsShot(path_container_front_[i].path, i, start_state, radius);
  }

  int minPathLen = 10000;
  int minPathIdx = -1;
  for(int i = 0; i < dubins_shot_paths_.size(); ++i)
  {
    if(dubins_shot_succ_[i] && dubins_shot_paths_[i].size() < minPathLen)
    {
      minPathLen = dubins_shot_paths_[i].size();
      minPathIdx = i;
    }
  }
  // 说明此时，在front里面没有找到dubins_shot成功的，所以在去back里面找
  if(minPathIdx < 0)
  {  
    ROS_WARN_STREAM("The start pt esdf is " << edt_environment_->sdf_map_->getDistance2D(start_state));
    ROS_ERROR("Can not find dubins shot in path_container_front_!");
    dubins_shot_paths_.resize(path_container_back_.size());
    dubins_shot_succ_.resize(path_container_back_.size());
    for(int i = 0; i < path_container_back_.size(); ++i)
    {
      findDubinsShot(path_container_back_[i].path, i, start_state, radius);
      if(dubins_shot_succ_[i])
      {
        minPathIdx = i;
        break;
        ROS_ERROR("But find dubins shot in path_container_back_!");
      }
    }
  }
  if(minPathIdx < 0)
  {
    ROS_ERROR("Also Can not find dubins shot in path_container_back_! ERROR!");  
    minPathIdx = 0;  
    return path_container_front_[0].path;
  }

  // publishTestPath(dubins_shot_paths_[minPathIdx], 1); 
  return dubins_shot_paths_[minPathIdx];
}


void TopologyPRM::findDubinsShot(const vector<Eigen::Vector3d>& path, const int& path_id,
                                 const Eigen::Vector3d& start_state,
                                 const double& radius)
{
  // publishTestPath(path, 1);  
  dubins_shot_paths_[path_id].resize(0);
  int sample_resolution = 5; 
  int sample_num = 5;
  vector<Eigen::Vector3d> dis_path = discretizePath(path);
  vector<Eigen::Vector3d> dubins_path; 
  DubinsPath::DubinsPath dubinsPath;   
  double path_length_best = 10000;
  int i_best = 0;
  for(int i = sample_resolution; i < dis_path.size() && i <= sample_num * sample_resolution; i += sample_resolution)
  {
    dubins_path.resize(0);
    Eigen::Vector3d end_pt = dis_path[i];
    Eigen::Vector3d end_pt_pre = dis_path[i - 1];
    double end_yaw = atan2(end_pt.y() - end_pt_pre.y(), end_pt.x() - end_pt_pre.x());
    double q0[] = { start_state(0), start_state(1), start_state(2) };
    double q1[] = { end_pt(0),   end_pt(1),   end_yaw };
    dubins_init(q0, q1, radius, &dubinsPath); 
    
    double x = resolution_;
    double move_step_size = resolution_;
    double dubins_length = dubins_path_length(&dubinsPath);
    double path_length = dubins_length + (dis_path.size() - i) * resolution_;
    bool isValid = true;
    while (x <  dubins_length){ // 如果关闭dubinsShot时的碰撞检测呢？
      double q[3];
      dubins_path_sample(&dubinsPath, x, q);
      x += move_step_size;      
      Eigen::Vector3d path_pt(q[0], q[1], ground_height_);
      // if(edt_environment_->sdf_map_->getDistance2D(path_pt) <= 0.5 * clearance_)
      // {
      //   isValid = false;
      //   break;
      // } 
      dubins_path.emplace_back(path_pt);
    }
    // publishTestPath(dubins_path, 2);    
    if(!isValid)  continue;
    if(path_length < path_length_best)
    {
      path_length_best = path_length;
      std::swap(dubins_shot_paths_[path_id], dubins_path);
      i_best = i;
    }
  }
  if(dubins_shot_paths_[path_id].empty()) 
  {
    dubins_shot_paths_[path_id] = dis_path;
    dubins_shot_succ_[path_id] = false;
  }
  else
  {
    // std::cout << "Find Dubins Shot!" << std::endl;
    dubins_shot_succ_[path_id] = true;
    dubins_shot_paths_[path_id].insert(dubins_shot_paths_[path_id].end(), dis_path.begin() + i_best + 1, dis_path.end());    
  }
  // publishTestPath(dubins_shot_paths_[path_id], 2);
  int debug = 0;
}

// label = 0，全部， 1， front, 2，back
vector<vector<Eigen::Vector3d>> TopologyPRM::getPathContainer(const int& label)
{
  std::vector<std::vector<Eigen::Vector3d>> paths;
  paths.reserve(path_container_front_.size() + path_container_back_.size());
  if(label == 0 || label == 1)
  {
    for(const auto& topopath : path_container_front_)
    {
      paths.emplace_back(topopath.path);
    }    
  }
  if(label == 0 || label == 2)
  {
    for(const auto& topopath : path_container_back_)
    {
      paths.emplace_back(topopath.path);
    }        
  }

  return paths;
}


// 从path的终点回溯dis距离，返回新的路径
// 这是为了让path的终点不要离障碍物太近
// 但是这里会不会造成死循环？
vector<Eigen::Vector3d> TopologyPRM::backtrackFromEnd(const std::vector<Eigen::Vector3d>& path, double dis) 
{
    if (path.empty()) {
        throw std::invalid_argument("Path cannot be empty");
    }

    double accumulatedDistance = 0.0;
    size_t n = path.size();

    // 从终点开始回溯
    for (size_t i = n - 1; i > 0; --i) {
        double segmentDistance = (path[i] - path[i - 1]).norm();
        accumulatedDistance += segmentDistance;

        if (accumulatedDistance >= dis) {
            // 距离超出，计算回溯点的位置
            double overshoot = accumulatedDistance - dis;
            Eigen::Vector3d direction = (path[i] - path[i - 1]).normalized();
            Eigen::Vector3d backtrackPoint = path[i - 1] + direction * overshoot;

            // 检查 ESDF 值，调整 backtrackPoint
            int iter = 0;
            while (edt_environment_->sdf_map_->getDistance2D(backtrackPoint) < 1.2 * clearance_ && iter < 5) {
                backtrackPoint -= direction * resolution_;
                ++ iter;
            }
            // 创建新路径，从回溯点到终点
            std::vector<Eigen::Vector3d> newPath;
            // 当前的backtrackPoint位于[i-1, i]区间，i-1这个点要包含进来
            newPath.insert(newPath.end(), path.begin(), path.begin() + i);
            newPath.emplace_back(backtrackPoint);
            return newPath;
        }
    }

    // 如果累积距离仍小于 dis，返回空路径
    return {};
}


vector<Eigen::Vector3d> TopologyPRM::backtrackFromStart(const std::vector<Eigen::Vector3d>& path, double dis) 
{
    if (path.empty()) {
        throw std::invalid_argument("Path cannot be empty");
    }

    double accumulatedDistance = 0.0;

    // 从起点开始回溯
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double segmentDistance = (path[i + 1] - path[i]).norm();
        accumulatedDistance += segmentDistance;

        if (accumulatedDistance >= dis) {
            // 距离超出，计算回溯点的位置
            double overshoot = accumulatedDistance - dis;
            Eigen::Vector3d direction = (path[i + 1] - path[i]).normalized();
            Eigen::Vector3d backtrackPoint = path[i + 1] - direction * overshoot;

            // 检查 ESDF 值，调整 backtrackPoint
            int iter = 0;
            while (edt_environment_->sdf_map_->getDistance2D(backtrackPoint) < 1.2 * clearance_ && iter < 5) {
                backtrackPoint += direction * resolution_;
                ++ iter;
            }

            // 创建新路径，从起点到回溯点
            std::vector<Eigen::Vector3d> newPath;
            newPath.emplace_back(backtrackPoint);    
            // 当前的backtrackPoint位于[i, i+1]区间，i+1这个点要包含进来        
            newPath.insert(newPath.end(), path.begin() + i + 1, path.end());
            return newPath;
        }
    }

    // 如果累积距离仍小于 dis，返回空路径
    return {};
}

// 根据笔记内容，来对path_container进行预处理
void TopologyPRM::preprocess()
{
  // 1. 检查所有路径的碰撞情况，标记碰撞的路径，并保留起点和终点连接的部分；标记碰撞区域
  checkPathContainerObstacle();
  // 2. 仿造ego-palnner，用A星对break的topo path重连
  reconnectTopoPaths();
  // 3. 更新起点段，直接把新起点的vector<Eigen::Vector3d>接到path前面。如果path已经失效了，则跳过？。
  // 4. short所有的路径。然后丢弃ratio太大的，和已经失效的。所有路径根据short后的结果sort
  updateAllPaths();

}

void TopologyPRM::checkPathContainerObstacle()
{
  for(int i = 0; i < path_container_front_.size(); ++i)
  {
    checkPathObstacle(i, true);
  }
  // 对于远组，应该也是差不多的吧
  for(int i = 0; i < path_container_back_.size(); ++i)
  {
    checkPathObstacle(i, false);
  }
}

// 我觉得这里，不如直接将路径离散化，然后判断每一个路径点的esdf值是否安全。
void TopologyPRM::checkPathObstacle(const int& path_id, const bool& inFront)
{
  TopoPath& onePath = inFront ? path_container_front_[path_id] : path_container_back_[path_id]; 
  // publishTestPath(onePath.path, 3); 
  onePath.path_break.first.resize(0);
  onePath.path_break.second.resize(0);
  Eigen::Vector3d colli_pt, break_pt;
  const double dis_to_obs = 0.4;
  const double dis_backtrack = dis_to_obs - 0.5 * clearance_;
  int break_idx = 0;
  for(; break_idx < onePath.path.size() - 1; ++break_idx)
  {
    if(!lineVisib(onePath.path[break_idx], onePath.path[break_idx + 1], 0.5 * clearance_, colli_pt, 0, -1))
    {
      // std::cout << "前半段碰撞区间：" << onePath.path[break_idx].transpose() << ", " << onePath.path[break_idx + 1].transpose() << "\n";
      onePath.safty = false;
      // 这里，我需要手动回溯dis_backtrack找到break_pt;
      vector<Eigen::Vector3d> path_break;
      path_break.insert(path_break.end(), onePath.path.begin(), onePath.path.begin() + break_idx + 1);
      path_break.emplace_back(colli_pt);
       
      onePath.path_break.first = backtrackFromEnd(path_break, dis_backtrack);
      if(onePath.path_break.first.empty()) onePath.path_break.first.emplace_back(onePath.path.front());

      // std::cout << "front: \n";
      // for(const auto& pt : onePath.path_break.first) std::cout << pt.transpose() << std::endl;  
      // publishTestPath(onePath.path_break.first, 1);   

      // 下面反向进行一次g
      bool safty = true;      
      int break_idx2 = onePath.path.size() - 1;
      path_break.resize(0);
      for(; break_idx2 > 0; --break_idx2)
      {
        // std::cout << onePath.path[break_idx2].transpose() << std::endl;
        // 这里，不能调换终点来检查lineVisib，调换会导致遍历出来地点不一样，有0.1的偏移
        if(!lineVisib(onePath.path[break_idx2 - 1], onePath.path[break_idx2], 0.5 * clearance_, colli_pt, 0, -1))
        {
          path_break.emplace_back(colli_pt);
          path_break.insert(path_break.end(), onePath.path.begin() + break_idx2, onePath.path.end());
          onePath.path_break.second = backtrackFromStart(path_break, dis_backtrack);          
          // std::cout << "back: \n";
          // for(const auto& pt : onePath.path_break.second) std::cout << pt.transpose() << std::endl;
          // publishTestPath(onePath.path_break.second, 2);    
          safty = false;
          break;
        }
      }
      // if(safty) 
      // {
        // publishTestPath(onePath.path, 3); 
        // publishTestPath(onePath.path_break.first, 1);
        // publishTestPath(onePath.path_break.second, 2);
        // ROS_ERROR("Path from start collided, but the one from end not!!");
        // ros::Duration(10).sleep();
      // }
      break;
    }
  }
  if(!onePath.safty && inFront)
  {
    ROS_WARN_STREAM("Path in " << (inFront ? "Front" : "Back") << " Collid! Id: " << path_id << ".");
  }

  int debug = 0;
}



void TopologyPRM::reconnectTopoPaths()
{
  for(int i = 0; i < path_container_front_.size(); ++i)
  {
    if(path_container_front_[i].safty) continue;
    if(path_container_front_[i].path_break.first.empty() || path_container_front_[i].path_break.second.empty())
    {
      // if(path_container_front_[i].path_break.first.empty()) 
      //   std::cout << "Will be erase beause 前面半段 is empty!\n";
      // else 
      //   publishTestPath(path_container_front_[i].path_break.first, 1);
      // if(path_container_front_[i].path_break.second.empty()) 
      //   std::cout << "Will be erase beause 后面半段 is empty!\n";
      // else 
      //   publishTestPath(path_container_front_[i].path_break.second, 2);
      continue;
    }
      
    Eigen::Vector3d break_start = path_container_front_[i].path_break.first.back();
    Eigen::Vector3d break_end = path_container_front_[i].path_break.second.front();
    double connect_dis_square = (break_end - break_start).norm();
    if(connect_dis_square > 10)  // 大于4m，直接就不要了
    {
      // publishTestPath(path_container_front_[i].path_break.first, 1);
      // publishTestPath(path_container_front_[i].path_break.second, 2);
      // std::cout << "2222222????" << std::endl;      
      // ros::Duration(10).sleep();
      continue;
    }
    // if(edt_environment_->evaluateCoarseEDT(break_start, -1, 1) <= 0.3 ||
    //    edt_environment_->evaluateCoarseEDT(break_end, -1, 1) <= 0.3)
    // {
    //   // publishTestPath(path_container_front_[i].path_break.first, 1);
    //   // publishTestPath(path_container_front_[i].path_break.second, 2);
    //   // publishTestPath(path_container_front_[i].path, 3);
    //   // ROS_WARN_STREAM("Front idx: " << i << ", the value of esdf: " << edt_environment_->evaluateCoarseEDT(break_start, -1, 1)
    //   //     << ", " << edt_environment_->evaluateCoarseEDT(break_end, -1, 1));
    //   // int debug = 0;
    // }
    reconnectBreakPath(i, true);
  }
  // 对于远组，应该也是差不多的吧
  for(int i = 0; i < path_container_back_.size(); ++i)
  {
    if(path_container_back_[i].safty) continue;
    if(path_container_back_[i].path_break.first.empty() || path_container_back_[i].path_break.second.empty())
      continue;
    Eigen::Vector3d break_start = path_container_back_[i].path_break.first.back();
    Eigen::Vector3d break_end = path_container_back_[i].path_break.second.front();
    double connect_dis_square = (break_end - break_start).norm();
    if(connect_dis_square > 10) continue; // 大于4m，直接就不要了

    // if(edt_environment_->evaluateCoarseEDT(break_start, -1, 1) <= 0.3 ||
    //    edt_environment_->evaluateCoarseEDT(break_end, -1, 1) <= 0.3)
    // {
    //   // publishTestPath(path_container_back_[i].path_break.first, 1);
    //   // publishTestPath(path_container_back_[i].path_break.second, 2);
    //   // publishTestPath(path_container_back_[i].path, 3);
    //   // ROS_WARN_STREAM("Back idx: " << i << ", the value of esdf: " << edt_environment_->evaluateCoarseEDT(break_start, -1, 1)
    //   // << ", " << edt_environment_->evaluateCoarseEDT(break_end, -1, 1));
    //   // int debug = 0;
    // }
    reconnectBreakPath(i, false);
  }
}

void TopologyPRM::reconnectBreakPath(const int& path_id, const bool& inFront)
{
  TopoPath& onePath = inFront ? path_container_front_[path_id] : path_container_back_[path_id]; 
  if(onePath.path_break.first.empty() || onePath.path_break.second.empty()) return;
  astar2D_path_finder_->reset();
  int search_result = astar2D_path_finder_->search(onePath.path_break.first.back(), 
                                                   onePath.path_break.second.front(), 0.8 * clearance_);
  if(search_result == 1) // REACH_END
  {
    vector<Eigen::Vector3d> connectPath = astar2D_path_finder_->getPath();
    onePath.safty = true;
    onePath.path.resize(0);
    onePath.path.insert(onePath.path.end(), onePath.path_break.first.begin(), onePath.path_break.first.end());
    onePath.path.insert(onePath.path.end(), connectPath.begin(), connectPath.end());
    onePath.path.insert(onePath.path.end(), onePath.path_break.second.begin(), onePath.path_break.second.end());

    // publishTestPath(onePath.path_break.first, 1);
    // publishTestPath(onePath.path_break.second, 2);
    // publishTestPath(onePath.path, 3);

    int debug = 0;
  } else{
    ROS_WARN_STREAM("Path in " << (inFront ? "Front" : "Back") << " Failed to Reconnect! Id: " << path_id << ".");
    // publishTestPath(onePath.path_break.first, 1);
    // publishTestPath(onePath.path_break.second, 2);
    // ros::Duration(10).sleep();
  }

}

void TopologyPRM::setStartChange(vector<Eigen::Vector3d>& start_change)
{
  // start_change_.swap(start_change);
  start_change_.resize(start_change.size());
  // 使用std::copy和std::reverse_iterator将start_change中的元素逆序复制到start_change_
  // std::copy(start_change.rbegin(), start_change.rend(), start_change_.begin());
  std::reverse_copy(start_change.begin(), start_change.end(), start_change_.begin());
}



void TopologyPRM::resetPathContainer()
{
  path_container_back_.resize(0);
  path_container_front_.resize(0);
  ROS_WARN("Reset Path Container because goal changed!");
}

void TopologyPRM::updateAllPaths()
{
  // (1)把失效的删除。如果近组全失效，就把远组提到前面去(用swap)
  // (2)short路径。【并行化有点问题】。
  //    检查剩下的路径是不是全部点都大于clearence_ = 0.3. 由于亚像素误差，实际取到的值为0.282843，不知道可能有什么问题
  // (3)重新计算path length,两个组重新排序。删除ratio太大的，近组远组都要删除，但是远组至少保留一个。
  // (3.5)当前正在执行的这条一定要保留下来，除非其无效了【【【不一定非要把这个保留下来吧。先不保留】】】
  // 对于这个保留的逻辑。首先，在findDubinsShots时，在path_container里面选择，然后记录下path_id和inFront
  // findDubinsShots里完成【如果它是在远组，就将其放到近组里面去。如果近组不足5个，就放到后面，如果近组等于5个，就把近组最后一个替换】
  // findDubinsShots里完成【要将这个在远组的前面的路径给删除，用于确保近组->远组是非降序的】
  // (4)只检查近组的同伦。在后续正常采样时，也只和近组的相比较同伦，远组的不用。这里，我就对近组和远组进行了区分
  // (5)后面新添加的时候，也不能大于ratio_to_short
  path_container_front_.erase(std::remove_if(path_container_front_.begin(), path_container_front_.end(),
                              [](const TopoPath& path){
                                return !path.safty;
                              }), path_container_front_.end());
  path_container_back_.erase(std::remove_if(path_container_back_.begin(), path_container_back_.end(),
                              [](const TopoPath& path){
                                return !path.safty;
                              }), path_container_back_.end());
  if(path_container_front_.empty()) path_container_front_.swap(path_container_back_);

  for(int i = 0; i < path_container_front_.size(); ++i)
  {
    if(!path_container_front_[i].safty) 
    {
      ROS_ERROR("Should not unsafty path appear here!");
      continue;
    }

    // 这里不能直接接上去了。先判断start_change和当前路径是不是同伦，如果是，则直接修改path的起点；
    // 如果不是，则还是接到后面去short
    // TODO，这里是不是可能出错？这个更新的时候，是不是不用考虑start_change，而是直接以当前起点为准，去找拓扑路径上从头开始的visible点？
    if(!start_change_.empty())
    {
      vector<Eigen::Vector3d> path1{start_change_[0], path_container_front_[i].path[1]};
      vector<Eigen::Vector3d> path2 = start_change_;
      path2.insert(path2.end(), path_container_front_[i].path.begin(), path_container_front_[i].path.begin() + 2);
      // publishTestPath(path1, 1);
      // publishTestPath(path2, 2);
      if(sameTopoPath(path1, path2, -1.0, false))
      {
        path_container_front_[i].path[0] = start_change_[0]; //直接更换起点
      }
      else
        path_container_front_[i].path.insert(path_container_front_[i].path.begin(), 
                start_change_.begin(), start_change_.end());      
    }
    shortcutPath(i, true);
    path_container_front_[i].length = pathLength(path_container_front_[i].path);
    // checkPathObstacle2(path_container_front_[i].path);
  }
  for(int i = 0; i < path_container_back_.size(); ++i)
  {
    if(!path_container_back_[i].safty) 
    {
      ROS_ERROR("Should not unsafty path appear here!");
      continue;
    }
    if(!start_change_.empty())
    {
      vector<Eigen::Vector3d> path1{start_change_[0], path_container_back_[i].path[1]};
      vector<Eigen::Vector3d> path2 = start_change_;
      path2.insert(path2.end(), path_container_back_[i].path.begin(), path_container_back_[i].path.begin() + 2);
      // publishTestPath(path1, 1);
      // publishTestPath(path2, 2);
      if(sameTopoPath(path1, path2, -1.0, false))
      {
        path_container_back_[i].path[0] = start_change_[0]; //直接更换起点
      }
      else
        path_container_back_[i].path.insert(path_container_back_[i].path.begin(), 
                start_change_.begin(), start_change_.end());      
    }
    // path_container_back_[i].path.insert(path_container_back_[i].path.begin(), 
    //         start_change_.begin(), start_change_.end());
    // publishTestPath(path_container_back_[i].path, 2);
    shortcutPath(i, false);
    path_container_back_[i].length = pathLength(path_container_back_[i].path);
    // publishTestPath(path_container_back_[i].path, 1);
    int debug = 0;
  }

  // 对两个组重新排序
  sort(path_container_front_.begin(), path_container_front_.end(),
       [&](TopoPath& path1, TopoPath& path2) { return path1.length < path2.length; });
  sort(path_container_back_.begin(), path_container_back_.end(),
       [&](TopoPath& path1, TopoPath& path2) { return path1.length < path2.length; });

  double minLength = std::numeric_limits<double>::max();
  // 如果path_container_front_都为空了，则说明整个path_container里面全部空了
  if(!path_container_front_.empty())
    minLength = path_container_front_.front().length;
  // else
  //   ROS_WARN("There is no path in path_container, maybe something wrong!");
  // if(path_container_front_.size() <)
  for(auto it = path_container_front_.begin(); it != path_container_front_.end(); )
  {
    if(it->length >= ratio_to_short_ * minLength)
    {
      it = path_container_front_.erase(it);   
      ROS_WARN("One path in Front erased by ratio_to_short_");   
    }
    else ++it;
  }
  if(!path_container_back_.empty())
  {
    // 至少在path_container_back_保留一个
    for(auto it = path_container_back_.begin() + 1; it != path_container_back_.end(); )
    {
      if(it->length >= ratio_to_short_ * minLength)
        it = path_container_back_.erase(it);
      else ++it;
    }
  }

  for(int i = 0; (i + 1) < path_container_front_.size(); ++i)
  {
    for(int j = i + 1; j < path_container_front_.size();)
    {
      bool same = sameTopoPath(path_container_front_[i].path, path_container_front_[j].path, 0.0, true);
      if(same)
      {
        path_container_front_.erase(path_container_front_.begin() + j);
      }
      else ++j;
    }
  }
  for(int i = 0; (i + 1) < path_container_back_.size(); ++i)
  {
    for(int j = i + 1; j < path_container_back_.size();)
    {
      bool same = sameTopoPath(path_container_back_[i].path, path_container_back_[j].path, 0.0, true);
      if(same)
      {
        path_container_back_.erase(path_container_back_.begin() + j);
      }
      else ++j;
    }
  }
  int debug = 0;
}

bool TopologyPRM::checkPathObstacle2(const std::vector<Eigen::Vector3d>& onePath)
{
  // publishTestPath(onePath, 1);

  Eigen::Vector3d colli_pt;
  bool safty = true;
  for(int i = 0; i < onePath.size() - 1; ++i)
  {
    if(!lineVisib(onePath[i], onePath[i + 1], clearance_, colli_pt, 0, -1))
    {
      ROS_ERROR_STREAM("Path Unsafty at (" << colli_pt.transpose() << "), dis = " << edt_environment_->sdf_map_->getDistance2D(colli_pt));
      publishTestPath(vector<Eigen::Vector3d>{onePath[i], onePath[i + 1]}, 2);
      safty = false;
    }
  }
  return safty;
}

// TopologyPRM::
}  // namespace fast_planner