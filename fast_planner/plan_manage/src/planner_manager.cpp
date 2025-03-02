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



// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <fstream>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <glog/logging.h>

bool save_info_ = 0;

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}
void calCtrlPtAngleAndVel(vector<double>& yaw_angle, vector<double>& yaw_angle_vel,
                          vector<double>& yaw_angle_vel_constraints,
                          const Eigen::MatrixXd& ctrl_pts, const double & ts);
vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}
void saveVectorToFile(const std::vector<double>& vec, const double& dt, const std::string& filename) {
    // 创建输出文件流对象
    std::ofstream outFile(filename);
    // 检查文件是否成功打开
    if (!outFile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    // 将vector中的每个元素写入文件
    int iter = 0;
    for (const double& value : vec) {
        outFile << value << " " << iter * dt << "\n";
        iter ++;
    }
    // 关闭文件
    outFile.close();
    // 检查文件是否成功关闭
    if (!outFile) {
        std::cerr << "Error closing file: " << filename << std::endl;
    }
}

void saveVector3dToFile(const std::vector<Eigen::Vector3d>& vec, const double& dt, 
                        const std::string& filename) {
    // 创建输出文件流对象
    std::ofstream outFile(filename);
    // 检查文件是否成功打开
    if (!outFile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    // 将vector中的每个元素写入文件
    int iter = 0;
    for (const Eigen::Vector3d& value : vec) {
        outFile << value.x() << " " << value.y() << " " << value.z() << " "<< iter * dt << "\n";
        iter ++;
    }
    // 关闭文件
    outFile.close();
    // 检查文件是否成功关闭
    if (!outFile) {
        std::cerr << "Error closing file: " << filename << std::endl;
    }
}


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

void saveTimeData(const double& time_cost, std::ofstream& file)
{
    file << time_cost << std::endl;
}

std::vector<double> calculateCurvature(const std::vector<Eigen::Vector3d>& points) {
    std::vector<double> curvatures;
    size_t num_points = points.size();
    // curvatures.push_back(0.0);
    for (size_t i = 1; i < num_points - 1; ++i) {
        Eigen::Vector3d p_prev = points[i - 1];
        Eigen::Vector3d p_curr = points[i];
        Eigen::Vector3d p_next = points[i + 1];

        // 向量计算
        double v1_x = p_curr(0) - p_prev(0);
        double v1_y = p_curr(1) - p_prev(1);
        double v2_x = p_next(0) - p_curr(0);
        double v2_y = p_next(1) - p_curr(1);

        // 向量长度
        double s1 = std::sqrt(v1_x * v1_x + v1_y * v1_y);
        double s2 = std::sqrt(v2_x * v2_x + v2_y * v2_y);

        // 向量夹角
        double dot_product = v1_x * v2_x + v1_y * v2_y;
        double cos_phi = dot_product / (s1 * s2);
        cos_phi = std::max(-1.0, std::min(1.0, cos_phi));  // 防止浮点数误差超出范围
        double phi = std::acos(cos_phi);

        // 曲率计算
        double curvature = phi / ((s1 + s2) / 2);
        curvatures.push_back(curvature);
    }
    // curvatures.push_back(0.0);
    return curvatures;
}

std::vector<Eigen::Vector3d> calVelFromGeoPath(const std::vector<Eigen::Vector3d>& path, double dt)
{
  std::vector<Eigen::Vector3d> vecVel;
  for(int i = 0; i < path.size() - 1; ++i)
  {
    vecVel.push_back((path[i+1] - path[i]) / dt);
  }
  return vecVel;
}

namespace fast_planner {

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);//这个是表示当前的环境是不是动态的，如果是动态的，那么就要使用动态的信息
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/only2D", only2D_, false);
  nh.param("manager/save_traj_info", save_info_, false);
  std::string file_path;
  nh.param("manager/FilePath", file_path);
  src_file = file_path;
  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  sdf_map_->setNeed2DMap(only2D_);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if(pp_.dynamic_)
  {
    obj_predictor_.reset(new ObjPredictor(nh));
    obj_predictor_->init();
  }


  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar2D());
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
      if(only2D_) bspline_optimizers_[i]->setOnly2D();
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
    topo_prm_->setOnly2D(only2D_);
  }
  google::InitGoogleLogging("");
  google::SetLogDestination(google::GLOG_INFO, (file_path + "test/debuglog/").c_str());      
  
  record_file_.open(file_path + "test/traj_record/data/planning_time_cost.txt", std::ios::out);
  if(!record_file_.is_open())
  {
      ROS_ERROR("Unable to open file to save traj info!");
  }

}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

//对已有的轨迹进行碰撞检测
//当前点是cur_pt，然后进入循环，fut_pt是往前0.02s采样的路径点
//如果fut_pt到障碍物的距离dist小于0.1，那么就说明碰撞了，并返回这个dist
//如果fut_pt到cur_pt的距离超过6m，则不再进行碰撞检查，节约时间
bool FastPlannerManager::checkTrajCollision(double& distance) {

  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0, this->only2D());
    // 注意，这个
    if (dist < 0.15) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel,
                                           Eigen::Vector3d start_yaw, Eigen::Vector3d end_yaw,
                                           vector<Eigen::Vector3d>& start_change) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: position: " << start_pt.transpose() << ", vel: " << start_vel.transpose() << ", acc: "
       << start_acc.transpose() << "\ngoal: position: " << end_pt.transpose() << ", vel: " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  local_data_.start_time_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0, t_topo = 0.0;;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();
  if(only2D_)
  {
    start_pt(2) = edt_environment_->sdf_map_->getGroundHeight() + 0.5;
    start_vel(2) = 0.0;
    start_acc(2) = 0.0;
    end_pt(2) = edt_environment_->sdf_map_->getGroundHeight() + 0.5;
    end_vel(2) = 0.0;
    start_yaw(0) = Mod2Pi(start_yaw(0));
    end_yaw(0) = Mod2Pi(end_yaw(0));
  }
  
  {//----------在这里搜索一个topo路径
    ROS_INFO("[Topo]: ---------");
    plan_data_.clearTopoPaths();
    Eigen::Vector3d start_pt_forward = start_pt + 0.0 * start_vel;
    start_pt_forward(2) = start_yaw(0);
    topo_prm_->setStartChange(start_change);
    list<GraphNode::Ptr>            graph;
    vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
    topo_prm_->findTopoPaths(start_pt_forward, end_pt, vector<Eigen::Vector3d>(), vector<Eigen::Vector3d>(), 
                             graph, raw_paths, filtered_paths, select_paths);

    // if (select_paths.size() == 0) {
    //   ROS_WARN("[Topo]: No path.");
    // }else
    {
       plan_data_.addTopoPaths(graph, raw_paths, topo_prm_->getPathContainer(2), topo_prm_->getPathContainer(1));
       plan_data_.topo_guide_path_ = topo_prm_->findDubinsShots(start_pt_forward, kino_path_finder_->getSteerRadius());
    }
    plan_data_.topo_sample_area_ = topo_prm_->getSampleArea();
  }
  t_topo = (ros::Time::now() - t1).toSec();
  t1                    = ros::Time::now(); //更新当前时间
  kino_path_finder_->reset();
  if(!plan_data_.topo_guide_path_.empty()) kino_path_finder_->setGuidePath(plan_data_.topo_guide_path_);
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, start_yaw(0), end_pt, end_vel, end_yaw(0), true);
  if (status == KinodynamicAstar::NO_PATH) 
  {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    // 如果是连续初始状态搜索，需要给定初始状态，由可能找不到可行的路径，搜索就要尝试不同的初始状态来搜索
    // 这个主要是给（主要触发）是在由悬停状态开始搜索的时候，如果在飞行过程中进行搜索，似乎都不需要二次搜索
    // 可以从traj_cmd期望的轨迹看到，如果是飞行过程中进行搜索，轨迹都是连续的；而如果是悬停状态开始搜索，则轨迹是不连续的
    // 这里的初始状态都是在每次调用search时计算的，如果时悬停状态，加速度是0，所以很有可能搜索不到路径
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, start_yaw(0), end_pt, end_vel, end_yaw(0), false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path!! in retry." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success. -------------retry--------" << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success. -------------init--------" << endl;
  }

  auto visitedNodes = kino_path_finder_->getVisitedNodes();
  vector<Eigen::Vector3d> visited_nodes;
  for(const auto & node : visitedNodes)
  {
    if(only2D_)
    {
      visited_nodes.push_back(Eigen::Vector3d(node->state(0), node->state(1), start_pt(2)));
    }
    else
    {
      visited_nodes.push_back(node->state.head(3));
    }
  }
  plan_data_.visited_nodes = visited_nodes;
  // if(kino_path_finder_->showSearchTree())
    plan_data_.search_tree = kino_path_finder_->getSearchTree();
  double delta_t_geo = 0.1;
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(delta_t_geo);
  // plan_data_.jps_path_ =  kino_path_finder_->getJpsPath();
  t_search = (ros::Time::now() - t1).toSec();
  t1                    = ros::Time::now(); //更新当前时间
  // 这里要改成手动的了，不要再getSamples
  // parameterize the path to bspline
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  double ts;
  if(only2D_)
  {
    ts = (pp_.ctrl_pt_dist / pp_.max_vel_);
    int iter = 0; int skip = ceil(ts/delta_t_geo + 1e-3);
    // ROS_WARN_STREAM("skip: " << skip << ", ts: " << ts << ", ctrl_pt_dis: " << pp_.ctrl_pt_dist << 
    //                 ", pp_.max_vel_: " << pp_.max_vel_);
    ts = skip * delta_t_geo;
    for(; iter < plan_data_.kino_path_.size(); iter += skip)
    {
      point_set.push_back(plan_data_.kino_path_[iter]);
    }
    if(iter < plan_data_.kino_path_.size() - 1)
    {
      point_set.push_back(plan_data_.kino_path_.back());
    }
    kino_path_finder_->getDerivatives(start_end_derivatives);
    if(status != KinodynamicAstar::REACH_END)
    {
      Eigen::Vector3d end_yaw = plan_data_.kino_path_[plan_data_.kino_path_.size() - 1] - plan_data_.kino_path_[plan_data_.kino_path_.size() - 2];
      start_end_derivatives[1] = end_yaw / end_yaw.norm() * 0.1;
    }
    // ROS_WARN("!!!!!!!!!!!!!!!!!1");
    // for(const auto & vec : start_end_derivatives)
    //   std::cout << vec.transpose() << std::endl;
  }
  else
  {
    ts = pp_.ctrl_pt_dist / pp_.max_vel_;//为什么是这两个数相除？，现在ts为0.5/3.0 = 0.16667s
    // getKinoTraj和getSamples都是按照时间间隔获取轨迹，而且如果ts相同，那么获得的点是一样的
    // getKinoTraj是用来可视化，所以点更加密集；getSamples是用来生成B样条轨迹的控制点，所以稀疏一点
    kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);    
  }

  // 用拟合的方法，通过SamplePoints来生成控制点
  // 这里是限制了起点和终点的位置、速度和加速度
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);
  if(save_info_) { saveBsplineInfo(init, "init"); }
  if(save_info_) { saveGeoPathInfo(plan_data_.kino_path_, "geo", delta_t_geo); }
  if(save_info_) { savePointsToFile(point_set, src_file + "path_sample.txt"); }
  local_data_.position_traj_tmp_ = init;
  //下面计算一下，开始和结尾两个控制点的连线方向是不是和规划的一样
  // std::cout << "ctrl_pts:\n" << ctrl_pts.matrix() << std::endl;
  int ctrl_pts_num = ctrl_pts.rows();
  double dx1 = ctrl_pts(1, 0) - ctrl_pts(0, 0), dy1 = ctrl_pts(1, 1) - ctrl_pts(0, 1);
  double dx2 = ctrl_pts(ctrl_pts_num - 1, 0) - ctrl_pts(ctrl_pts_num - 2, 0), dy2 = ctrl_pts(ctrl_pts_num - 1, 1) - ctrl_pts(ctrl_pts_num - 2, 1);
  // Eigen::Vector2d vec1 = ();
  double ctrl_pts_yaw_start = atan2(dy1, dx1), ctrl_pts_yaw_end= atan2(dy2, dx2);
  double error_yaw_start = Mod2Pi((ctrl_pts_yaw_start - start_yaw(0))), error_yaw_end = Mod2Pi((ctrl_pts_yaw_end - end_yaw(0)));
  // std::cout << "plan start yaw by vel:" << atan2(start_vel(1), start_vel(0)) << std::endl;
  // std::cout << "plan start yaw: " << start_yaw(0) << ", ctrl_pts_yaw_start: " << ctrl_pts_yaw_start << ", error: " << error_yaw_start << std::endl; 
  // std::cout << "plan end   yaw: " << end_yaw(0)   << ", ctrl_pts_yaw_end:   " << ctrl_pts_yaw_end   << ", error: " << error_yaw_end << std::endl; 
  // if(abs(error_yaw_start) * 57.3 > 20)
  // {
  //   ROS_ERROR("start yaw plan error!");
  //   // std::cout << "The start_end_derivatives yaw is: " << atan2(start_end_derivatives[0](1), start_end_derivatives[0](0)) << std::endl;
  // }
  // if(abs(error_yaw_end) * 57.3 > 20)
  // {
  //   ROS_ERROR("end yaw plan error!");
  //   // std::cout << "The start_end_derivatives yaw is: " << atan2(start_end_derivatives[1](1), start_end_derivatives[1](0)) << std::endl;
  // }

  double duration_init = init.getTimeSum();


  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    // cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ros::Time opt_t1, opt_t2, opt_t3, opt_t4, opt_t5, opt_t6;
  opt_t1 = ros::Time::now();
  auto ctrl_pts_before = ctrl_pts;
  if(only2D_)
    ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 4, 1);
  else
    ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);
  opt_t2 = ros::Time::now();
  // for(int i = 0; i < ctrl_pts.rows(); ++i)
  // {
  //   double dis1;
  //   Eigen::Vector3d grad1;
  //   edt_environment_->evaluateEDTWithGrad2D(ctrl_pts.row(i), -1, dis1, grad1);
  //   if(dis1 < 0.3)
  //     ROS_WARN_STREAM("idx:" << i << ", " << ctrl_pts_before.row(i) << " | " << ctrl_pts.row(i) << ", dis: " << dis1);
  // }
  { 
    auto traj_opt = NonUniformBspline(ctrl_pts, 3, ts);
    local_data_.position_traj_tmp_ = traj_opt;
    if(save_info_) saveBsplineInfo(traj_opt, "opt"); 
  }
  // Ctrl points yaw constraint
  // 这里变量的名字需要更换
  {
    int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::CTRLPTYAW;
    // cost_function &= ~BsplineOptimizer::SMOOTHNESS;
    auto ctrl_pts_yaw_constr = bspline_optimizers_[4]->BsplineOptimizeTraj(ctrl_pts_before, ts, cost_function, 4, 1);
    NonUniformBspline traj_yaw_constrin(ctrl_pts_yaw_constr, 3, ts);
    if(save_info_){ saveBsplineInfo(traj_yaw_constrin, "mid"); }
    // local_data_.position_traj_tmp_ = traj_yaw_constrin;// 
    // ctrl_pts = ctrl_pts_yaw_constr;
    opt_t3 = ros::Time::now();
    int fix_num_start, fix_num_end;
    bspline_optimizers_[4]->checkFixInterval(fix_num_start, fix_num_end);
    ROS_WARN_STREAM("fix_num: " << fix_num_start << ", " << fix_num_end);
    fix_num_start = min(6, fix_num_start);
    fix_num_start = max(9, fix_num_start);

    fix_num_end = min(6, fix_num_end);
    fix_num_end = max(9, fix_num_end);    
    if (status != KinodynamicAstar::REACH_END) {
      fix_num_end = -1;
    }
    cost_function = BsplineOptimizer::NORMAL_PHASE;
    cost_function = cost_function | BsplineOptimizer::CTRLPTYAWS2;
    auto ctrl_pts_yaw_mid = bspline_optimizers_[5]->BsplineOptimizeTraj
                            (ctrl_pts_yaw_constr, ts, cost_function, 1, 1, fix_num_start, fix_num_end, 0);
    NonUniformBspline traj_yaw_mid(ctrl_pts_yaw_mid, 3, ts);
    local_data_.position_traj_tmp_ = traj_yaw_mid;
    if(save_info_) saveBsplineInfo(traj_yaw_mid, "yaw_constr");
    ctrl_pts = ctrl_pts_yaw_mid;
    opt_t4 = ros::Time::now();
    // for(int i = 0; i < ctrl_pts.rows(); ++i)
    // {
    //   double          dist;
    //   Eigen::Vector3d dist_grad, g_zero(0, 0, 0);
    //   edt_environment_->evaluateEDTWithGrad2D(ctrl_pts.row(i), -1.0, dist, dist_grad);
    //   std::cout << edt_environment_->sdf_map_->getDistance2D(static_cast<Eigen::Vector2d>(ctrl_pts.row(i).head(2))) << ", ";
    //   std::cout << dist << "\n";
    //   std::cout << "idx:" << i << ", " << ctrl_pts_before.row(i) << " | " << ctrl_pts.row(i) << std::endl;
    // }

  }

  // perception
  {
    // int cost_function = BsplineOptimizer::PERCEPTION;
    int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::PERCEPTION;
    // cost_function &= ~BsplineOptimizer::FEASIBILITY;
    auto ctrl_pts_new = bspline_optimizers_[3]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1, -1, -1, 0);   
    // if(bspline_optimizers_[3]->getFlag() != 0 && bspline_optimizers_[3]->getMidCtrlPt().rows() > 0)
    // {
    //   NonUniformBspline traj_perception_mid(bspline_optimizers_[3]->getMidCtrlPt(), 3, ts);
    //   if(save_info_){saveBsplineInfo(traj_perception_mid, "mid");}    
    //   local_data_.position_traj_tmp_ = traj_perception_mid;// 使用考虑perception的规划  
    // }

    NonUniformBspline traj_perception_mid(ctrl_pts_new, 3, ts);
    plan_data_.block_pts_ = bspline_optimizers_[3]->getPerceptionInfo();    
    plan_data_.block_pts_[3] = traj_perception_mid.evaluateDeBoorT(plan_data_.block_pts_[3](0));
    if(save_info_) {saveBsplineInfo(traj_perception_mid, "perception");} 
    // local_data_.position_traj_tmp_ = traj_perception_mid;//   
    opt_t5 = ros::Time::now();
    // cost_function = cost_function | BsplineOptimizer::CTRLPTYAWS3;
    // auto ctrl_pts_new2 = bspline_optimizers_[6]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1, -1, -1, 0);   
    opt_t6 = ros::Time::now();
    // NonUniformBspline traj_perception_mid2(ctrl_pts_new2, 3, ts);
    // if(save_info_) {saveBsplineInfo(traj_perception_mid2, "mid");}   
    // local_data_.position_traj_tmp_ = traj_perception_mid2;// 
    ctrl_pts = ctrl_pts_new;
  }
  std::cout << "Opt time: \n smooth: " << (opt_t2 - opt_t1).toSec() * 1000 << ". \n yawS1: " << (opt_t3 - opt_t2).toSec() * 1000 << 
  ". \n yawS2: " << (opt_t4 - opt_t3).toSec() * 1000 << ". \n perception: " << (opt_t5 - opt_t4).toSec() * 1000 << ". \n perception yawS3: " <<
  (opt_t6 - opt_t5).toSec() * 1000 << ". " << std::endl;
 
  t_opt = (ros::Time::now() - t1).toSec();
  // iterative time adjustment
  t1                    = ros::Time::now();
  // 生成优化后的Bpline
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);


  // local_data_.position_traj_tmp_ = pos;
  // double                  tm, tmp;
  // pos.getTimeSpan(tm, tmp);
  // std::cout << "BSpline traj points Before Time Reallocate: ------------" << std::endl; 
  // for (double t = tm; t <= tmp; t += (tmp - tm)/100) {
  //   Eigen::Vector3d pt = pos.evaluateDeBoor(t);
  //   std::cout << pt.transpose() << std::endl;
  // }

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(1.05 * pp_.max_vel_, 1.05 * pp_.max_acc_); // 这里要加上一个尺度因子
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }
  if(save_info_) { saveBsplineInfo(pos, "reall", false); }
  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;
  // pos.getTimeSpan(tm, tmp);
  // std::cout << "BSpline traj points After Time Reallocate: ------------" << std::endl; 
  // for (double t = tm; t <= tmp; t += (tmp - tm)/100) {
  //   Eigen::Vector3d pt = pos.evaluateDeBoor(t);
  //   std::cout << pt.transpose() << std::endl;
  // }


  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();
  double t_total_e = (ros::Time::now() - local_data_.start_time_).toSec();
  // save planned results

  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: total time [ms]: " << t_total_e * 1000 << ", topo: " << t_topo * 1000 << ", search: " << t_search * 1000 << ", optimize: " << t_opt * 1000
       << ", adjust time:" << t_adjust * 1000 << endl;
  saveTimeData(t_total_e * 1000, record_file_);
  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // generate global reference trajectory

  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos); //把start_pos放到points的第一个

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // write position matrix
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;
  time(0) = max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double            dt, duration;
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /* truncate a new local segment for replanning */
  ros::Time time_now = ros::Time::now();
  double    t_now    = (time_now - global_data_.global_start_time_).toSec();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;

  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;//其实这个碰撞点非常靠近障碍物了吧
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                    << " seconds, time change is: " << time_inc);
}


void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = min(1.01, ratio);
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  ros::Time t1;
  double    tm1, tm2, tm3;

  t1 = ros::Time::now();

  // parameterize B-spline according to the length of guide path
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

  tm1 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (ros::Time::now() - t1).toSec();
  ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  // t_s记录了碰撞开始的时间，t_e记录了碰撞结束的时间。这两个时间都不碰撞
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }
  //没有碰撞
  if (colli_start.size() == 0) return;
  //进入碰撞但是一只没有出碰撞
  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int    sn = ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION
// 将当前计算出来的路径结果更新local_data_
// 具体地、更新轨迹、速度、加速度、起点、时间周期和轨迹编号
void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();//整一条轨迹的耗时？
  local_data_.traj_id_ += 1;
}


// 按照时间dt_yaw对已经优化的B样条轨迹进行采样，然后通过两个点来计算yaw的方向
// 这一部分之前看漏了，以为是像traj一样，计算初末状态和中间的yaw值，然后来拟合一条B样条曲线
// 现在看来是只计算初末状态，然后中间值都是0，来拟合
// 在优化阶段，用中间的yaw当作waypoints来引导
// 这么做的目的是防止smooth项来yaw角全部拉平
void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw, const Eigen::Vector3d& end_yaw_traj) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading
  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  double            forward_t = 2.0;
  if(only2D_)       forward_t = 0.5;
  double                  last_yaw  = Mod2Pi(start_yaw(0));
  // if(abs(last_yaw - start_yaw(0)) > 1e-5) cout << "start_yaw(0)调整，yaw的规划是不是出错？" << std::endl;
  double                  last_yaw_record = last_yaw;
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states
  // 两个pos才算一个waypt，waypt的数量就是seg_num
  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);//pc和pf是两个路径点
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    // 如果是已经到终点了
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
      if((waypt(0) - last_yaw_record) < -1.5 * M_PI) waypt(0) += 2*M_PI;
      else if((waypt(0) - last_yaw_record) > 1.5 * M_PI) waypt(0) -= 2*M_PI;
      last_yaw_record = waypt(0);
    } else {
      waypt = waypts.back();
    }
    // std::cout << "yaw: " << waypt(0) << std::endl;
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints
  // 计算初始和终止状态的控制点,yaw是控制点
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * Eigen::Vector3d(last_yaw, 0.0, 0.0);

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  // 注意这里end_yaw直接用外部输入的了
  end_yaw = end_yaw_traj;
  calcNextYaw(last_yaw, end_yaw(0));
  if((end_yaw(0) - last_yaw_record) < -1.5 * M_PI) end_yaw(0) += 2*M_PI;
  else if((end_yaw(0) - last_yaw_record) > 1.5 * M_PI) end_yaw(0) -= 2*M_PI;
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // std::cout << "yaw_dot_constraints: \n";
  // 这里是想还是按照均匀时间分布来对yaw角控制点加限制。
  // 但是现在换一种限制添加方式，用Hartley Judd算法.因为考虑到yaw的分布，其实并不是空间上均匀的吧
  std::vector<double> yaw_dot_constraints;  
  {
    double delta_t = duration / (seg_num + 3 - 1);
    for(double t = 0; t <= duration + 1e-5; t += delta_t)
    {
      yaw_dot_constraints.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t).norm() * tan(60 / 57.3) / 0.6);
      // yaw_dot_constraints.push_back(std::numeric_limits<double>::max());
      // std::cout << yaw_dot_constraints.back() << std::endl;
    }
  }
  std::vector<double> yaw_dot_desire_curve;
  string yaw_dot_desire_file = string(src_file + "angle_vel_constr.txt");
  NonUniformBspline  vel      = local_data_.position_traj_.getDerivative();
  double t_duration = vel.getTimeSum();
  for(double t = 0; t < t_duration + 1e-5; t += 0.05)
  {
    yaw_dot_desire_curve.push_back(vel.evaluateDeBoorT(t).norm() * tan(60 / 57.3) / 0.6);
  }
  if(save_info_) saveVectorToFile(yaw_dot_desire_curve, 0.05, yaw_dot_desire_file);  

  // solve
  // 使用B样条优化器来进行优化
  // 但是这个东西，用B样条来优化是没有什么意思的吧？
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  auto yaw_origin = yaw;
  if(only2D_)
    yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 4, 1);
  else
    yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);
  
  // std::cout << "ctrl_points_yaw origion:\n" << yaw.matrix().transpose() << std::endl;

  int opt2_succ = false;
  {
    bspline_optimizers_[2]->setWaypoints(waypts, waypt_idx);
    bspline_optimizers_[2]->setYawDotConstrains(yaw_dot_constraints);
    int cost_func =  BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS | BsplineOptimizer::FEASIBILITYYaw;
    yaw_origin           = bspline_optimizers_[2]->BsplineOptimizeTraj(yaw_origin, dt_yaw, cost_func, 1, 1);
    
    opt2_succ = bspline_optimizers_[2]->getFlag();
    NonUniformBspline yaw_traj_opt2(yaw_origin, 3, dt_yaw);
    if(save_info_) {saveYawBsplineInfo(yaw_traj_opt2, "opt2");}

    NonUniformBspline yaw_traj_opt1(yaw, 3, dt_yaw);
    if(save_info_) {saveYawBsplineInfo(yaw_traj_opt1, "opt1");}
  }

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();


  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  // 这个地方应该不用打开。如果打开的话，虽然yaw角更平滑了，但是其实就跟不上轨迹了？
  if(opt2_succ != 0)
  {
    local_data_.yaw_traj_.setUniformBspline(yaw_origin, 3, dt_yaw);
    local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  }

  std::cout << "[plan yaw] plan heading: " << (ros::Time::now() - t1).toSec() << ", last yaw: " << last_yaw << ", use FeasYAW: " << opt2_succ << std::endl;
}

// 将所有的yaw角卡在初始yaw+[-pi, pi]范围内。防止环绕引起的跳变
void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

void FastPlannerManager::saveBsplineInfo(NonUniformBspline& bspline, 
                                         const string& feature, bool isUniform)
{
  NonUniformBspline bspline_vel = bspline.getDerivative();
  NonUniformBspline bspline_acc = bspline_vel.getDerivative();
  double delta_t = 0.05;
  double duration = bspline.getTimeSum();
  int seg_num = floor(duration / delta_t);
  delta_t = duration / double(seg_num); 
  std::vector<Eigen::Vector3d> vecPath(seg_num + 1); 
  std::vector<Eigen::Vector3d> vecVel(seg_num + 1), vecAcc(seg_num + 1);
  for(int i = 0; i < seg_num + 1; ++i)
  {
    vecPath[i] = bspline.evaluateDeBoorT(i * delta_t);
    vecVel[i]  = bspline_vel.evaluateDeBoorT(i * delta_t);
    vecAcc[i]  = bspline_acc.evaluateDeBoorT(i * delta_t);
  }
  std::vector<double> vecCurv = bspline.getCurvature(delta_t);
  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
  std::vector<Eigen::Vector3d> ctrl_pts_vec = matrixToVectors(ctrl_pts);
  std::vector<double> vecW(vecCurv.size(), 0.0);
  for(int i = 0; i < vecCurv.size(); ++i)
  {
    vecW[i] = vecCurv[i] * vecVel[i].norm();
  }
  string file_path = src_file + "path_" + feature + ".txt";
  string file_vel  = src_file + "vel_" + feature + ".txt";
  string file_acc  = src_file + "acc_" + feature + ".txt";
  string file_ctrl = src_file + "path_ctrl_" + feature + ".txt";
  string file_curv = src_file + "curv_" + feature + ".txt";
  string file_w    = src_file + "w_" + feature + ".txt";
  savePointsToFile(vecPath, file_path);
  savePointsToFile(ctrl_pts_vec, file_ctrl);

  saveVector3dToFile(vecVel, delta_t, file_vel);
  saveVector3dToFile(vecAcc, delta_t, file_acc);
  saveVectorToFile(vecCurv, delta_t, file_curv);
  saveVectorToFile(vecW,    delta_t, file_w);

  string file_ctrl_yaw       = src_file + "ctrlYaw_" + feature + ".txt";
  string file_ctrl_yaw_dot   = src_file + "ctrlYawDot_" + feature + ".txt";
  string file_ctrl_yaw_const = src_file + "ctrlYawConstr_" + feature + ".txt";
  vector<double> yaw_angle, yaw_angle_vel, yaw_angle_vel_constraints;
  if(isUniform)
  {
    calCtrlPtAngleAndVel(yaw_angle, yaw_angle_vel, yaw_angle_vel_constraints, ctrl_pts, bspline.getInterval());
    saveVectorToFile(yaw_angle,       bspline.getInterval(), file_ctrl_yaw);
    saveVectorToFile(yaw_angle_vel,   bspline.getInterval(), file_ctrl_yaw_dot);
    saveVectorToFile(yaw_angle_vel_constraints, bspline.getInterval(), file_ctrl_yaw_const);
  }
}

void FastPlannerManager::saveYawBsplineInfo(NonUniformBspline& bspline, 
                                            const string& feature, bool isUniform)
{
  NonUniformBspline bspline_vel = bspline.getDerivative();
  double delta_t = 0.05;
  double duration = bspline.getTimeSum();
  int seg_num = floor(duration / delta_t);
  delta_t = duration / double(seg_num); 
  std::vector<double> vecAngle(seg_num + 1), vecAngleVel(seg_num + 1);
  for(int i = 0; i < seg_num + 1; ++i)
  {
    vecAngle[i]  = bspline.evaluateDeBoorT(i * delta_t)(0);
    vecAngleVel[i]  = bspline_vel.evaluateDeBoorT(i * delta_t)(0);
  }

  string file_angle      = src_file + "angle_" + feature + ".txt";
  string file_angle_vel  = src_file + "angle_vel_" + feature + ".txt";

  saveVectorToFile(vecAngle,  delta_t, file_angle);
  saveVectorToFile(vecAngleVel,  delta_t, file_angle_vel);
}

void FastPlannerManager::saveGeoPathInfo(const vector<Eigen::Vector3d> & path, 
                                         const string & feature, const double & delta_t)
{
  string file_path = src_file + "path_" + feature + ".txt";
  string file_vel  = src_file +  "vel_" + feature + ".txt";
  string file_curv = src_file + "curv_" + feature + ".txt";
  savePointsToFile(path, file_path);
  saveVector3dToFile(calVelFromGeoPath(path, delta_t), delta_t, file_vel);
  saveVectorToFile(calculateCurvature(path), delta_t, file_curv);
}

void FastPlannerManager::topoUpdate(std::vector<Eigen::Vector3d>& start_change)
{
  topo_prm_->setStartChange(start_change);
  topo_prm_->preprocess();
}

}  // namespace fast_planner

void calCtrlPtAngleAndVel(vector<double>& yaw_angle, vector<double>& yaw_angle_vel, 
                          vector<double>& yaw_angle_vel_constraints,
                          const Eigen::MatrixXd& ctrl_pts, const double & ts)
{
  yaw_angle.clear();
  yaw_angle_vel.clear();
  yaw_angle_vel_constraints.clear();
  Eigen::Vector3d vec1, vec0, vel;
  double angle, angle_vel;
  double rorate_dir;
  // 前3个不动
  // 使用后向差分来计算当前点的角速度.计算的是当前i的！！
  for(int i = 0; i < ctrl_pts.rows() - 2; ++i)
  {
    vec0 = ctrl_pts.row(i + 1) - ctrl_pts.row(i - 0);  //前一个vec
    vec1 = ctrl_pts.row(i + 2) - ctrl_pts.row(i + 1);
    
    angle = std::atan2(vec0(1), vec0(0));
    // theta1 - theta0的值大小就是两个向量的夹角，然后再通过旋转方向来判断正负
    angle_vel = acos(vec0.dot(vec1) / (vec1.norm() * vec0.norm()))/ts;
    rorate_dir = Eigen::Vector3d(0, 0, 1).dot(vec0.cross(vec1));
    // if(rorate_dir < 0) angle_vel *= -1;
    yaw_angle.push_back(angle);
    yaw_angle_vel.push_back(angle_vel);
    vel = vec0;
    yaw_angle_vel_constraints.push_back(vel.norm()/ts * tan(60 / 57.3) / 0.6);
  }
}
