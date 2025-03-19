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

#include <path_searching/kinodynamic_astar_2D.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

template<typename T> 
Eigen::Matrix<T, 2, 1> Vec3ToVec2(const Eigen::Matrix<T, 3, 1>& vec3) 
{
  return vec3.head(2); 
}

template<typename T> 
Eigen::Matrix<T, 3, 1> Vec2ToVec3(const Eigen::Matrix<T, 2, 1>& vec2) 
{
  return Eigen::Matrix<T, 3, 1>(vec2(0), vec2(1), 0); 
}

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

std::vector<double> computePathDis(const std::vector<Eigen::Vector3d>& path) {
    std::vector<double> distances(path.size(), 0.0);
    // 从倒数第二个点开始，往前累加路径距离
    for (int i = path.size() - 2; i >= 0; --i) {
        // 计算当前点和下一个点之间的欧氏距离
        double segmentDistance = (path[i + 1] - path[i]).norm();
        
        // 当前点到终点的距离 = 下一个点到终点的距离 + 当前段的距离
        distances[i] = distances[i + 1] + segmentDistance;
    }
    return distances;
}

namespace fast_planner
{
KinodynamicAstar2D::~KinodynamicAstar2D()
{
  for (int i = 0; i < allocate_num_; ++i)
  {
    delete path_node_pool_[i];
  }
}

// init是表明是不是这一次搜索的第一次。
// dynamic是说是不是要把时间也考虑为状态量，一般就考虑x,y,z
// 这个接口，2D和3D要一致，但是进来之后，先处理一下
int KinodynamicAstar2D::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, double start_yaw,
                               Eigen::Vector3d end_pt, Eigen::Vector3d end_v, double end_yaw, bool init, bool dynamic, double time_start, bool gen_search)
{
  // // ------------现在用topo路径替换JPS来做搜索引导，但是依然使用jps_path_finder_里面的辅助函数--------
  if(topo_path_.size() >= 2)
  {
    // for(const auto pt : topo_path_) std::cout << pt.transpose() << std::endl;

    // jps_path_finder_->reset();
    // jps_path_finder_->arrangePath(topo_path_);
    // jps_path_finder_->setZ(start_pt(2));
    // auto topo_path_points = jps_path_finder_->getJpsPathPoints();
    // topo_path_distance_ = jps_path_finder_->getJpsPathDistance();

    // 换了一种guide 路径生成方式，这里输入的直接就是稠密离散的路径点
    const auto& topo_path_points = topo_path_;
    topo_path_distance_ = computePathDis(topo_path_points);

    pcl::PointCloud<pcl::PointXY>::Ptr path_cloud(new pcl::PointCloud<pcl::PointXY>);    
    for(const auto& topo_path_point : topo_path_points)
    {
      pcl::PointXY tmpPoint;
      tmpPoint.x = topo_path_point(0);
      tmpPoint.y = topo_path_point(1);
      path_cloud->points.emplace_back(tmpPoint);
      // std::cout << topo_path_point.transpose() << std::endl;
    }
    JPSPathKdTree.setInputCloud(path_cloud);
  }
  start_yaw_ = start_yaw;
  end_yaw_ = end_yaw;
  start_vel_ = start_v;
  start_acc_ = start_a;
  ground_height_ = start_pt(2);
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(2) = start_pt.head(2);
  cur_node->state(2) = start_yaw;
  cur_node->state(3) = start_v.norm();
  cur_node->index = stateToIndex(cur_node->state.head(3));
  cur_node->g_score = 0.0;
  cur_node->steering_grade = 0;
  // 起点就是在障碍物里面的
  if(edt_environment_->sdf_map_->getInflateOccupancy2D(start_pt.head(2)) == 1)
  {
    ROS_ERROR("the start point is in obsctal");
  }

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(2) = end_pt.head(2);
  end_state(2) = end_yaw;
  end_state(3) = end_v.norm();
  end_index = stateToIndex(end_state.head(3));
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal) + 0.0;
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  // 通过上一次搜索所用的节点数来判断是否遇到了U型墙
  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  bool init_expand = true;
  // 最终达到终点一定要通过oneshot完成
  const int tolerance_dis = ceil(1 / resolution_);
  // const int tolerance_yaw = ceil(1 / );

  vector<Eigen::Matrix<double, 6, 1>> mid_states_tmp;
  mid_states_tmp.resize(mid_states_size_);
  // main loop
  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // 上面判断到达终点的条件，以及oneshot的触发全部需要修改
    // 注意终止条件：1.到达horizon；2.oneshot成功；3.oneshot没有成功，但是达到了end附近（说明就无法one shot了）
    // TODO: 这里不对，终止条件只有到达horizon和near_end，只有在这两个条件下才会触发one shot。
    // 但其实one shot应该更多地被触发
    bool reach_horizon = (cur_node->state.head(2) - start_pt.head(2)).norm() >= horizon_;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance_dis &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance_dis;
    bool need_shot = (cur_node->state.head(2) - end_state.head(2)).norm() <= shot_distance_;
    need_shot = need_shot && (!init_search ); //这里可能再加一个限制
    if(reach_horizon || need_shot)
    {
      if(need_shot)
      {
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        // ROS_WARN_STREAM("[Shot] Start state: " << cur_node->state.transpose() << ".");
        // ROS_WARN_STREAM("[Shot]   End state: " << end_state.transpose() << ".");
        //只在终点附近才OneShot，附近的范围是4m范围，其实不算小了。如果成功，就设置了is_shot_succ_为true
        computeShotTraj(cur_node->state, end_state, time_to_goal);        
      }       
      if(is_shot_succ_)
      {
          terminate_node = cur_node;
          retrievePath(terminate_node);//就是回溯出路径 
          std::cout << "reach end, ";
          cout << "use node num: " << use_node_num_ << endl;
          use_node_num_last_ = use_node_num_;
          return REACH_END;
      }
      else
      {
        if(reach_horizon)
        {
          terminate_node = cur_node;
          retrievePath(terminate_node);//就是回溯出路径
          std::cout << "reach horizon, ";
          cout << "use node num: " << use_node_num_ << endl;
          use_node_num_last_ = use_node_num_;
          return REACH_HORIZON;
        }
        if(near_end)
        {
          terminate_node = cur_node;
          retrievePath(terminate_node);//就是回溯出路径
          if (cur_node->parent != NULL)//已经到达终点附近
          {
            std::cout << "near end, ";
            cout << "use node num: " << use_node_num_ << endl;
            use_node_num_last_ = use_node_num_;
            return NEAR_END;
          }
          else
          {
            std::cout << "no path, ";
            cout << "use node num: " << use_node_num_ << endl;
            use_node_num_last_ = use_node_num_;
            return NO_PATH;
          }
        }
      }
    }

    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;//lattice的状态
    vector<PathNodePtr> tmp_expand_nodes; //当前节点扩展时产生的新节点
    Eigen::Vector2d um;
    double pro_t;                  //pro_t，记录扩展的节点到起点的总时间
    vector<Eigen::Vector2d> inputs;//计算前向latteice的话，输入就是三轴加速度和dt
    vector<double> durations;
    int tmp_steering_grade;
    // TODO: 把速度离散也考虑进去or考虑不同的时间？
    // REVERSE
    // 这个是不是应该和左右脚有关呢？比如规划的时候正在迈左脚，或在右脚的落脚姿态是朝右，那么接下来的一步要么向前要么向右，不能向左？
    if (init_search)
    {     
      // for (int i = -steering_angle_discrete_num_; i <= steering_angle_discrete_num_; ++i)
      // {
      //   um << cur_node->state(3), steering_angle_rads_[i + steering_angle_discrete_num_];
      //   inputs.push_back(um);
      // }  
      um << cur_node->state(3), 0.0;
      inputs.push_back(um);
      // durations.push_back(init_max_tau_);
      if(start_v.norm() >= 0.7 * max_vel_) durations.push_back(static_cast<int>(floor(5 * init_max_tau_)) * 0.1);
      else durations.push_back(static_cast<int>(floor(5 * init_max_tau_)) * 0.1);
      // inputs.push_back(Eigen::Vector2d(max_vel_, steering_angle_rads_[cur_node->steering_grade + steering_angle_discrete_num_]));
      // durations.push_back(max_tau_);
      init_search = false; /////////--------!!!---------------注意，这里在经过一轮init_search之后就已经设置为false了
                           /////////这就是我想要的，当机器人正在运行的过程中，刚开始的几次搜索就应该沿着机器人当前状态进行，后面的可以变化
    }
    else
    {
      for (int i = -steering_angle_discrete_num_; i <= steering_angle_discrete_num_; ++i)
      {
        um << max_vel_, steering_angle_rads_[i + steering_angle_discrete_num_];
        inputs.push_back(um);
        // TODO：REVERSE
        if(reverse_enable_)
        {
          um << -max_vel_, steering_angle_rads_[i + steering_angle_discrete_num_];
          inputs.push_back(um);
        }
      }
      durations.push_back(max_tau_);
      durations.push_back(static_cast<int>(floor(5 * max_tau_)) * 0.1);
      // durations.push_back(0.5 * max_tau_);
    }

    // 扩展节点
    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        if(std::abs(um(1)) > 40/57.3) tau = static_cast<int>(floor(5 * init_max_tau_)) * 0.1;///----待修改!!

        // Check safety，同时也计算出最后的状态
        // 是不是要对这个检查是否在地图内？
        Eigen::Vector2d pos;
        Eigen::Matrix<double, 6, 1> x0, xt;
        bool is_occ = false;
        x0 = cur_state;
        if(show_search_tree_) mid_states_tmp[0] = x0;
        for (int k = 1; k <= (int)round(tau/expand_time_); ++k)
        {
          stateTransit(x0, xt, um, expand_time_);
          x0 = xt;
          if(show_search_tree_) mid_states_tmp[k] = xt;
          pos = xt.head(2);
          if (edt_environment_->sdf_map_->getDistance2D(pos) < obs_dis_ )
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }
        pro_state = xt;

        pro_t = cur_node->time + tau;
        // TODO: REVERSE。这个必须要考虑其他的情况
        tmp_steering_grade = um(1) / (steering_radian_) * steering_angle_discrete_num_; 
        // Check if in close set
        Eigen::Vector3i pro_id = stateToIndex(pro_state.head(3));
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        //如果这个lattice状态已经被expaned过了，且在CLOSE_SET里面
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }


        //前面的check都通过了，说明这个neighbor node是合格的
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = estimateG(cur_state, cur_node->steering_grade, pro_state, tmp_steering_grade, tau);
        tmp_g_score += cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent，当前的节点作为父节点的情况
        // 在扩展（expand）的时候，有一系列的um（加速度输入）和dt，每个都要计算一条lattice
        // 然后把每个lattice都要加入到tmp_expand_nodes里面。
        // 但是如果在同一个curr_node扩展的时候，如果某个lattice是在其他lattice已经扩展过的，那就比较一下
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = Vec2ToVec3(um);
              expand_node->duration = tau;
              expand_node->direction = PathNode::FORWARD;
              expand_node->steering_grade = tmp_steering_grade;              
              // expand_node->parent = cur_node; //父节点都是同样的，不用改
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          //如果该次扩展的节点不在之前expanded_nodes_里面，是新访问的，则新建
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = Vec2ToVec3(um);
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            pro_node->direction = PathNode::FORWARD;
            pro_node->steering_grade = tmp_steering_grade;
            if(show_search_tree_) 
            {
              pro_node->mid_states_ = mid_states_tmp;
              pro_node->mid_states_num_ = round(tau/expand_time_) + 1;              
            }
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory. use node num: " << use_node_num_ << endl;
              return NO_PATH;
            }
          }
          // 如果这个节点是在OPEN SET里面，且状态更好，则更新
          // 注意，这里不是用现在的节点替换掉之前在open set里面的节点，而是直接修改之前节点的状态和参数
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id; // 这个为什么不用修改？
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = Vec2ToVec3(um);
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              pro_node->direction = PathNode::FORWARD;
              pro_node->steering_grade = tmp_steering_grade;
              if(show_search_tree_) 
              {
                pro_node->mid_states_ = mid_states_tmp;
                pro_node->mid_states_num_ = round(tau/expand_time_) + 1;              
              }
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar2D::setParam(ros::NodeHandle& nh)
{
  nh.param("search_2D/max_tau", max_tau_, -1.0);            //0.6，在算前向latteice的时候，的采样时间。
  nh.param("search_2D/init_max_tau", init_max_tau_, -1.0);  //0.8，
  nh.param("search_2D/max_vel", max_vel_, -1.0);
  nh.param("search_2D/max_acc", max_acc_, -1.0);
  nh.param("search_2D/w_time", w_time_, -1.0);              //在计算cost的时候，time的权重
  nh.param("search_2D/horizon", horizon_, -1.0);            //搜索的边界范围
  nh.param("search_2D/resolution_astar", resolution_, -1.0);//地图分辨率
  nh.param("search_2D/time_resolution", time_resolution_, -1.0);//采样时间的分辨率
  nh.param("search_2D/lambda_heu", lambda_heu_, -1.0);      //启发项权重
  nh.param("search_2D/allocate_num", allocate_num_, -1);    //100000，提前给node pool分配的数量
  nh.param("search_2D/check_num", check_num_, -1);          //碰撞检测的采样点个数
  nh.param("search_2D/optimistic", optimistic_, true);      //不知道是干啥的
  nh.param("search_2D/obs_dis", obs_dis_, -1.0);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search_2D/vel_margin", vel_margin, 0.0);//这个是干啥的？
  // max_vel_ += vel_margin;
  max_vel_ *= 1.05;
  // max_acc_ *= 1.7;

  steering_angle_ = nh.param("search_2D/steering_angle", 30);
  steering_angle_discrete_num_ = nh.param("search_2D/steering_angle_discrete_num", 2);
  // TODO: 把速度离散加进去
  vel_discrete_num_            = nh.param("search_2D/vel_discrete_num", 1);
  wheel_base_ = nh.param("search_2D/wheel_base", 0.8);         //轴距，即前后轮的距离
  steering_penalty_ = nh.param("search_2D/steering_penalty", 1.3);
  steering_change_penalty_ = nh.param("search_2D/steering_change_penalty", 1.5);
  reversing_penalty_ = nh.param("search_2D/reversing_penalty", 2.0);
  vehicle_length_ = nh.param("search_2D/vehicle_length", 2.0);
  vehicle_width_ = nh.param("search_2D/vehicle_width", 1.0);
  vehicle_rear_dis_ = nh.param("search_2D/vehicle_rear_dis", 0.5);
  yaw_resolution_ = nh.param("search_2D/yaw_resolution", 10.0) * M_PI / 180.0;
  shot_distance_ = nh.param("search_2D/shot_distance", 5.0);
  expand_time_ = nh.param("search_2D/expand_time", 0.1);
  show_search_tree_ = nh.param("search_2D/show_search_tree", true);
  reverse_enable_ = nh.param("search_2D/reverse_enable", false);
  useJPS_ = nh.param("search_2D/use_JPS", false);   // we do not it now.
  {
    jps_path_finder_.reset(new JumpPointSearch());
    jps_path_finder_->setParam(nh);
    jps_path_finder_->setEnvironment(edt_environment_);
    jps_path_finder_->init();   
    // ROS_WARN("use JPS for H! %d", useJPS_);
    useJPS_ = false;
  }
}

// 注意，这里的节点，start的节点是没有输出、duration等信息的
// 后续的节点，储存的state是parent扩展后的，input和duration是扩展parent时用的
// start节点也是在path_nodes_里面的
void KinodynamicAstar2D::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);
  double expand_len = 0.0;
  while (cur_node->parent != NULL)
  {
    expand_len += cur_node->input(0) * cur_node->duration;
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }
  expand_len_ = expand_len;

  reverse(path_nodes_.begin(), path_nodes_.end());
}

//这个按照深蓝学院课程第四章推导就有了
double KinodynamicAstar2D::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  double h, h_topo = 0.0;
  h = (x1.head(2) - x2.head(2)).lpNorm<2>();
  // TODO: REVERSE  
  if (reverse_enable_) {
      // h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
      //                            current_node_ptr->state_.z(),
      //                            terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
      //                            terminal_node_ptr->state_.z());
  }
  else if (
    h < 1.4 * shot_distance_ && 
    !reverse_enable_)
  {
      // start
      double q0[] = { x1(0), x1(1), x1(2) };
      // goal
      double q1[] = { x2(0), x2(1), x2(2) };
      // initialize the path
      // calculate the path
      dubins_init(q0, q1, steering_radius_ * 1.0, &path_); //最小转弯半径为0.76m

      float length = dubins_path_length(&path_);
      h = length;
  }
  // TODO: 这里，是不是用梯形时间分布更好[肯定要考虑这个速度]
  // 改了，改成匀减速运动的模型
  // total_t_ = 2 * h / x1(3);
  // acc_shot_ = (x1(3) - 0) / total_t_;
  // optimal_time = h / (max_vel_);
  optimal_time = 0.0;
  // 这里加入JPS路径or Topo路径作为引导
  // 这里的H项还需要进一步设计
  if(topo_path_.size() >= 2)
  {
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    pcl::PointXY queryPoint;
    queryPoint.x = x1(0);
    queryPoint.y = x1(1);
    JPSPathKdTree.nearestKSearch(queryPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    float nearDistance = std::sqrt(pointNKNSquaredDistance[0]);
    if(nearDistance > 0.4) 
    {
      h_topo = topo_path_distance_[pointIdxNKNSearch[0]] + nearDistance;
    }
    else
    {
      h_topo = topo_path_distance_[pointIdxNKNSearch[0]];
    }
    h = h_topo;
  }
  // h = std::max(h, h_topo);
  // std::cout << "h: " << 1.0 * (tie_breaker_) * h << std::endl;
  return 1.0 * (tie_breaker_) * h;
}

bool KinodynamicAstar2D::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  double length = dubins_path_length(&path_);
  double x = 0.0;
  double move_step_size = resolution_;
  while (x <  length){
    double q[3];
    dubins_path_sample(&path_, x, q);
    if (q[0] < origin_(0) || q[0] >= map_size_3d_(0) || q[1] < origin_(1) || q[1] >= map_size_3d_(1))
    {
      return false;
    }
    Eigen::Vector2d coord(q[0], q[1]);
    if (edt_environment_->sdf_map_->getDistance2D(coord) < obs_dis_)
    {
      return false;
    }
    x += move_step_size;
  }
  shot_len_ = length; 
  is_shot_succ_ = true;
  return true;
}

//初始化地图
void KinodynamicAstar2D::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  this->inv_yaw_resolution_ = 1.0 / yaw_resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  steering_radian_ = steering_angle_ * M_PI / 180.0;
  steering_radius_ = wheel_base_ / tan(steering_radian_);
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);
  map_size_3d_ += origin_;
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);//先将所有的节点都创建好。用空间换取时间。后面所有的节点都是从path_node_pool_里面找
  mid_states_size_ = round(max(init_max_tau_, max_tau_)/expand_time_) + 1;  
  for (int i = 0; i < allocate_num_; ++i)
  {
    path_node_pool_[i] = new PathNode;
    if(show_search_tree_) 
    {
      path_node_pool_[i]->mid_states_.resize(mid_states_size_);
    }
    
  }

  use_node_num_ = 0;
  iter_num_ = 0;

  for(int i = -steering_angle_discrete_num_; i <= steering_angle_discrete_num_; ++i)
  {
    steering_angle_rads_.push_back(steering_radian_ / steering_angle_discrete_num_ * i);
  }
}

void KinodynamicAstar2D::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

//每次在规划的时候都需要先reset，所以这个地方一定要高效
void KinodynamicAstar2D::reset()
{
  expanded_nodes_.clear();//使用这种形式更加高效
  path_nodes_.clear();
  // topo_path_.clear();  //不要直接clear，有时候这一次找不到引导路径，还可以用上次的
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; ++i)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

// 返回轨迹上的时间离散点。包括OneShot轨迹上的点
// 感觉这里就很呆，为什么不用原代码里面地扩展方式？得到地还是多项式，挺方便。或者先拟合为五次多项式也挺好啊
std::vector<Eigen::Vector3d> KinodynamicAstar2D::getKinoTraj(double& delta_t)
{
  vector<Vector3d> state_list;      //总的
  vector<Vector3d> state_list_tmp;  //每一个搜索扩展的
  /* ---------- get traj of searching ---------- */
  double start_v = start_vel_.norm();
  if(!is_shot_succ_) total_len_ = expand_len_;
  else total_len_ = expand_len_ + shot_len_;
  start_v = min(start_v, max_vel_);
  std::cout << "total_len_: " << total_len_ << ", start_v: " << start_v << std::endl;
  std::cout << "expand_len_: " << expand_len_ << ", shot_len_: " << shot_len_ << std::endl;
  case_id_ = calVelOnShotTraj(start_v); 
  int seg_num = floor(total_t_ / delta_t);
  delta_t = total_t_ / double(seg_num); 
  std::cout << "total_t_: " << total_t_ << ", delta_t: " << delta_t << std::endl;
  // 此时的path_nodes_已经retrive过了，.back()就是最后一个节点. path_nodes_里面最少有1个
  // 我要从start往end开始遍历
  Matrix<double, 6, 1> x0, xt;  
  int node_num = 1; //当前的位置处于哪个node的区间。无效的start节点也在path_nodes_里面的
  std::vector<double> vecLength;

  double accum_dis = 0.0, accum_dis_last = 0.0;
  double curr_node_dis = 0.0;
  double target_dis = 0.0;
  double left_dis = 0.0, left_time = 0.0;
  int left_num = 0;
  double q[3];
  // 这里是在初始化
  if(path_nodes_.size() > 1)
  {
    curr_node_dis = path_nodes_[node_num]->duration * path_nodes_[node_num]->input(0);
    accum_dis_last = accum_dis;
    accum_dis += curr_node_dis;
    x0 = path_nodes_[node_num]->parent->state;      //注意啊！这里是parent的状态   
    vecLength.resize(path_nodes_.size(), 0);
    for(int i = 1; i < path_nodes_.size(); ++i)
    {
      vecLength[i] = (vecLength[i - 1] + path_nodes_[i]->duration * path_nodes_[i]->input(0));
    }
  }
  // std::cout << "getKinoTraj: 1" << std::endl;
  double delta_t_use = delta_t; double scale_factor = 1.0; double shot_curv = 0.0;
  double curv_tmp;
  for(double t = 0; t < total_t_ + 1e-3; t += delta_t_use)
  {
    target_dis = getDist(t, start_v);
    if(target_dis < expand_len_ - 1e-3)
    {
      if(target_dis > accum_dis + 1e-3) 
      {   
        ++ node_num;
      }
      accum_dis_last = vecLength[node_num - 1];
      accum_dis      = vecLength[node_num];

      left_time = (target_dis - accum_dis_last) / path_nodes_[node_num]->input(0);
      // std::cout << left_time << std::endl;
      left_num = max(0, (int)floor(left_time/expand_time_));
      
      if(node_num >= path_nodes_.size()) 
      {
        ROS_ERROR_STREAM("node_num error! " << node_num << ", " << path_nodes_.size());    
        t += delta_t_use;
        continue;
      }  
      if(left_num >= path_nodes_[node_num]->mid_states_num_) ROS_ERROR_STREAM("node_num error! " << left_num);
      left_time = left_time - left_num * expand_time_;
      x0 = path_nodes_[node_num]->mid_states_[left_num]; 
      stateTransit(x0, xt, path_nodes_[node_num]->input.head(2), left_time);
      // std::cout << node_num << ", " << x0.transpose() << ", " << xt.transpose() << ", " << path_nodes_[node_num]->input.head(2).transpose() << ", "
      // << ", " << left_time  << ", " << left_num << std::endl;
      state_list.emplace_back(Vector3d(xt(0), xt(1), ground_height_)); 

      curv_tmp = 1.0 / wheel_base_ * tan(path_nodes_[node_num]->input(1));
      scale_factor = calScaleFactor(t, start_v, curv_tmp);
    }
    else
    {
      if(!is_shot_succ_) break;
      left_dis = (target_dis - expand_len_);      
      // std::cout << "-----left_dis : " << left_dis << ", shot_len_: " << shot_len_ << std::endl;
      dubins_path_sample(&path_, left_dis, q, &shot_curv);
      scale_factor = calScaleFactor(t, start_v, shot_curv);

      Eigen::Vector3d coord(q[0], q[1], ground_height_);
      state_list.emplace_back(coord);
    }
    // TODO: 好像不需要这个？
    // std::cout << "scale_factor: " << scale_factor << std::endl;
    delta_t_use = scale_factor * delta_t;    
  }
  // std::cout << "getKinoTraj: 2" << std::endl;
  // // 此时的path_nodes_已经retrive过了，.back()就是最后一个节点
  // PathNodePtr node = path_nodes_.back();
  // double shot_v = node->state(3);
  // Matrix<double, 6, 1> x0, xt;
  // int times;
  // while (node->parent != NULL)
  // {
  //   Vector2d ut = Vec3ToVec2(node->input);
  //   double duration = node->duration;
  //   x0 = node->parent->state;      //注意啊！这里是parent的状态
  //   times = 0;
  //   for (double t = duration; t >= 1e-5; t -= delta_t)
  //   {
  //     stateTransit(x0, xt, ut, delta_t);
  //     x0 = xt;
  //     state_list_tmp.push_back(Vector3d(xt(0), xt(1), ground_height_));
  //     times++;
  //     // std::cout << "t: " << t << ",times: " << times << std::endl;
  //   }
    
  //   state_list.insert(state_list.begin(), state_list_tmp.begin(), state_list_tmp.end());
  //   state_list_tmp.resize(0);
  //   node = node->parent;
  // }
  // state_list.insert(state_list.begin(), Eigen::Vector3d(node->state(0), node->state(1), ground_height_));
  // // reverse(state_list.begin(), state_list.end());
  // /* ---------- get traj of one shot ---------- */
  // if (!is_shot_succ_) return state_list;
  // shot_v = min(shot_v, max_vel_);
  // case_id_ = calVelOnShotTraj(shot_v);
  // int seg_num = floor(total_t_ / delta_t);
  // delta_t = total_t_ / double(seg_num); 
  // double tar_len = 0;
  // double q[3];
  // for(double t = delta_t; t < total_t_ + 1e-5; t += delta_t)
  // {
  //   tar_len = getDist(t, shot_v);
  //   dubins_path_sample(&path_, tar_len, q);
  //   Eigen::Vector3d coord(q[0], q[1], ground_height_);
  //   state_list.push_back(coord);
  //   // std::cout << tar_len << std::endl;
  // }


  // double x = 0.0;
  // double move_step_size = max_vel_ * delta_t;
  // while (x <  length){
  //     double q[3];
  //     dubins_path_sample(&path_, x, q);
  //     Eigen::Vector3d coord(q[0], q[1], ground_height_);
  //     state_list.push_back(coord);
  //     x += move_step_size;
  // }
  // double q[3];
  // dubins_path_sample(&path_, length, q);
  // state_list.push_back(Eigen::Vector3d(q[0], q[1], ground_height_));
  
  return state_list;
}

void KinodynamicAstar2D::getDerivatives(vector<Eigen::Vector3d>& start_end_derivatives)
{
  double start_vel_norm = max(start_vel_.norm(), 0.01);
  Eigen::Vector3d start_vel(start_vel_norm * cos(start_yaw_), start_vel_norm * sin(start_yaw_), 0);
  start_end_derivatives.push_back(start_vel);
  Eigen::Vector3d end_vel(0.01 * cos(end_yaw_), 0.01 * sin(end_yaw_), 0);
  start_end_derivatives.push_back(end_vel);
  if(start_vel_.norm() < 5 * 1e-2) start_end_derivatives.push_back(Eigen::Vector3d::Zero());
  else start_end_derivatives.push_back(start_acc_);
  start_end_derivatives.push_back(Eigen::Vector3d::Zero()); //加速度的方向
}


// 返回轨迹上固定数量的采样点
void KinodynamicAstar2D::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  ROS_ERROR("禁用!!!!!!");
  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector2d ut = Vec3ToVec2(node->input);
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= ts)
    {
      stateTransit(x0, xt, ut, ts);
      x0 = xt;
      point_set.push_back(Vector3d(xt(0), xt(1), ground_height_));
    }
    node = node->parent;
  }
  reverse(point_set.begin(), point_set.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    double length = dubins_path_length(&path_);
    double x = 0.0;
    double move_step_size = one_shot_vel_ * ts;
    while (x <  length){
        double q[3];
        dubins_path_sample(&path_, x, q);
        Eigen::Vector3d coord(q[0], q[1], ground_height_);
        point_set.push_back(coord);
        x += move_step_size;
    }
  }
  Eigen::Vector3d start_vel(node->state(3) * cos(node->state(2)), node->state(3) * sin(node->state(2)), 0);
  start_end_derivatives.push_back(start_vel);
  start_end_derivatives.push_back(Eigen::Vector3d::Zero());
  start_end_derivatives.push_back(Eigen::Vector3d::Zero());
  start_end_derivatives.push_back(Eigen::Vector3d::Zero());
}

std::vector<PathNodePtr> KinodynamicAstar2D::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

// search tree已经查过了，根据状态转移方程，第一次扩展就是会沿着原来的方向
vector<Vector4d> KinodynamicAstar2D::getSearchTree()
{
  vector<Vector4d> tree;
  Vector4d point_pair;
  for(int i = 1; i < use_node_num_; ++i)
  {
    // std::cout << "---------mid_states_num_:   " << path_node_pool_[i]->mid_states_num_ << "  ---------" << std::endl;
    // if(path_node_pool_[i]->mid_states_num_ > 6) continue;
    for (int l = 0; l < path_node_pool_[i]->mid_states_num_ - 1; ++l) {
        point_pair.head(2) = path_node_pool_[i]->mid_states_[l].head(2);
        point_pair.tail(2) = path_node_pool_[i]->mid_states_[l + 1].head(2);
        // std::cout << "---------   " << l << "  ---------" << std::endl;
        tree.emplace_back(point_pair);
    }

    // point_pair.head(2) = node->mid_states_[0].head(2);
    // point_pair.tail(2) = node->parent_node_->state_.head(2);
    tree.emplace_back(point_pair);
  }
  return tree;
}

Eigen::Vector3i KinodynamicAstar2D::stateToIndex(Eigen::Vector3d state)
{
  Vector3i idx;
  idx[0] = ((state[0] - origin_(0)) * inv_resolution_);
  idx[1] = ((state[1] - origin_(1)) * inv_resolution_);
  idx[2] = ((state[2] - (-M_PI)) * inv_yaw_resolution_);
  return idx;
}

Eigen::Vector2i KinodynamicAstar2D::stateToIndex2D(Eigen::Vector2d state)
{
  Vector2i idx;
  idx[0] = ((state[0] - origin_(0)) * inv_resolution_);
  idx[1] = ((state[1] - origin_(1)) * inv_resolution_);
  return idx;
}

Eigen::Vector2d KinodynamicAstar2D::indexToState2D(Eigen::Vector2i index)
{
  Vector2d state;
  state[0] = index[0] * resolution_ + origin_(0);
  state[1] = index[1] * resolution_ + origin_(1);
  return state;
}

int KinodynamicAstar2D::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

// state4维，前2维是位置，第3维是朝向，第4维是速度，um是速度标量(有正负)和转向角，tau是时间
void KinodynamicAstar2D::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                      Eigen::Vector2d um, double tau)
{
  Eigen::Vector3d dot_s(um(0) * cos(state0(2)), um(0) * sin(state0(2)), um(0) * tan(um(1)) / wheel_base_);

  state1.head(3) = state0.head(3) + dot_s * tau;
  state1(2) = Mod2Pi(state1(2));
  state1(3) = um(0);
}

double KinodynamicAstar2D::estimateG(const Eigen::Matrix<double, 6, 1>& state0, const int & steering0,
                                     const Eigen::Matrix<double, 6, 1>& state1, const int & steering1,
                                     double ts) const {
    double g;
    // if (neighbor_node_ptr->direction == PathNode::FORWARD) 
    {
        if (steering1 != steering0) {
            if (steering1 == 0) {
                g = state0(3) * ts * steering_change_penalty_;
            } else {
                g = state0(3) * ts * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (steering1 == 0) {
                g = state0(3) * ts;
            } else {
                g = state0(3) * ts * steering_penalty_;
            }
        }
    } 
    // TODO: REVERSE
    // else {
    //     if (neighbor_node_ptr->steering_grade != current_node_ptr->steering_grade) {
    //         if (neighbor_node_ptr->steering_grade == 0) {
    //             g = current_node_ptr->state(3) * ts * steering_change_penalty_ * reversing_penalty_;
    //         } else {
    //             g = current_node_ptr->state(3) * ts * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
    //         }
    //     } else {
    //         if (neighbor_node_ptr->steering_grade == 0) {
    //             g = current_node_ptr->state(3) * ts * reversing_penalty_;
    //         } else {
    //             g = current_node_ptr->state(3) * ts * steering_penalty_ * reversing_penalty_;
    //         }
    //     }
    // }

    return g;
}

//
int KinodynamicAstar2D::calVelOnShotTraj(const double& v0)
{
  int case_id = 0;
  // 这里讨论三种情况
  double minDis1 = 0.5 * v0 * v0 / max_acc_;                      //以当前速度直接减速到0需要的距离
  // double minDis2 = 0.5 * max_vel_ * max_vel_ / max_acc_;            //以最大速度直接减速到0需要的距离
  double minDis2 = 0.5 * max_vel_ * max_vel_ / max_acc_ + 
                   0.5 * (max_vel_ * max_vel_ - v0 * v0) / max_acc_;
  // 如果 total_len_ < minDis1，则直接减速，重新计算所需加速度
  if(total_len_ <= minDis1)
  {
    total_t_ = 2 * total_len_ / v0;
    return 1;
  }
  // 如果 minDis1 < total_len_ < minDis2，则可以先加速再直接减速
  else if(minDis1 < total_len_ && total_len_ <= minDis2)
  {
    v_p_ = sqrt(max_acc_ * total_len_ + 0.5 * v0 * v0);
    t1_ = (v_p_ - v0)/max_acc_;
    t2_ = v_p_/max_acc_;
    total_t_ = t1_ + t2_;
    return 2;
  }
  // 如果 total_len_ > minDis2，则可以直接加速到vel_max，然后匀速一段，然后减速 
  else if(total_len_ > minDis2)
  {
    t1_ = (max_vel_ - v0)/max_acc_;
    t3_ = (max_vel_ - 0)/max_acc_;
    t2_ = (2 * max_acc_ * total_len_ + v0 * v0 - 2 * max_vel_ * max_vel_)/(2 * max_acc_ * max_vel_);
    total_t_ = t1_ + t2_ + t3_;
    return 3;
  }
  return 0;
}

double KinodynamicAstar2D::getDist(const double& t, const double & v0)
{
  double s_tmp = 0.0;
  switch(case_id_)
  {
    case 1:
    {
      s_tmp = v0 * t - 0.25 * (v0 * v0 * t * t) / total_len_;
      break;
    }
    case 2:
    {
      if(0 <= t && t <= t1_) s_tmp = v0 * t + 0.5 * max_acc_ * t * t;
      else if (t1_ < t)
      {
        double s1 = v0 * t1_ + 0.5 * max_acc_ * t1_ * t1_;
        double dt = t - t1_;
        s_tmp = s1 + v_p_ * dt - 0.5 * max_acc_ * dt * dt;
      }
      break;
    }
    case 3:
    {
      if(0 <= t && t <= t1_) s_tmp = v0 * t + 0.5 * max_acc_ * t * t;
      else if(t1_ < t && t <= (t1_ + t2_))
      {
        double s1 = v0 * t1_ + 0.5 * max_acc_ * t1_ * t1_;
        s_tmp = s1 + max_vel_ * (t - t1_);
      }
      else if((t1_ + t2_) < t)
      {
        double s1 = v0 * t1_ + 0.5 * max_acc_ * t1_ * t1_;
        double s2 = max_vel_ * (t2_);
        double dt = t - t1_ - t2_;
        s_tmp = s1 + s2 + max_vel_ * dt - 0.5 * max_acc_ * dt * dt;
      }
      break;
    }
    default:
    {
      ROS_ERROR("Invalid case_id!!!!!");
      break;
    }
  }
  return s_tmp;
}

double KinodynamicAstar2D::calScaleFactor(const double& t, const double & v0, const double& curv_tmp)
{
  double v_tmp, acc_tang = max_acc_;
  switch(case_id_)
  {
    case 1:
    {
      v_tmp = v0 - max_acc_ * t;
      acc_tang = max_acc_;
      break;
    }
    case 2:
    {
      if(0 <= t && t <= t1_) v_tmp = v0 + max_acc_ * t;
      else if (t1_ < t)
      {
        v_tmp = max_vel_ - max_acc_ * (t - t1_);
      }
      break;
    }
    case 3:
    {
      if(0 <= t && t <= t1_) v_tmp = v0 + max_acc_ * t;
      else if(t1_ < t && t <= (t1_ + t2_))
      {
        v_tmp = max_vel_;
        acc_tang = 0.0;
      }
      else if((t1_ + t2_) < t)
      {
        v_tmp = max_vel_ - max_acc_ * (t - t1_ - t2_);
      }
      break;
    }
    default:
    {
      ROS_ERROR("Invalid case_id!!!!!");
      break;
    }
  }
  double acc_normal = v_tmp * v_tmp * curv_tmp ;
  double sacle_factor = 1.0;
  if((acc_normal * acc_normal + acc_tang * acc_tang) > max_acc_ * max_acc_)
  {
    // TODO：这个开起来，会导致速度下降，角速度约束优化的时候出现问题
    sacle_factor = pow(max_acc_ * max_acc_ / (acc_normal * acc_normal + acc_tang * acc_tang), 1.0/10.0);
  }
  return sacle_factor;
}


}  // namespace fast_planner
