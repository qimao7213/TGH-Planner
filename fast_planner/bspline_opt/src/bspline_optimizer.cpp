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



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// #include <glog/logging.h>

// #define LOG_INFO
const double thredHalfFOV = 0.766; // cos(40)

// using namespace std;
namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3); //如果有这一项，说明没有reach end，最后几个控制点是可以调整的
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);
const int BsplineOptimizer::NONE        = (1 << 7);
const int BsplineOptimizer::FEASIBILITYYaw = (1 << 8);
const int BsplineOptimizer::PERCEPTION  = (1 << 9);
const int BsplineOptimizer::CTRLPTYAW   = (1 << 10);
const int BsplineOptimizer::CTRLPTYAWS2   = (1 << 11);
const int BsplineOptimizer::CTRLPTYAWS3   = (1 << 12);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
// const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS;
const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);  // smoothess
  nh.param("optimization/lambda2", lambda2_, -1.0);  // distance 
  nh.param("optimization/lambda3", lambda3_, -1.0);  // feasibility
  nh.param("optimization/lambda4", lambda4_, -1.0);  // endpoints
  nh.param("optimization/lambda5", lambda5_, -1.0);  // guide    -- topo path guide
  nh.param("optimization/lambda6", lambda6_, -1.0);  // perception
  nh.param("optimization/lambda7", lambda7_, -1.0);  // waypoint -- yaw plan
  nh.param("optimization/lambda8", lambda8_, -1.0);  // feasibility yaw
  nh.param("optimization/lambda9", lambda9_, -1.0);  // ctrl points yaw
  nh.param("optimization/lambda10", lambda10_, -1.0);  // smoothness weight for ctrl points yaw S2
  nh.param("optimization/w_ctrl_in_perception", w_ctrl_in_perception_, -1.0); // ctrl points yaw 在perception阶段的权重

  nh.param("optimization/dist0", dist0_, -1.0);          // 0.2m
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);      
  nh.param("optimization/visib_min", visib_min_, -1.0);  // what?
  nh.param("optimization/dlmin", dlmin_, -1.0);          // delta_v
  nh.param("optimization/wnl", wnl_, -1.0);              // weight of safe term
  nh.param("optimization/phi_min", phi_min_, -1.0);      // 0.25，预定义的可见性等级

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_num5", max_iteration_num_[4], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);

  double steering_angle = nh.param("search_2D/steering_angle", 60);
  double wheel_base = nh.param("search_2D/wheel_base", 0.6); 
  min_r_ = wheel_base / tan(steering_angle * M_PI / 180.0);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
  if (cost_function_ & FEASIBILITYYaw) cost_str += " feasi_yaw |";
  if (cost_function_ & PERCEPTION) cost_str += " perception |";
  if (cost_function_ & CTRLPTYAW) cost_str += " ctrl_pt_yaw |";
  if (cost_function_ & CTRLPTYAWS2) cost_str += " ctrl_pt_yaw_s2 |";
  if (cost_function_ & CTRLPTYAWS3) cost_str += " ctrl_pt_yaw_s3 |";

  // ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::setYawDotConstrains(const vector<double>& yaw_dot_constraints) {
  yaw_dot_constraints_ = yaw_dot_constraints;
}

void BsplineOptimizer::setInitBspline(const Eigen::MatrixXd& points, const double &ts)
{
    bspline_init_ = NonUniformBspline(points, 3, ts);
    double t_duration = bspline_init_.getTimeSum();
    bspline_pts_.clear();
    bspline_pts_.reserve((t_duration / t_sample_) + 10);
    for(double t = 0.0; t < t_duration + t_sample_; t += t_sample_)
    {
        bspline_pts_.emplace_back(bspline_init_.evaluateDeBoorT(t).head(3));
    }
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id, int fix_num_start, int fix_num_end,
                                                      bool if_printf) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);
  if(fix_num_start != -1) fix_num_start_ = fix_num_start;
  if(fix_num_end != -1)   fix_num_end_   = fix_num_end;

  fix_num_start_ = std::max(order_, fix_num_start_);
  // if(cost_function_ & CTRLPTYAW && ! (cost_function_ & CTRLPTYAWS2)) fix_num_end_ = 1;
  fix_num_end_   = ((cost_function_ & ENDPOINT)) ? std::max(0, fix_num_end_) : std::max(order_, fix_num_end_);
  if_printf_ = if_printf;
  if(cost_function_ & CTRLPTYAW ) lambda3_ = 0.005;  
  if(cost_function_ & CTRLPTYAWS2) lambda1_ = lambda10_;
  if(cost_function_ & PERCEPTION ) 
  {
    lambda1_ = 0.13 * lambda10_;
    lambda3_ = 0.002; 
  }
  flag_out_ = 0;

  if(!(cost_function_ & PERCEPTION)) 
  {
    optimize();
    return this->control_points_;
  }
  if_printf_ = false;
  //重置这三个量的状态。不然可视化的时候会出错
  unknown_ = false, visible_ = true, safty_ = true;
  setInitBspline(points, ts);
  unknown_ = FrontierIntersection();
  if(!unknown_) return this->control_points_; //如果没有发现未知区域，就直接返回
  visible_ = CriticalView();
  if(visible_)                return this->control_points_; //如果整条路径都是visible的，就直接返回
  
  Eigen::Vector3d vel_c = bspline_init_.getDerivative().evaluateDeBoor(tc_);
  double df_c = bspline_init_.getLength(tc_, tf_, t_sample_); // 用这个曲线的dis
  safty_ = checkSafty(vel_c.norm(), df_c);
  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  // 这个应该关闭，不然很多时都不会触发调整
  // if(safty_)                  return this->control_points_; //如果safe的，即可以再最坏情况下停下来，则直接返回
  
  // 注意区分，vel_c是c点的速度，用来判断是不是需要调整c点的
  // 而vel_s_bar_是调整点s处的速度，只不过目前还不知道s具体是谁，所以大致设置了一个平均速度
  // ds_bar_是用来寻找s的
  // 这一部分比较乱，但是最后调整的那个点，是pfov_
  v_c_ = (pc_ - pf_).normalized();
  vel_s_bar_ = bspline_init_.getLength(0.0, tf_, t_sample_) / (tf_ + 1e-5);
  ds_bar_ = R_q_ + 0.5 * vel_s_bar_ * vel_s_bar_ /max_acc_;
  ts_ = findInitTs();
  // if(cost_function)
  this->cost_perception_ = 0.0; this->cost_perception_last_ = 0.0;
  // 这里把ts_算出来了，那就可以正式开始接下来的优化了。
  // 但是我先检查前面的信息，可视化，先不要进行optimize()；
  if_printf_ = true;
  if_printf_ = if_printf_ && if_printf;
  optimize();
  if(cost_function_ & PERCEPTION && iter_num_ >= 1) flag_out_ = 1;
  return this->control_points_;
}

// 这里主要是构造NLopt的求解器，最重要的是constFunction，这里面定义了各个
void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_q_last_ = g_q_;
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);
  g_feasibility_yaw_.resize(pt_num);
  g_perception_.resize(pt_num);
  g_ctrl_pt_yaw_.resize(pt_num);
  x_var_vec_.resize(pt_num);
  std::fill(x_var_vec_.begin(), x_var_vec_.end(), 0);

  this->stop_signal_ = false;
  this->stop_signal_last_ = false; 

  variable_num_ = dim_ * (pt_num - fix_num_start_ - fix_num_end_); 
  if(variable_num_ <= 0) return;
  if (cost_function_ & ENDPOINT) {    
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } 

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  if(cost_function_ & PERCEPTION) opt.set_min_objective(BsplineOptimizer::costFunctionPerception, this);
  // else if(cost_function_ & CTRLPTYAW) opt.set_min_objective(BsplineOptimizer::costFunctionCtrlPtYaw, this);
  else opt.set_min_objective(BsplineOptimizer::costFunction, this);

  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);
  if(cost_function_ & PERCEPTION 
  // || cost_function_ & CTRLPTYAW
  ) 
  {
    opt.set_maxeval(15);
    // opt.set_xtol_rel(1e-3);
    // opt.set_ftol_abs(1e-3);
  } 

  vector<double> q(variable_num_);
  for (int i = fix_num_start_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - fix_num_end_) continue;
    for (int j = 0; j < dim_; ++j) {
      q[dim_ * (i - fix_num_start_) + j] = control_points_(i, j);
    }
  }

  //设置变量的上下界
  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost); // result check
    // ROS_WARN_STREAM("Opt success by: " << result);
    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
    if(cost_function_ & FEASIBILITYYaw) flag_out_ = 1;
  } 
  catch (nlopt::forced_stop &e) {
    ROS_WARN("[Optimization]: stopped after cost tolerances were met");
    cout << e.what() << endl;
  }  
  catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = fix_num_start_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - fix_num_end_) continue;
    for (int j = 0; j < dim_; ++j) {
      control_points_(i, j) = best_variable_[dim_ * (i - fix_num_start_) + j];
    }
  }

  // if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; ++i) {
    /* evaluate jerk */
    // 论文里面的cost其实是acc，
    // 但这里其实是jerk。这个公式很好推导
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    // if(cost_function_ & CTRLPTYAW) std::cout << "jerk: " << jerk.transpose() << std::endl;
    if(cost_function_ & CTRLPTYAWS2 && (i < (fix_num_start_ -2) || i > (q.size() - fix_num_end_ - 4))) 
      continue;
    else
      cost += jerk.squaredNorm(); // squaredNorm()是norm()的平方
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    // 对jerk的平方求导
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
  // if(cost_function_ & CTRLPTYAW) std::cout << "smooth term cost: " << cost << std::endl;
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - fix_num_end_;

  for (int i = fix_num_start_; i < end_idx; ++i) {
      edt_environment_->evaluateEDTWithGrad2D(q[i], -1.0, dist, dist_grad);

    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    // 只有在和障碍物的距离小于安全距离时，才加上障碍物项
    // 我发现这两个的差一般都是在0.05～0.2之间，如果直接取平方，会让比较小的那个值变得更小，从而容易被忽略
    if((cost_function_ & PERCEPTION))
    {
      if (dist < dist0_) {
        cost += pow(dist - dist0_, 2);
        gradient[i] += 2.0 * (dist - dist0_) * dist_grad * 0.5;
      }      
    }
    else
    {
      //下面使用幂函数构造cost, 但是这样容易造成一个很大的梯度
      double cost_tmp;
      if (dist < dist0_) {
        cost_tmp = pow(-(dist - dist0_), 0.5);
        cost_tmp = max(cost_tmp, 0.2);
        cost += cost_tmp;
        gradient[i] += -0.5 / cost_tmp * dist_grad; // 我曹，你这个梯度方向不久是反了吗
      }      
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; ++i) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; ++j) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; ++j) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
  for(int i = 0; i < gradient.size(); ++i)
  {
    for (int j = 0; j < 3; ++j) {
      if(gradient[i][j] < -0.3/lambda3_) gradient[i][j] = -0.3/lambda3_;
      else if(gradient[i][j] > 0.3/lambda3_) gradient[i][j] = 0.3/lambda3_;
    }
  }
}

void BsplineOptimizer::calcFeasibilityYawCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 1; i < q.size() - 2; ++i) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; ++j) {
      double vd = vi(j) * vi(j) * ts_inv2 - yaw_dot_constraints_[i] * yaw_dot_constraints_[i];
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  // /* acceleration feasibility */
  // for (int i = 0; i < q.size() - 2; ++i) {
  //   Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

  //   for (int j = 0; j < 3; ++j) {
  //     double ad = ai(j) * ai(j) * ts_inv4 - am2;
  //     if (ad > 0.0) {
  //       cost += pow(ad, 2);

  //       double temp_a = 4.0 * ad * ts_inv4;
  //       gradient[i + 0](j) += temp_a * ai(j);
  //       gradient[i + 1](j) += -2 * temp_a * ai(j);
  //       gradient[i + 2](j) += temp_a * ai(j);
  //     }
  //   }
  // }
}

// 末端控制点到enc_point的距离，让轨迹尽可能靠近终点
// 当混合A*的搜索结果不是KinodynamicAstar::REACH_END，触发这一项
void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

// 这里使用了局部加权，可以让路径更加平滑；而GUIDE的，只负责牵引，而不管是否平滑，因为后面还有第二次优化
// 因次，为了加入XX抵抗XX，我们使用这一种cost
void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - fix_num_end_;

  for (int i = fix_num_start_; i < end_idx; ++i) {
    Eigen::Vector3d gpt = guide_pts_[i - fix_num_start_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt); // 下降的方向是负梯度方向
  }
}

// void BsplineOptimizer::calcPerceptionCost(const vector<Eigen::Vector3d>& q, double& cost,
//                                           vector<Eigen::Vector3d>& gradient) {
//   cost = 0.0;
//   Eigen::Vector3d zero(0, 0, 0);
//   std::fill(gradient.begin(), gradient.end(), zero);

//   // 首先构造一个B样条
//   NonUniformBspline bspline_traj(vectorsToMatrix(q), order_, bspline_interval_);

//   // 计算曲线上的平均速度，然后计算ds
//   // vel_s_bar_ = min(1.0 * max_vel_ , vel_s_bar_);
//   vel_s_bar_ = 1.05 * 1.6;
//   double ds = 0.5 * vel_s_bar_ * vel_s_bar_ / max_acc_ + R_q_; 

//   // 然后找到ts和p_ts
  
//   double ts = ts_;
//   Eigen::Vector3d p_ts = bspline_traj.evaluateDeBoorT(ts);
//   double v_ts_squr = bspline_traj.getDerivative().evaluateDeBoorT(ts).squaredNorm();
//   double d_sf = bspline_traj.getLength(ts, tf_, 0.05);
//   this->cost_perception_last_ = this->cost_perception_;
//   this->cost_perception_ = max(0.0, (0.5 * vel_s_bar_ * vel_s_bar_ / max_acc_) - (d_sf - R_q_)); 
//   // 然后找到该ts所在的区间和控制点
//   vector<double> ctrl_point_weights;
//   vector<int> ctrl_point_idx;
//   bspline_traj.getControlPointIdxAndWeight(ctrl_point_idx, ctrl_point_weights, ts);
//   // ctrl_point_idx_ = ctrl_point_idx;
//   // if(if_printf_)
//   // {
//   //   std::cout << "idx   : " << ctrl_point_idx[0] << ", " << 
//   //             ctrl_point_idx[1] << ", " << ctrl_point_idx[2] << ", " << ctrl_point_idx[3] << "\n";
//   //   std::cout << "weight: " << ctrl_point_weights[0] << ", " << 
//   //             ctrl_point_weights[1] << ", " << ctrl_point_weights[2] << ", " << ctrl_point_weights[3] << "\n";
//   // }

//   Eigen::Vector3d dv = calDv(p_ts);
//   double dv_norm = dv.norm();
//   double cost1 = (dv_norm > dlmin_ ) ? ((dv_norm - dlmin_) * (dv_norm - dlmin_)) : 0.0; // vis的cost
//   double cost2 = (d_sf < ds) ? ((d_sf - ds) * (d_sf - ds)) : 0.0;                       // dis的cost
//   if(if_printf_) std::cout << "dv_norm: " << dv_norm << ", d_sf: " << d_sf << ", v_ts_squr: " << v_ts_squr << ", ds: " << ds << ", vel_s_bar_: " << vel_s_bar_ << std::endl;
//   if(if_printf_) std::cout << "cost: " << cost1 << ", " << cost2 << std::endl; 
//   // if(!(cost_function_ & CTRLPTYAWS3))
//   { 
//     cost += cost1 + wnl_ * cost2;
//     Eigen::Vector3d gradient1 = Eigen::Vector3d::Zero();
//     Eigen::Vector3d gradient2 = Eigen::Vector3d::Zero();
//     if(dv_norm > dlmin_)
//     {
//       gradient1 = 2*(dv_norm-dlmin_)*dv.transpose() * (Eigen::Matrix3d::Identity()-v_c_*v_c_.transpose())/dv_norm;
//     }
//     if(d_sf < ds)
//     {
//       // 我的妈呀，论文里面这个公式也错了
//       gradient2 = 2*(d_sf-ds)*(p_ts-pf_).transpose()/d_sf;
//     }
//     for(int i = 0; i < ctrl_point_idx.size(); ++i)
//     {
//       gradient[ctrl_point_idx[i]] += (gradient1  + wnl_ * gradient2) * ctrl_point_weights[i];
//     }
//   }
//   vel_s_bar_ *= 1.05;

//   // 下面是考虑调整ps所在点相关的控制点的角速度来实现感知的加强
//   if(!(cost_function_ & CTRLPTYAWS3)) return;

//   vector<Eigen::Vector3d> gradient_tmp(gradient.size(), Eigen::Vector3d::Zero());
//   Eigen::Vector3d vA, vB;
//   double vAnorm, vBnorm, vABnorm;
//   double m, mm, constraint, acos_m; //中间值

//   static const double  dt      = bspline_interval_;
//   static const double  dt_inv2 = 1.0 / dt / dt;
//   static const vector<int> idx{0, 1, 2};
//   static const double scale = 1.05; // 每次增加1.05的角速度
//   for(int i = 0; i < ctrl_point_idx.size(); ++i)
//   {
//     int id = ctrl_point_idx[i];
//     if(id < fix_num_start_ || (id+2) > (q.size() - fix_num_end_)) continue;
//     vA = q.at(id + 1) - q.at(id + 0);  //前一个vec
//     vB = q.at(id + 2) - q.at(id + 1);
//     vAnorm = vA.norm(); vBnorm = vB.norm();
//     vABnorm = vAnorm * vBnorm;        
//     // theta1 - theta0的值大小就是两个向量的夹角，然后再通过旋转方向来判断正负
//     m = (vA.dot(vB) / (vAnorm * vBnorm));
//     acos_m = std::acos(m);
//     mm = -2 * acos_m / (sqrt(1 - m * m));
//     for(int j = 0; j < idx.size(); ++j)
//     {
//         switch (idx[j])
//         {
//         case 0:
//         {
//           gradient_tmp[id + idx[j]] = dt_inv2 * ((mm * 
//                                      (-vB * vABnorm + vA * vBnorm / vAnorm * (vA.dot(vB))) / vABnorm / vABnorm)
//                                       ) * 0.1025 * ctrl_point_weights[i];
//           break;
//         }
//         case 1:
//         {
//           gradient_tmp[id + idx[j]] = dt_inv2 * ((mm * 
//                                      ((vB - vA) * vABnorm - (vA * vBnorm / vAnorm - vB * vAnorm / vBnorm) * (vA.dot(vB))) / vABnorm / vABnorm)
//                                       ) * 0.1025 * ctrl_point_weights[i];
//           break;
//         }          
//         case 2:
//         {
//           gradient_tmp[id + idx[j]] = dt_inv2 * ((mm * 
//                                      (vA * vABnorm + vB * vAnorm / vBnorm * (vA.dot(vB))) / vABnorm / vABnorm)
//                                       ) * 0.1025 * ctrl_point_weights[i];
//           break;
//         }       
//         default:
//         {
//           ROS_ERROR("invlade idx");
//           break;
//         }
//         }

//     }
//   }
//   for(int i = 0; i < gradient.size(); ++i)
//   {
//     gradient[i] += w_ctrl_in_perception_ * gradient_tmp[i]; //注意这个正负。-0.05是权重
//     if(if_printf_) std::cout << "idx: " << i << ", gradient ctrl pt yaw s3: " << gradient_tmp[i].transpose() << std::endl;
//   }
// }

void BsplineOptimizer::calcPerceptionCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  if(pfov_(2) == 10) return; // 没有找到需要调整的点

  NonUniformBspline bspline_traj(vectorsToMatrix(q), order_, bspline_interval_);
  vector<double> ctrl_point_weights;
  vector<int> ctrl_point_idx;
  bspline_traj.getControlPointIdxAndWeight(ctrl_point_idx, ctrl_point_weights, tfov_);
  // for (int i = fix_num_start_; i < end_idx; ++i) {
  //   Eigen::Vector3d gpt = guide_pts_[i - fix_num_start_];
  //   cost += (q[i] - gpt).squaredNorm();
  //   gradient[i] += 2 * (q[i] - gpt);
  // }

  Eigen::Vector3d pfov_new = bspline_traj.evaluateDeBoor(tfov_);
  double pfov_change = (pfov_new - pfov_).norm();
  // 不能改变太多了
  if (pfov_change > 0.5) return; 
  // ROS_WARN_STREAM("pfov_change: " << pfov_change);


  for(int i = 0; i < ctrl_point_idx.size(); ++i)
  {
    int id = ctrl_point_idx[i];
    if(id < fix_num_start_ || (id+2) > (q.size() - fix_num_end_)) continue;
    gradient[id] += -0.1 * v_fov_ * ctrl_point_weights[i] * 30; // 最后一个是权重
  }
  // return;

  // 下面是调整角速度，是有效的。
  vector<Eigen::Vector3d> gradient_tmp(gradient.size(), Eigen::Vector3d::Zero());
  Eigen::Vector3d vA, vB;
  double vAnorm, vBnorm, vABnorm;
  double m, mm, constraint, acos_m; //中间值
  static const double  dt      = bspline_interval_;
  static const double  dt_inv2 = 1.0 / dt / dt;
  static const vector<int> idx{0, 1, 2};
  static const double scale = 1.05; // 每次增加1.05的角速度
  for(int i = 0; i < ctrl_point_idx.size(); ++i)
  {
    int id = ctrl_point_idx[i];
    if(id < fix_num_start_ || (id+2) > (q.size() - fix_num_end_)) continue;
    vA = q.at(id + 1) - q.at(id + 0);  //前一个vec
    vB = q.at(id + 2) - q.at(id + 1);
    vAnorm = vA.norm(); vBnorm = vB.norm();
    vABnorm = vAnorm * vBnorm;        
    m = (vA.dot(vB) / (vAnorm * vBnorm));
    acos_m = std::acos(m);
    mm = -2 * acos_m / (sqrt(1 - m * m));
    for(int j = 0; j < idx.size(); ++j)
    {
        switch (idx[j])
        {
        case 0:
        {
          gradient_tmp[id + idx[j]] = dt_inv2 * ((mm * 
                                     (-vB * vABnorm + vA * vBnorm / vAnorm * (vA.dot(vB))) / vABnorm / vABnorm)
                                      ) * 0.1025 * ctrl_point_weights[i];
          break;
        }
        case 1:
        {
          gradient_tmp[id + idx[j]] = dt_inv2 * ((mm * 
                                     ((vB - vA) * vABnorm - (vA * vBnorm / vAnorm - vB * vAnorm / vBnorm) * (vA.dot(vB))) / vABnorm / vABnorm)
                                      ) * 0.1025 * ctrl_point_weights[i];
          break;
        }          
        case 2:
        {
          gradient_tmp[id + idx[j]] = dt_inv2 * ((mm * 
                                     (vA * vABnorm + vB * vAnorm / vBnorm * (vA.dot(vB))) / vABnorm / vABnorm)
                                      ) * 0.1025 * ctrl_point_weights[i];
          break;
        }       
        default:
        {
          ROS_ERROR("invlade idx");
          break;
        }
        }

    }
  }
  for(int i = 0; i < gradient.size(); ++i)
  {
    gradient[i] += gradient_tmp[i]; //注意这个正负
    if(if_printf_) std::cout << "idx: " << i << ", gradient ctrl pt yaw s3: " << gradient_tmp[i].transpose() << std::endl;
  }

}

void BsplineOptimizer::calcCtrlPtYawCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d vA, vB;
  double vAnorm, vBnorm, vABnorm;
  double m, mm, constraint, acos_m; //中间值
  vector<double> cost_tmp(gradient.size(), 0.0);
  vector<Eigen::Vector3d> gradient_tmp(gradient.size(), Eigen::Vector3d::Zero());

  static const double  ts      = bspline_interval_;
  static const double  ts_inv2 = 1.0 / ts / ts;
  static const vector<int> idx{0, 1, 2};
  // 前3个不动
  // 使用前向差分来计算当前点的角速度.计算的是当前i的！！
  // 使用前向差分来计算当前点的速度和角速度限制
  // TODO: 最后一个应该到哪里，还需要再考虑一下
  for(int i = 0; i < q.size() - 2; ++i)
  {
    vA = q.at(i + 1) - q.at(i + 0);  //前一个vec
    vB = q.at(i + 2) - q.at(i + 1);
    // theta1 - theta0的值大小就是两个向量的夹角，然后再通过旋转方向来判断正负
    m = (vA.dot(vB) / (vB.norm() * vA.norm()));
    acos_m = std::acos(m);
    mm = -2 * acos_m / (sqrt(1 - m * m));
    
    constraint = vA.squaredNorm();
    if(acos_m * acos_m > constraint / min_r_ / min_r_)
    {
      cost_tmp[i] = ts_inv2 * (acos_m * acos_m - constraint / min_r_ / min_r_);
      vAnorm = vA.norm(); vBnorm = vB.norm();
      vABnorm = vAnorm * vBnorm;
      for(int j = 0; j < idx.size(); ++j)
      {
        switch (idx[j])
        {
        case 0:
        {
          gradient_tmp[i + idx[j]] = ts_inv2 * ((mm * 
                                     (-vB * vABnorm + vA * vBnorm / vAnorm * (vA.dot(vB))) / vABnorm / vABnorm)
                                      + 2 * vA / min_r_ / min_r_);
          break;
        }
        case 1:
        {
          gradient_tmp[i + idx[j]] = ts_inv2 * ((mm * 
                                     ((vB - vA) * vABnorm - (vA * vBnorm / vAnorm - vB * vAnorm / vBnorm) * (vA.dot(vB))) / vABnorm / vABnorm)
                                      - 2 * vA / min_r_ / min_r_);
          break;
        }          
        case 2:
        {
          gradient_tmp[i + idx[j]] = ts_inv2 * ((mm * 
                                     (vA * vABnorm + vB * vAnorm / vBnorm * (vA.dot(vB))) / vABnorm / vABnorm)
                                      );
          break;
        }       
        default:
        {
          ROS_ERROR("invlade idx");
          break;
        }
        }
      }
    }
    
  }
  // std::cout << "----CtrlPtYawCost: cost and gradient: \n";
  for(int i = 0; i < cost_tmp.size(); ++i)
  {
    // std::cout << "cost: " << cost_tmp[i] << ", gradient: " << gradient_tmp[i].transpose() << std::endl;
    #ifdef LOG_INFO
      if(if_printf_) LOG(INFO) << "idx: " << i << ", cost: " << cost_tmp[i] << ", gradient: " << gradient_tmp[i](0) << ", " << gradient_tmp[i](1) << std::endl;
    #endif
    gradient[i] += gradient_tmp[i];
    if(gradient_tmp[i].squaredNorm() > 1e-3 && !this->stop_signal_) x_var_vec_[i] ++;
  }
  cost += std::accumulate(cost_tmp.begin(), cost_tmp.end(), 0.0);
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */
  g_q_last_ = g_q_;

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  // g_q_就是控制点，只不过里面存放了原始的（固定了初始和最后的几个点不优化）和优化后的控制点（变量x）

  // 这里，我要调整一下，对于两阶段优化的策略
  // 如果第一阶段（ctrl points yaw）结束了，那么我会把这阶段的结果存起来，然后固定开头和结尾的一部分
  // 下面在combineCost的时候，也应该只考虑一部分
  // if(iter_num_ == 0 && if_printf_)
  // {
  //   std::cout << "control_pts: \n" << control_points_.matrix() << std::endl;
  // }
  for (int i = 0; i < fix_num_start_; ++i) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; ++i) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + fix_num_start_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + fix_num_start_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < fix_num_end_; ++i) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[fix_num_start_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - fix_num_end_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[fix_num_start_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }
  if(cost_function_&PERCEPTION && iter_num_ == 1)
  {
    ctrl_pt_mid_ = vectorsToMatrix(g_q_);
  }
  if(if_printf_ && 1)
  {
    // std::cout << "------------ iter num: " << iter_num_ << std::endl;
    #ifdef LOG_INFO
    LOG(INFO) << std::endl; 
    LOG(INFO) << "------------ iter num: " << iter_num_ << std::endl;
    LOG(INFO) << "------- ctrl points at " << iter_num_ << " ---------" << std::endl;
    for(int i = 0; i < g_q_.size(); ++i)
    {
      LOG(INFO) << "idx: " << i << ", last: " 
                << g_q_last_[i](0) << ", " 
                << g_q_last_[i](1) << ", " 
                << g_q_last_[i](2) << ", " 
                << ", curr: " 
                << g_q_[i](0) << ", " 
                << g_q_[i](1) << ", " 
                << g_q_[i](2) << ", "  
                << std::endl;
    }
    LOG(INFO) << "x_var_: " << x_var_;
    #endif
  }
  if(cost_function_ & CTRLPTYAW && iter_num_ != 0)
  {
    x_var_ = 0.0;  
    for(int i = 0; i < g_q_.size(); ++i)
    {
      x_var_ += (g_q_last_[i] - g_q_[i]).norm();//注意这个不要被注释了
    }
    // std::cout << "x_var_: " << x_var_ << std::endl;;    
  }

  if(iter_num_ <= 3) x_var_ = 100.0;

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, 
         f_waypoints, f_feasibility_yaw, f_perception, f_ctrl_pt_yaw;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;
  f_feasibility_yaw = f_perception = f_ctrl_pt_yaw = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + fix_num_start_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda2_ * g_distance_[i + fix_num_start_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + fix_num_start_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + fix_num_start_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda5_ * g_guide_[i + fix_num_start_](j);
  }
  if (cost_function_ & PERCEPTION) {
    calcPerceptionCost(g_q_, f_perception, g_perception_);
    f_combine += lambda6_ * f_perception;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda6_ * g_perception_[i + fix_num_start_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + fix_num_start_](j);
  }
  if (cost_function_ & FEASIBILITYYaw) {
    calcFeasibilityYawCost(g_q_, f_feasibility_yaw, g_feasibility_yaw_);
    f_combine += lambda8_ * f_feasibility_yaw;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda8_ * g_feasibility_yaw_[i + fix_num_start_](j);
  }
  if (cost_function_ & CTRLPTYAW) {
    calcCtrlPtYawCost(g_q_, f_ctrl_pt_yaw, g_ctrl_pt_yaw_);
    f_combine += lambda9_ * f_ctrl_pt_yaw;
    for (int i = 0; i < variable_num_ / dim_; ++i)
      for (int j = 0; j < dim_; ++j) grad[dim_ * i + j] += lambda9_ * g_ctrl_pt_yaw_[i + fix_num_start_](j);
  }

  
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  if(if_printf_){
    std::ostringstream oss;
    if(cost_function_ & SMOOTHNESS) oss << "smooth: " <<  lambda1_ * f_smoothness << "| ";
    if(cost_function_ & DISTANCE)   oss << "dis: " << lambda2_ * f_distance << "| ";
    if(cost_function_ & FEASIBILITY) oss << "feasi: " << lambda3_ * f_feasibility << "| ";
    if(cost_function_ & PERCEPTION)  oss << "perception: " <<  lambda6_ * f_perception << "| ";
    if(cost_function_ & CTRLPTYAW) oss << "ctrl_yaw: " <<  lambda9_ * f_ctrl_pt_yaw << "| ";
    if(cost_function_ & CTRLPTYAWS2) oss << "ctrl_yaw_s2: " <<  lambda10_ * f_smoothness << "| ";
    // std::cout << oss.str() << std::endl;
    #ifdef LOG_INFO
      LOG(INFO) << oss.str() << std::endl;
    #endif
    for(int i = 0; i < g_distance_.size(); ++i)
    {
      // std::cout << "idx: " << i << "| ";
      // if(cost_function_ & SMOOTHNESS) std::cout << "smooth: " <<  lambda1_ * g_smoothness_[i].transpose() << "| ";
      // if(cost_function_ & DISTANCE)   std::cout << "dis: " << lambda2_ * g_distance_[i].transpose() << "| ";
      // if(cost_function_ & FEASIBILITY) std::cout << "feasi: " << lambda3_ * g_feasibility_[i].transpose() << "| ";
      // if(cost_function_ & PERCEPTION)  std::cout << "perception: " <<  lambda6_ * g_perception_[i].transpose() << "| ";
      // if(cost_function_ & CTRLPTYAW) std::cout << "ctrl_yaw: " <<  lambda9_ * g_ctrl_pt_yaw_[i].transpose() << "| ";
      // if(cost_function_ & CTRLPTYAWS2) std::cout << "ctrl_yaw_s2: " <<  lambda10_ * g_smoothness_[i].transpose() << "| ";
      // std::cout << std::endl;
      #ifdef LOG_INFO
        LOG(INFO) << "idx: " << i << ", "
          << ((cost_function_ & SMOOTHNESS) ? ("smooth: " + std::to_string(lambda1_ * g_smoothness_[i](0) ) + ", " + std::to_string(lambda1_ * g_smoothness_[i](1) ) + "| ") : "")
          << ((cost_function_ & DISTANCE) ? ("dis: " + std::to_string(lambda2_ * g_distance_[i](0) ) + ", " + std::to_string(lambda2_ * g_distance_[i](1) ) + "| ") : "")
          << ((cost_function_ & FEASIBILITY) ? ("feasi: " + std::to_string(lambda3_ * g_feasibility_[i](0) ) + ", " + std::to_string(lambda3_ * g_feasibility_[i](1) ) + "| ") : "")
          << ((cost_function_ & PERCEPTION) ? ("perception: " + std::to_string(lambda6_ * g_perception_[i](0) ) + ", " + std::to_string(lambda6_ * g_perception_[i](1) ) + "| ") : "")
          << ((cost_function_ & CTRLPTYAW) ? ("ctrl_yaw: " + std::to_string(lambda9_ * g_ctrl_pt_yaw_[i](0) ) + ", " + std::to_string(lambda9_ * g_ctrl_pt_yaw_[i](1) ) + "| ") : "")
          << ((cost_function_ & CTRLPTYAWS2) ? ("ctrl_yaw_s2: " + std::to_string(lambda10_ * g_smoothness_[i](0) ) + ", " + std::to_string(lambda10_ * g_smoothness_[i](1) ) + "| ") : "")
          << std::endl;
      #endif
    }
  }
}

// 如果含有PERCEPTION项，则不能使用这个，因为cost会一直增加，从而不会赋值
// 这里会返回一个cost，这个是给到nlopt用来判断是不是cost下降的，如果下降才会更新变量x??
double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;
  // if(opt->if_printf_) std::cout << "curr cost: " << cost << ", min cost: " << opt->min_cost_ << std::endl;
  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
    // if(opt->if_printf_) ROS_WARN("---------update best_variable--------");
  }
  else
  {
    // if(opt->if_printf_) ROS_WARN("-------- No Update --------");
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

double BsplineOptimizer::costFunctionCtrlPtYaw(const std::vector<double>& x, std::vector<double>& grad,
                                               void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;
  // std::cout << "curr cost: " << cost << ", min cost: " << opt->min_cost_ << std::endl;
  /* save the min cost result */
  if (cost < opt->min_cost_ && !opt->stop_signal_) {
    // ROS_WARN("---------update best_variable--------");
    #ifdef LOG_INFO
    if(opt->if_printf_) LOG(INFO) << "---------update best_variable--------";
    #endif
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  else 
  {
    // ROS_WARN("-------- No Update --------");
    #ifdef LOG_INFO
    if(opt->if_printf_) LOG(INFO) << "---------No Update--------";
    #endif
  }
  opt->stop_signal_last_ = opt->stop_signal_;
  std::ostringstream oss;
  for (const auto& num : opt->x_var_vec_) {
      oss << num << ", ";
  }
  // std::cout << "---- x_var_vec_: " << oss.str() << std::endl;
  #ifdef LOG_INFO
  if(opt->if_printf_) LOG(INFO) << "---- x_var_vec_: " << oss.str() << std::endl;
  if(opt->if_printf_) LOG(INFO) << "fix_num_start_: " << opt->fix_num_start_ << ", fix_num_end_: " << opt->fix_num_end_ << std::endl;
  #endif
  if (opt->x_var_ <= 1e-5) {
      // ROS_WARN("----------- should be stop!!! -------------");
      #ifdef LOG_INFO
      if(opt->if_printf_) LOG(INFO) << "--------- should be stop!!! --------";
      #endif
      opt->stop_signal_ = true;
      throw nlopt::forced_stop();
  }
  return cost;
}

double BsplineOptimizer::costFunctionPerception(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;
  cost = opt->cost_perception_;
  
  /* save the min cost result */
  if (!opt->stop_signal_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  // 应该在这里设置终止条件，设置为 什么的cost
  #ifdef LOG_INFO
  if(opt->if_printf_) LOG(INFO) << "cost_perception_last: " << opt->cost_perception_last_ << ", cost_perception_: " << opt->cost_perception_ << std::endl;
  #endif
  if(opt->if_printf_)
  {
    std::cout << "curr cost: " << cost << ", min cost: " << opt->min_cost_ << std::endl;
    std::cout << "cost_perception_last: " << opt->cost_perception_last_ << ", cost_perception_: " << opt->cost_perception_ << std::endl;
  }
  if (opt->iter_num_ >= 15) {
      // ROS_WARN("----------- should be stop!!! -------------");
      #ifdef LOG_INFO
      if(opt->if_printf_) LOG(INFO) << "--------- should be stop!!! --------";
      #endif
      opt->stop_signal_ = true;
      std::fill(grad.begin(), grad.end(), 0.0);
      throw nlopt::forced_stop();
  }
  // return opt->cost_perception_;
  return 100 * pow(0.95, opt->iter_num_);
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::vectorsToMatrix(const vector<Eigen::Vector3d>& ctrl_q) {
  Eigen::MatrixXd ctrl_pts(ctrl_q.size(), 3);  // 行数为向量的数量，列数为3
  for (int i = 0; i < ctrl_q.size(); ++i) {
    ctrl_pts.row(i) = ctrl_q[i];  // 将每个Eigen::Vector3d赋值到矩阵的对应行
  }
  return ctrl_pts;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

bool BsplineOptimizer::FrontierIntersection()
{
    double tm, tmp;
    char p_state = 0, p_state_last = 0;
    for(int i = 0; i < bspline_pts_.size(); ++i)
    {
      char p_state = edt_environment_->sdf_map_->queryState2D(bspline_pts_[i]);
      if(p_state_last == 0 && p_state == -1)
      {
          pf_ = bspline_pts_[i];
          tf_ = i * t_sample_;
          idx_f_ = i;
          return true;
      }
      p_state_last = p_state;
    }
    return false;
}

// 找到要被调整的那个关键点pfov_
// 一个是通过通过fov，一个是通过RAPTOR里介绍的phi_min_
// 最后取tfov_较小的那个. perception调整的就是pfov_这一项
bool BsplineOptimizer::CriticalView()
{
    // 我觉得这个地方不应该是从pf开始往后遍历，而是应该依然从前往后遍历.
    // 还是从后向前呢。因为这里假设了，终点处的phi是比较大的？和地图的clearence差不多的。
    pfov_ = Eigen::Vector3d(0, 0, 10);
    tfov_ = 0.0;
    for(int i = idx_f_ - 1; i >= 0; --i)
    {
      Eigen::Vector3d path_dir = bspline_pts_[i + 1] - bspline_pts_[i];
      Eigen::Vector3d end_dir  = pf_ - bspline_pts_[i];
      if(end_dir.squaredNorm() > 1.5 * 1.5) break;
      if(path_dir.dot(end_dir) / (path_dir.norm() * end_dir.norm()) < thredHalfFOV)
      {
        pfov_ = bspline_pts_[i];
        tfov_ = i * t_sample_;
        v_fov_ = -end_dir.normalized();
        break;
      }
    }  
    
    for(int i = idx_f_ - 1; i >= 0; --i)
    // for(int i = 0; i < idx_f_; ++i)
    {
      double phi = edt_environment_->getMinDisViaLineSeg2D(pf_, bspline_pts_[i]);
      if(phi <= phi_min_)
      {
          pc_ = bspline_pts_[i];
          tc_ = i * t_sample_;
          idx_c_ = i;
          if(tc_ < tfov_)
          {
            tfov_ = tc_;
            pfov_ = pc_;
            v_fov_ = (bspline_pts_[i] - pc_).normalized();
            if(bspline_init_.getLength(0, tfov_, 0.05) < 1)
            {
              pfov_ = Eigen::Vector3d(0, 0, 10);
              return true;
            }
          }

          return false;
      }
    }
    return true;
}

//如果满足，那么就是safe的
bool BsplineOptimizer::checkSafty(const double & v, const double & df)
{
  return (0.5 * v * v <= (df - R_q_) * max_acc_);
}

Eigen::Vector3d BsplineOptimizer::calDv(const Eigen::Vector3d& ps)
{
    return ((ps - pf_) - (ps - pf_).dot(v_c_) * v_c_);
}

double BsplineOptimizer::findInitTs()
{
    double cost_min = 1000;
    double ts;
    Eigen::Vector3d ptmp, dv;
    double dv_norm;
    for(int i = 0; i < idx_c_; ++i)
    {
      ptmp = bspline_pts_[i];
      dv = calDv(ptmp);
      dv_norm = dv.norm();
      double cost1 = (dv_norm > dlmin_ ) ? ((dv_norm - dlmin_) * (dv_norm - dlmin_)) : 0.0;
      double dis = (ptmp - pf_).norm();
      double cost2 = (dis < ds_bar_) ? ((dis - ds_bar_) * (dis - ds_bar_)) : 0.0;
      if((cost1 + wnl_ * cost2) < cost_min)
      {
          cost_min = cost1 + wnl_ * cost2;
          ts = i * t_sample_;
      }
    }
    return ts;
}

vector<Eigen::Vector3d> BsplineOptimizer::getPerceptionInfo()
{
  vector<Eigen::Vector3d> perception_info(5, Eigen::Vector3d(0, 0, 10));
  if(unknown_)  
  {
    perception_info[0] = (bspline_init_.evaluateDeBoorT(tf_));
    std::cout << "pf_: " << pf_.transpose();
  }
  if(!visible_) 
  {
    perception_info[1] = (bspline_init_.evaluateDeBoorT(tc_));
    std::cout << ", pc_: " << pc_.transpose();
    std::cout << ", vc_: " << v_c_.transpose(); 
  }
  if(!safty_)    
  {
    perception_info[2] = (bspline_init_.evaluateDeBoorT(ts_));
    perception_info[3] = Eigen::Vector3d(ts_, 0, 0);
    std::cout << ", ps_: " << perception_info[2].transpose();
    std::cout << ", ts_: " << ts_;
  }
  perception_info[4] = pfov_;
  std::cout << std::endl;
  return perception_info;
}

void BsplineOptimizer::checkFixInterval(int& fix_num_start, int& fix_num_end)
{
  const vector<int> num_vec = this->x_var_vec_;
  int fix_start = 0;
  int fix_end   = 0;
  for(int i = 1; i < num_vec.size(); ++i)
  {
    if(num_vec[i - 1] != 0 && num_vec[i] == 0) 
    {
      fix_start = i;
      break;
    }
  }
  for(int i = num_vec.size() - 2; i >= 0; --i)
  {
    if(num_vec[i + 1] != 0 && num_vec[i] == 0)
    {
      fix_end = num_vec.size() - i;
      break;
    }
  }
  fix_num_start = fix_start + 1;
  fix_num_end   = fix_end;
}

}  // namespace fast_planner