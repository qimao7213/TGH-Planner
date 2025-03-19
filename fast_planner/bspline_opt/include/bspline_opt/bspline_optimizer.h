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



#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <plan_env/edt_environment.h>
#include <bspline/non_uniform_bspline.h>
#include <ros/ros.h>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace fast_planner {
class BsplineOptimizer {

public:
  static const int SMOOTHNESS;   //平滑
  static const int DISTANCE;     //障碍物
  static const int FEASIBILITY;  //动力学可行性
  static const int ENDPOINT;     //终点？
  static const int GUIDE;        //
  static const int WAYPOINTS;    //
  static const int NONE;
  static const int FEASIBILITYYaw; // 
  static const int PERCEPTION;   //
  static const int CTRLPTYAW;    // 
  static const int CTRLPTYAWS2;  //
  static const int CTRLPTYAWS3;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE; //包含上面的前三项，就是论文里面的

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /* main API */
  void            setEnvironment(const EDTEnvironment::Ptr& env);
  void            setParam(ros::NodeHandle& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id, 
                                      int fix_num_start = -1, int fix_num_end = -1, bool if_printf = 0);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setCostFunction(const int& cost_function);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most
  void setYawDotConstrains(const vector<double>& yaw_dot_constraints);
  void setInitBspline(const Eigen::MatrixXd& points, const double &ts);
  void optimize();

  Eigen::MatrixXd         getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);
  Eigen::MatrixXd vectorsToMatrix(const vector<Eigen::Vector3d>& ctrl_q);
private:
  EDTEnvironment::Ptr edt_environment_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  vector<double>          yaw_dot_constraints_;
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function
  bool   dynamic_;                     // moving obstacles ?
  double start_time_;                  // global time for moving obstacles
  bool only2D_;
  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // feasibility yaw weight
  double lambda9_;                // ctrl points yaw weights
  double lambda10_;               // jerk smoothness weight for ctrl points yaw S2
  double w_ctrl_in_perception_;

  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int    algorithm1_;             // optimization algorithms for quadratic cost
  int    algorithm2_;             // optimization algorithms for general cost
  int    max_iteration_num_[6];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;
  vector<Eigen::Vector3d> g_q_last_;
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_waypoints_;
  vector<Eigen::Vector3d> g_feasibility_yaw_;
  vector<Eigen::Vector3d> g_perception_;
  vector<Eigen::Vector3d> g_ctrl_pt_yaw_;

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  std::vector<double> best_variable_;  //
  double              min_cost_;       //

  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility
  vector<int> ctrl_point_idx_;
  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  static double costFunctionPerception(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data);
  static double costFunctionCtrlPtYaw (const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data);  
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);
  // 这个是单独规划yaw时设置的约束，后续可能不用这个
  void calcFeasibilityYawCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcVisibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                         vector<Eigen::Vector3d>& gradient);
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  // 这里把visibility项和safe项放到一起可以吧
  void calcPerceptionCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcCtrlPtYawCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  
  bool isQuadratic();

  // 考虑感知的
  NonUniformBspline bspline_init_;
  vector<Eigen::Vector3d> bspline_pts_; // 先按照0.05s的间隔把路径点采样出来，后续会反复使用
  double t_sample_ = 0.05;
  Eigen::Vector3d pf_, pc_, v_c_, dv_, pfov_, v_fov_;                  // 注意，pf_是已经再unknown了，这样合理吗？
  double tf_, tc_, ts_, dv_norm_, tfov_;
  int idx_f_, idx_c_;
  double ds_bar_, vel_s_bar_, R_q_ = 0.45, phi_min_ = 0.25; 

  bool unknown_ = false, visible_ = true, safty_ = true;                       
  bool FrontierIntersection();
  bool CriticalView();
  bool checkSafty(const double & v, const double & df);
  double findInitTs();
  Eigen::Vector3d calDv(const Eigen::Vector3d& ps);

  double min_r_;
  int fix_num_start_ = 0;
  int fix_num_end_ = 0;
  double cost_perception_ = 0.0, cost_perception_last_ = 0.0;
  bool stop_signal_ = false; // 这是一个全局信号，用于在PERCEPTION时判断是否已经达到停止的标准
  bool stop_signal_last_ = false;
  vector<int> x_var_vec_;
  double x_var_ = 100.0;
  bool if_printf_ = 0;

  Eigen::MatrixXd ctrl_pt_mid_;// 这个是用来储存暂时数据的，看看这个smooth项为什么会这么小呢？后面的smooth项为什么这么大？.
  Eigen::MatrixXd ctrl_pt_mid2_;

  int flag_out_;
  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time      time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }
  inline void setDisWeight(double times){  lambda2_ *= times;}
  inline void setOnly2D(){  only2D_ = true;}
  vector<Eigen::Vector3d> getPerceptionInfo();
  Eigen::MatrixXd getMidCtrlPt() {return this->ctrl_pt_mid_;}
  void checkFixInterval(int& fix_num_start, int& fix_num_end);
  int getFlag(){return this->flag_out_;}

  typedef unique_ptr<BsplineOptimizer> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif