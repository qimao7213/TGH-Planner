#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <deque>
#include "iosqp.hpp"

#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "std_msgs/Empty.h"
#include <std_msgs/Float64MultiArray.h>
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <glog/logging.h>
namespace car_mpc {

static constexpr int n = 4;  // state [x y phi v]，状态的维度
static constexpr int m = 2;  // input [a delta]，输入的维度
typedef Eigen::Matrix<double, n, n> MatrixA;//线性化时的A
typedef Eigen::Matrix<double, n, m> MatrixB;//线性化时的B
typedef Eigen::Vector4d VectorG;//线性化时的G
typedef Eigen::Vector4d VectorX;//状态向量 [x y phi v]
typedef Eigen::Vector2d VectorU;//输入向量 [a delta]
using fast_planner::NonUniformBspline;

void printf_break(int id)
{
  std::cout << "------------" << id << "-------------" << std::endl;
}

class CarMpc {
 private:
  ros::NodeHandle nh_;
  ros::Publisher ref_pub_, traj_pub_, traj_delay_pub_, ref_pub2_;

  double ll_;//车长
  double dt_;//预测周期的时长
  double rho_;//过程的权重？
  int N_;//预测的horizon
  double rhoN_;//终端状态的权重？

  double v_max_, a_max_, delta_max_, ddelta_max_;//bound约束
  double delay_;//延迟

  bool path_direction_ = 1;//1表示前进
  bool if_printf_ = false;

  vector<fast_planner::NonUniformBspline> traj_;//参考轨迹
  double traj_duration_;
  double desired_v_;//期待的速度？预估的一个速度？？

  osqp::IOSQP qpSolver_;

  std::vector<VectorX> refState_;
  std::vector<VectorX> predictState_;//预测出来的state
  std::vector<VectorU> predictInput_;//预测出来的input
  std::deque<VectorU> historyInput_;//先前的input，用于有delay的场景
  int history_length_;//由于delay造成的延迟的预测周期数
  VectorX x0_observe_;//预测时的状态，由car_simulator直接龙格库塔积分得到
  std::vector<VectorX> compensateDelayX0_;
  bool useCompensate1 = 0, useCompensate2 = 0;
  // 线性化时用到的几个矩阵
  // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
  MatrixA Ad_;//4*4
  MatrixB Bd_;//4*2
  VectorG gd_;//4*1

  /**整合为这样的形式，就可以直接放到求解器里面计算了？
   * osqp interface:
   * minimize     0.5 x^T P_ x + q_^T x
   * subject to   l_ <= A_ x <= u_
   **/
  // 上面公式里面的x是指要优化的量，是输入u吧？？
  // P_和q_是cost的权重矩阵
  // A_，l_，u_是整合之后的约束矩阵
  Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;

  /* *
   *               /  x1  \
   *               |  x2  |
   *  lx_ <=  Cx_  |  x3  |  <= ux_
   *               | ...  |
   *               \  xN  /
   * */
  //和state相关的约束
  Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // p, v constrains
  /* *
   *               /  u0  \
   *               |  u1  |
   *  lu_ <=  Cu_  |  u2  |  <= uu_
   *               | ...  |
   *               \ uN-1 /
   * */
  //和input相关的约束
  Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // [a delta ] constrains
  Eigen::SparseMatrix<double> Qx_;//这个是Q_bar

  //在某点处线性化
  void linearization(const double& phi,
                     const double& v,
                     const double& delta) {
    // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
    // TODO: set values to Ad_, Bd_, gd_
    // ...
    Ad_.setIdentity();  // Ad for instance
    Bd_.setZero();
    gd_.setZero();

    MatrixA Ac_;
    Ac_.setZero();
    Ac_.coeffRef(0, 2) = -1 * v * sin(phi);
    Ac_.coeffRef(0, 3) = cos(phi);
    Ac_.coeffRef(1, 2) = v * cos(phi);
    Ac_.coeffRef(1, 3) = sin(phi);
    Ac_.coeffRef(2, 3) = tan(delta)/ll_;
    Ad_ += Ac_ * dt_;

    Bd_.coeffRef(3, 0) = 1;
    Bd_.coeffRef(2, 1) = v/(ll_ * cos(delta) * cos(delta));
    Bd_ *= dt_;

    gd_(0) = v*phi*sin(phi);
    gd_(1) = -1*v*phi*cos(phi);
    gd_(2) = -1*v*delta/(ll_ * cos(delta) * cos(delta));
    gd_(3) = 0;
    gd_ *= dt_;

    return;
  }


  void calLinPoint(const double& t0, double &x, double &y, double& phi, double& v, double& delta) 
  {
    Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t0);
    Eigen::Vector3d vel = traj_[1].evaluateDeBoorT(t0);
    x = pos(0);
    y = pos(1);
    v = vel.head(2).norm();

    double yaw = traj_[3].evaluateDeBoorT(t0)[0];
    phi = yaw;
    double yawdot = traj_[4].evaluateDeBoorT(t0)[0];
    delta = atan2(ll_ * yawdot / (v) , 1.0);
    if(t0 >= traj_duration_)
    {
      v = 0;
      delta = 0;
    }
  }

  inline VectorX diff(const VectorX& state,
                      const VectorU& input) const {
    VectorX ds;
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    ds(0) = v * cos(phi);
    ds(1) = v * sin(phi);
    ds(2) = v / ll_ * tan(delta);
    ds(3) = a;
    return ds;
  }

  inline void step(VectorX& state, const VectorU& input, const double dt) const {
    // Runge–Kutta
    VectorX k1 = diff(state, input);
    VectorX k2 = diff(state + k1 * dt / 2, input);
    VectorX k3 = diff(state + k2 * dt / 2, input);
    VectorX k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }


 public:
  CarMpc(ros::NodeHandle& nh) : nh_(nh) {
    // load map
    // nh.getParam("car_mpc/desired_v", desired_v_);
    // load parameters
    nh_.getParam("car_mpc/wheel_base", ll_);
    nh_.getParam("car_mpc/dt", dt_);
    nh_.getParam("car_mpc/rho", rho_);
    nh_.getParam("car_mpc/N", N_);
    nh_.getParam("car_mpc/rhoN", rhoN_);
    nh_.getParam("car_mpc/v_max", v_max_);
    nh_.getParam("car_mpc/a_max", a_max_);
    // v_max_ *= 1.7; a_max_ *= 1.7;
    // 这里获取的delta_max_是最大转向角
    // w_max = v_max/R_min = v_max * tan(steering_angle)/ll
    // ERROR。这里的最大角速度设置是有问题的
    nh_.getParam("car_mpc/delta_max", delta_max_);
    nh_.getParam("car_mpc/ddelta_max", ddelta_max_);
    nh_.getParam("car_mpc/verbose", if_printf_);
    


    // Debug
#ifdef DEBUG_MODE
    {
      ll_ = 0.4;
      dt_ = 0.04;
      rho_ = 1.0;
      N_ = 50;
      rhoN_ = 2.0;
      v_max_ = 1.5;
      a_max_ = 1.0;
      delta_max_ = 30.0;
      ddelta_max_ = 2.0;
    }
#endif

    delta_max_ /= (57.3); //ERROR,这里的设置是有问题的。设置为1附近就差不多
    // delta_max_ *= 1.2;
    desired_v_ = v_max_;


    ref_pub_  = nh_.advertise<nav_msgs::Path>("mpc_path_ref", 10);
    ref_pub2_ = nh_.advertise<nav_msgs::Path>("mpc_path_ref_new", 10); // 两个一样的
    traj_pub_ = nh_.advertise<nav_msgs::Path>("mpc_path_pred", 10);

    // TODO: set initial value of Ad, Bd, gd
    Ad_.setIdentity();  // Ad for instance
    Bd_.setZero();
    gd_.setZero();
    // ...
    // set size of sparse matrices
    P_.resize(m * N_, m * N_);
    q_.resize(m * N_, 1);
    Qx_.resize(n * N_, n * N_);
    // stage cost
    // every        1 0 0   0  
    //          Q = 0 1 0   0
    //              0 0 rho 0
    //              0 0 0   0
    //那就是看你cost function怎么设计了，前面的两个1是x和y的位置误差，rho是对phi，对方向的跟踪？对v没有跟踪
    Qx_.setIdentity();
    for (int i = 1; i < N_; ++i) {
      Qx_.coeffRef(i * n - 2, i * n - 2) = rho_; // 这个rho_是phi的权重
      Qx_.coeffRef(i * n - 1, i * n - 1) = 0;
      // if(i <= 10)
      // {
      //   Qx_.coeffRef(i * n - 3, i * n - 3) *= 1.2; 
      //   Qx_.coeffRef(i * n - 4, i * n - 4) *= 1.2;
      // }
    }
    //这下面是设置Q_bar里面的F矩阵
    Qx_.coeffRef(N_ * n - 4, N_ * n - 4) = rhoN_;
    Qx_.coeffRef(N_ * n - 3, N_ * n - 3) = rhoN_;
    Qx_.coeffRef(N_ * n - 2, N_ * n - 2) = rhoN_ * rho_;//
    int n_cons = 4;  // [v a delta ddelta]，约束的维度。对每一个预测周期都需要约束
    A_.resize(n_cons * N_, m * N_);
    l_.resize(n_cons * N_, 1);
    u_.resize(n_cons * N_, 1);
    // [v] constrains
    Cx_.resize(1 * N_, n * N_);
    lx_.resize(1 * N_, 1);
    ux_.resize(1 * N_, 1);
    // [a delta ddelta] constrains
    Cu_.resize(3 * N_, m * N_);
    lu_.resize(3 * N_, 1);
    uu_.resize(3 * N_, 1);
    // set lower and upper boundaries
    for (int i = 0; i < N_; ++i) {
      // TODO: set stage constraints of inputs (a, delta, ddelta)
      // -a_max <= a <= a_max for instance:
      Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
      lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
      uu_.coeffRef(i * 3 + 0, 0) = a_max_;
      // ...
      Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
      lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
      uu_.coeffRef(i * 3 + 1, 0) = delta_max_;

      // TODO: 这里可能还有问题
      if(i > 0)
      {
        Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1/dt_;
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1/dt_;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_;        
      }
      else
      {
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      }
      // if(i > 0) Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1;
      // Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      // lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_ * dt_;
      // uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_ * dt_;  


      // TODO: set stage constraints of states (v)
      // -v_max <= v <= v_max
      Cx_.coeffRef(i, i * n + 3) = 1;
      lx_.coeffRef(i, 0) = -v_max_; 
      ux_.coeffRef(i, 0) = v_max_;
    }
    // set predict mats size
    refState_    .resize(N_);    
    predictState_.resize(N_);
    predictInput_.resize(N_);
    for (int i = 0; i < N_; ++i) {
      predictInput_[i].setZero();
    }
  }

  // bool check_goal(const VectorX& x0)
  // {
  //   Eigen::Vector2d dxy = s_(s_.arcL(), 1);
  //   double phi = atan2(dxy.y(), dxy.x());
  //   if(path_direction_ == 0) phi -= M_PI;
  //   if (phi - x0(2) > M_PI) {
  //     phi -= 2 * M_PI;
  //   } else if (phi - x0(2) < -M_PI) {
  //     phi += 2 * M_PI;
  //   } 
    
  //   Eigen::Vector2d gxy = s_(s_.arcL(), 0);
  //   double dx = gxy.x() - x0.x();
  //   double dy = gxy.y() - x0.y();
  //   std::cout << "dxy: " <<  (dx * dx + dy * dy) << ", v: " << std::abs(x0(3));
  //   std::cout << ", std::abs(phi - x0(2)): " << std::abs(phi - x0(2)) << std::endl;
  //   if((dx * dx + dy * dy) < 0.1 * 0.1 && std::abs(x0(3)) < 0.03 && std::abs(phi - x0(2)) < 0.05) return true;

  //   return false;
  // }
  //TODO: 
  bool check_goal(const double &tc, const VectorX &x0)
  {
    //需要检查一下速度是不是已经降下来了
    // if(tc >)
    return false;

  }


  int solveQP(const VectorX& x0_observe, const double & tc) {
    auto x0 = x0_observe;
    x0_observe_ = x0_observe;
    //这是XX的约束，由预测出来的delta，结合ddelta_max来确定delta的上下界？
    lu_.coeffRef(2, 0) = predictInput_.front()(1) - ddelta_max_ * dt_;
    uu_.coeffRef(2, 0) = predictInput_.front()(1) + ddelta_max_ * dt_;
    bool arrive_goal = check_goal(tc, x0);
    // std::cout << "arrive the goal ?? : " << arrive_goal << std::endl;
    if(arrive_goal)
    {
      // for (int i = 0; i < N_; ++i) 
      // {
      //   predictInput_[i] = VectorU::Zero();
      //   predictState_[i] = predictMat.col(i);
      // }
      std::cout << "到达终点时的状态为：" << x0.transpose() << std::endl;
      return 2;//表示已经到达终点了
    }
    // set BB, AA, gg
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n * N_, m * N_);
    AA.setZero(n * N_, n);
    gg.setZero(n * N_, 1);
    double t0 = tc;
    double x, y;
    double phi, v, delta;  
    double last_phi = x0(2);
    Eigen::SparseMatrix<double> qx;
    qx.resize(n * N_, 1);
    for (int i = 0; i < N_; ++i) {
      calLinPoint(t0, x, y, phi, v, delta);
      // check 当前状态和traj t0时的状态
      if(i == 0)
      {
        // std::cout << "Current state: " << x0.transpose() << ".\nTraj t0 state: " << VectorX(x, y, phi, v).transpose() << std::endl;
      }
      if (phi - last_phi > M_PI) {
        phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        phi += 2 * M_PI;
      }
      last_phi = phi;
      linearization(phi, v, delta);//这些量都是当前状态所处轨迹上的点
      // calculate big state-space matrices
      /* *                BB                AA
       * x1    /       B    0  ... 0 \    /   A \
       * x2    |      AB    B  ... 0 |    |  A2 |
       * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
       * ...   |     ...  ...  ... 0 |    | ... |
       * xN    \A^(n-1)B  ...  ... B /    \ A^N /
       *
       *     X = BB * U + AA * x0 + gg
       * */
      if (i == 0) {
        BB.block(0, 0, n, m) = Bd_;
        AA.block(0, 0, n, n) = Ad_;
        gg.block(0, 0, n, 1) = gd_;
      } else {
        // TODO: set BB AA gg
        // ...
        BB.block(n*i, m*i, n, m) = Bd_;
        for(int j = i - 1; j >= 0; --j)
        {
          BB.block(n * i, m * j, n, m) = Ad_ * BB.block(n*(i-1), m*j, n, m);
        }
        AA.block(n * i, 0, n, n) = Ad_ * AA.block(n*(i-1), 0, n, n);
        gg.block(n * i, 0, n, 1) = Ad_ * gg.block(n*(i-1), 0, n, 1) + gd_;
      }

      //这个地方不能用矩阵幂的方式来求，因为Ad_之类的在每一次迭代的时候都是变化的
      // for(int j = 0; j <= i; ++j)
      // {
      //   BB.block(n * i, m * j, n, m ) = matrixAPower(Ad_, i-j) * Bd_;
      // }
      // AA.block(n * i, 0, n, n) = matrixAPower(Ad_, i+1);
      // for(int j = 0; j <= i; ++j)
      // {
      //   gg.block(n * i, 0, n, 1) += matrixAPower(Ad_, j) * gd_;
      // }

      // TODO: set qx
      Eigen::Vector2d xy(x, y);  // reference (x_r, y_r)。这里不应该是下一个期待到达的点吗？为什么是当前的点呢？

      // cost function should be represented as follows:
      /* *
       *           /  x1  \T       /  x1  \         /  x1  \
       *           |  x2  |        |  x2  |         |  x2  |
       *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
       *           | ...  |        | ...  |         | ...  |
       *           \  xN  /        \  xN  /         \  xN  /
       * */

      // qx.coeffRef(...
      // ...
      // qx = -Qx_.toDense().block(n * i, n * i, 4, 4) transpose() * VectorX(xy(0), xy(1), phi, 0);
      qx.coeffRef(n*i + 0, 0) = -Qx_.coeffRef(n * i + 0, n * i + 0) * xy(0);
      qx.coeffRef(n*i + 1, 0) = -Qx_.coeffRef(n * i + 1, n * i + 1) * xy(1);
      qx.coeffRef(n*i + 2, 0) = -Qx_.coeffRef(n * i + 2, n * i + 2) * phi;
      qx.coeffRef(n*i + 3, 0) = -Qx_.coeffRef(n * i + 3, n * i + 3) * v;
      // qx.coeffRef(n*i + 3, 0) = -0;//本来就是让期望的末速度为0了？？
      t0 += dt_;
      t0 = t0 < traj_duration_ ? t0 : traj_duration_;
      // if(i == 0)
      //   std::cout << "desired: " << xy.transpose() << ", " << phi << ", " << v << std::endl;
      refState_[i] = VectorX(x, y, phi, v);
    }
    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

    // state constrants propogate to input constraints using "X = BB * U + AA * x0 + gg"
    /* *
     *               /  x1  \                              /  u0  \
     *               |  x2  |                              |  u1  |
     *  lx_ <=  Cx_  |  x3  |  <= ux_    ==>    lx <=  Cx  |  u2  |  <= ux
     *               | ...  |                              | ...  |
     *               \  xN  /                              \ uN-1 /
     * */
    Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse; // N_*mN_
    Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;//N_*1
    Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;//N_*1

    /* *      / Cx  \       / lx  \       / ux  \
     *   A_ = \ Cu_ /, l_ = \ lu_ /, u_ = \ uu_ /
     * */
    
    //这里为什么要先转置一遍呀
    //A_的维度为4N_*mN_，所以优化变量的维度是mN_*1，还是控制量，
    //不是PPT24页里面把状态量和控制量放在一起优化，而是把对状态量的约束转换为了对控制量的约束

    Eigen::SparseMatrix<double> A_T = A_.transpose();
    A_T.middleCols(0, Cx.rows()) = Cx.transpose();
    A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
    A_ = A_T.transpose();
    //这里就是直接拼接起来，那么优化变量就变成了？？
    for (int i = 0; i < lx.rows(); ++i) {
      l_.coeffRef(i, 0) = lx.coeff(i, 0);
      u_.coeffRef(i, 0) = ux.coeff(i, 0);
    }
    for (int i = 0; i < lu_.rows(); ++i) {
      l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
      u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
    }
    Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
    P_ = BBT_sparse * Qx_ * BB_sparse;
    q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;
    // osqp
    Eigen::VectorXd q_d = q_.toDense();
    Eigen::VectorXd l_d = l_.toDense();
    Eigen::VectorXd u_d = u_.toDense();
    qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
    qpSolver_.solve();
    int ret = qpSolver_.getStatus();
    if (ret != 1) {
      ROS_ERROR_STREAM("fail to solve QP! ret: " << ret);
      return ret;
    }
    //将很长的向量映射为矩阵
    //qpSolver_求解的只是input u，而状态x是计算出来的
    Eigen::VectorXd sol = qpSolver_.getPrimalSol();
    Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_);
    Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
    Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);
    //然后将矩阵放到vector中去
    for (int i = 0; i < N_; ++i) {
      predictInput_[i] = solMat.col(i);
      predictState_[i] = predictMat.col(i);
    }
    return ret;
  }

  void getPredictXU(double t, VectorX& state, VectorU& input) {
    if (t <= dt_) {
      state = predictState_.front();
      input = predictInput_.front();
      return;
    }

    //输入的t一直都是0，那么下面这一段是用来干什么的呢?
    int horizon = std::floor(t / dt_);
    double dt = t - horizon * dt_;
    state = predictState_[horizon - 1];
    input = predictInput_[horizon - 1];
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    state(0) += dt * v * cos(phi);
    state(1) += dt * v * sin(phi);
    state(2) += dt * v / ll_ * tan(delta);
    state(3) += dt * a;
  }

  // visualization
  // 在可视化参考轨迹，预测的轨迹，带延迟的预测轨迹。第一个是不变的，后面两个和运行的时刻有关
  void visualization(const double & t_start) {
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p;
    double t_pred = 0.0;
    for (int i = 0; i < N_; ++i) {
      t_pred = min(i * dt_ + t_start, traj_duration_);
      p.pose.position.x = traj_[0].evaluateDeBoorT(t_pred)(0);
      p.pose.position.y = traj_[0].evaluateDeBoorT(t_pred)(1);
      p.pose.position.z = -0.5;
      // std::cout << "Error: " << p.pose.position.y - refState_[i](1) << std::endl;
      msg.poses.push_back(p);
    }
    ref_pub_.publish(msg);

    msg.poses.clear();
    for (int i = 0; i < N_; ++i) {
      p.pose.position.x = refState_[i](0);
      p.pose.position.y = refState_[i](1);
      p.pose.position.z = -0.5;
      // std::cout << "pred state: " << refState_[i].transpose() << std::endl; 
      msg.poses.push_back(p);
    }
    ref_pub2_.publish(msg);

    msg.poses.clear();
    for (int i = 0; i < N_; ++i) {
      p.pose.position.x = predictState_[i](0);
      p.pose.position.y = predictState_[i](1);
      p.pose.position.z = -0.5;
      msg.poses.push_back(p);
    }
    traj_pub_.publish(msg);

    if(!if_printf_) return;
    // LOG(INFO) << "---------------" << std::endl;
    for (int i = 0; i < N_; ++i)
    {
      std::ostringstream oss;
      oss << "idx: " << i << ", (" << refState_[i].transpose() << "), "
          << ", (" << predictState_[i].transpose() << "). ";
      // LOG(INFO) << oss.str() << std::endl;
    }
    // std_msgs::Float64MultiArray msg;
    // msg.data.push_back();
    // msg.data.push_back();
    // msg.data.push_back();
    // msg.data.push_back();

    return;
  }

  void setBsplineTraj(const vector<NonUniformBspline> &traj)
  {
    traj_ = traj;
    traj_duration_ = traj_[0].getTimeSum();
    ROS_WARN("Set New Traj!!");
  }
};

}  // namespace car_mpc