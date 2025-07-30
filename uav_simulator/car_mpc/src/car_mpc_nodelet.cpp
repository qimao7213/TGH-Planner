#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <Eigen/Geometry>
#include "car_mpc.hpp"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}
//状态方程
Eigen::Vector4d diff(const Eigen::Vector4d& s,
                     const Eigen::Vector2d& input) {
    Eigen::Vector4d ds;
    double phi = s(2); // 航向角
    double v = input(0); // 输入的线速度
    double omega = input(1); // 输入的角速度
    // if(v < 0) omega = -omega;

    ds(0) = v * cos(phi);   // x方向的速度分量
    ds(1) = v * sin(phi);   // y方向的速度分量
    ds(2) = omega;          // 航向角的变化由角速度直接给出
    ds(3) = 0;              // 速度变化为0，因为线速度是直接输入的        
    return ds;
}

//用龙格库塔来算数值解
void step(Eigen::Vector4d& state, const Eigen::Vector2d& input, const double dt) {
  // Runge–Kutta
  Eigen::Vector4d k1 = diff(state, input);
  Eigen::Vector4d k2 = diff(state + k1 * dt / 2, input);
  Eigen::Vector4d k3 = diff(state + k2 * dt / 2, input);//////ERROR？？
  Eigen::Vector4d k4 = diff(state + k3 * dt, input);
  state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
}
using std::vector;
using fast_planner::NonUniformBspline;
namespace car_mpc {
class Nodelet : public nodelet::Nodelet {
 private:
  std::shared_ptr<CarMpc> mpcPtr_;
  ros::Timer mpcplan_timer_;
  double dt_sim_ = 0.01;
  ros::Subscriber bspline_sub_, replan_sub_, new_sub_, odom_sub_;
  ros::Subscriber mpc_pause_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  cmd_ros_pub_;
  ros::Publisher  get_new_traj_pub_, state_vel_pub_;

  nav_msgs::Odometry car_odom_;
  VectorX state_;  
  VectorU last_u_;
  car_msgs::CarCmd cmd_;


  bool receive_traj_ = false;      //是不是收到了新的轨迹，如果收到了，那么就要设置新轨迹
  bool init_odom_ = false;         //是不是收到了odom，如果收到了，则可以开始mpc plan
  bool is_backward_ = false;      //是不是在倒车
  bool init_path_set_ = false;     //是不是已经将一条轨迹设置给了mpc planner，如果设置了，又没有新的轨迹进来，则不用重复设置
  bool mpc_ready_ = false;
  int get_new_traj = 0;
  vector<NonUniformBspline> traj_; //一条轨迹，里面有位置、速度、加速度、yaw、dotyaw、ddotyaw，都是B样条
  double traj_duration_;
  ros::Time start_time_;
  int traj_id_;
  double delta_max_;               //最大转向角度
  double wheel_base_;
  double vel_max_ = 0.5;
  double omega_max_ = 1.57;
  double dt_;
  double vel_odom_;
  double k_vel_ = 10.0, k_omega_ = 1.0;
  double kp_ = 0.02, kd_ = 0.0;
  double yaw_error_last_ = 0.0;

  bool arrive_goal = false;
  bool mpc_pause = false;              //刚开始的时候，是不会pause的；后面如果重置了robot pose，那么就要让mpc停止，直到新的轨迹进来后再继续
  bool use_teb = false;                

  vector<Eigen::Vector3d> traj_cmd_, traj_real_;

  void mpcplan_timer_callback(const ros::TimerEvent& event) 
  {
    if(mpc_pause) return;    
    if(!init_odom_) return;
    if(!init_path_set_ && receive_traj_)//如果还没有初始化路径，就初始化
    {
      if(traj_.empty()) return;
      mpcPtr_->setBsplineTraj(traj_);
      init_path_set_ = true;
      receive_traj_ = false;
      mpc_ready_ = true;
    }
    if(! mpc_ready_) return;
    // 开启mpc跟踪部分
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec() + dt_;
    // // 这里不能直接用t_cur，而是需要每一次都校正。
    // double delta_t = 0.02;
    // int check_num = 10;
    // double min_dis = 100; double min_t = t_cur;
    // for(int i = -check_num; i <= check_num; ++i)
    // {
    //   double dis = (state_.head(2) - traj_[0].evaluateDeBoorT(t_cur + i * delta_t).head(2)).squaredNorm();
    //   if(dis < min_dis)
    //   {
    //     min_dis = dis;
    //     min_t = t_cur + i * delta_t;
    //   }
    // }
    // t_cur = min_t;

    // t_cur = min(t_cur, traj_duration_);
    // t_cur = max(t_cur, 0.00);
    
    // if (get_new_traj)
    // {
    //   get_new_traj = 0;
    //   std_msgs::Float64 msg;
    //   msg.data = 1.0;
    //   get_new_traj_pub_.publish(msg);

    //   std_msgs::Float64MultiArray init_vel_msg;
    //   init_vel_msg.data.clear();
      
    //   init_vel_msg.data.push_back(traj_[1].evaluateDeBoorT(t_cur).norm());
    //   init_vel_msg.data.push_back(traj_[1].evaluateDeBoorT(t_cur+0.4).norm());
    //   init_vel_msg.data.push_back(traj_[1].evaluateDeBoorT(t_cur+0.8).norm());
    //   init_vel_msg.data.push_back(traj_[1].evaluateDeBoorT(t_cur+1.0).norm());
    //   replan_init_vel_pub_.publish(init_vel_msg);
    //   ROS_WARN_STREAM_THROTTLE(0.1, "t_cur: " << t_cur << ", traj_duration_: " << traj_duration_);
    // }
    // else
    // {
    //   std_msgs::Float64 msg;
    //   msg.data = 0.0;
    //   get_new_traj_pub_.publish(msg);
    // }

    // 如果是直接从bsplie里面读取速度和角速度的话，就打开下面这一段。否则就是通过mpc算的。
    if(0){
      Eigen::Vector3d pos, vel, acc, pos_f;
      double yaw, yawdot;
    
      if (t_cur < traj_duration_ && t_cur >= 0.0) {
        pos = traj_[0].evaluateDeBoorT(t_cur);
        vel = traj_[1].evaluateDeBoorT(t_cur);
        acc = traj_[2].evaluateDeBoorT(t_cur);
        yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
        yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    
        double tf = min(traj_duration_, t_cur + 2.0);
        pos_f = traj_[0].evaluateDeBoorT(tf);
    
      } else if (t_cur >= traj_duration_) {
        /* hover when finish traj_ */
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();
        yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
        yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];
    
        pos_f = pos;
    
      } else {
        cout << "[Traj server]: invalid time." << endl;
      }

      double yaw_error = yaw - state_(2);
      
      yaw_error = Mod2Pi(yaw_error);
      if(yaw_error > M_PI) yaw_error -= 2*M_PI;
      else if (yaw_error < -M_PI) yaw_error += 2-M_PI;      
      // ROS_INFO_STREAM("desired and current: " << yaw << ", " << state_(2) << ", error: " << yaw_error);

      // const double kp = 0.02;
      // const double kd = 0.00000;
      double yaw_vel = kp_ * yaw_error + kd_ * (yaw_error - yaw_error_last_) / dt_sim_;

      double vel_val = vel.norm();
      double omega_limit = 1.0 * std::fabs(vel_val) / wheel_base_;
      // 限幅处理
      vel_val = std::clamp(vel_val, -vel_max_, vel_max_);
      yawdot = std::clamp(yawdot, -omega_limit, omega_limit);
      yaw_vel = std::clamp(yaw_vel, -omega_limit, omega_limit);

      // vel *= k_vel_ ; 
      // omega *= k_omega_;
      // 创建 cmd_vel 消息
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = vel_val;          // 线速度
      cmd_vel.angular.z = yawdot * 0.75;     // 角速度
      // 发布 cmd_vel 消息
      cmd_ros_pub_.publish(cmd_vel);
      return;
    }

    int ret = mpcPtr_->solveQP(state_, t_cur);

    ros::Time t2 = ros::Time::now();
    double solve_time = (t2 - time_now).toSec();
    // std::cout << "solve qp costs: " << 1e3 * solve_time << "ms" << std::endl;
    // TODO
    car_msgs::CarCmd msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    VectorX x;
    VectorU u;

    if (ret != 1 && ret != 2) {
      ROS_ERROR_STREAM("MPC solve failed, ret: " << ret);
      mpcPtr_->visualization(t_cur);
      msg.a = -last_u_(0);
      msg.delta = -last_u_(1);
    } else {
      mpcPtr_->getPredictXU(0, x, u);
      msg.a = u(0);
      msg.delta = std::min(u(1), delta_max_ / 57.3);
      last_u_ = u;
    }
    // if(!use_teb) msg.delta *= 0.95;
    cmd_pub_.publish(msg);
    mpcPtr_->visualization(t_cur);
    double scale_factor = ((msg.a) > -1e-3) ? 1.2 : 1.7;
    if(state_(3) < 0.0 && fabs(state_(3)) < 0.4 * vel_max_) msg.a = fabs(msg.a);

    // ------- cmd_vel 统一发布部分 --------
    double vel = state_(3) + scale_factor * dt_sim_ * msg.a;
    double omega = 1.0 * vel * std::tan(msg.delta) / wheel_base_;
    double omega_limit = 1.0 * std::fabs(vel) / wheel_base_;

    // 限幅处理
    vel = std::clamp(vel, -vel_max_, vel_max_);
    omega = std::clamp(omega, -omega_limit, omega_limit);

    // 生成并发布 cmd_vel
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = omega;
    cmd_ros_pub_.publish(cmd_vel);

    if(use_teb)
    {
      // 如果不是use_teb，那么就需要通过龙格库塔来更新（暂时的假）状态
      state_(3) = vel;
      step(state_, Eigen::Vector2d(vel, omega), dt_sim_);
    }
    // {
    //   std_msgs::Float64MultiArray state_vel_msg;
    //   state_vel_msg.data.push_back(state_(3));
    //   state_vel_pub_.publish(state_vel_msg);
    // }
  }

  // 订阅odom，更新当前状态，并记录历史轨迹
  void odomCallbck(const nav_msgs::Odometry::ConstPtr& msg) {
    // std::cout << "odomCallback start error?" << std::endl;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Vector3d rot_x = q.toRotationMatrix().block(0, 0, 3, 1);
    double yaw_angle         = atan2(rot_x(1), rot_x(0));
    Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    double vel_angle = atan2(msg->twist.twist.linear.y, msg->twist.twist.linear.x);
    double angle_diff = vel_angle - yaw_angle;
    angle_diff = Mod2Pi(angle_diff);
    if(abs(angle_diff) > 0.75 * M_PI)
    {
      // ROS_WARN("The car is running backward!");
      state_ << x, y, yaw_angle, -v.norm();
    }
    else
      state_ << x, y, yaw_angle, v.norm();
    init_odom_ = true;
    // std::cout << "odomCallbck :" << state_.transpose() << std::endl;
    // traj_real_.push_back(
    //     Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

    // if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
    // std::cout << "odomCallback end error?" << std::endl;
  }


  void bsplineCallback(const plan_manage::BsplineConstPtr &msg)
  {
    // parse pos traj
    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
    Eigen::VectorXd knots(msg->knots.size());
    for (int i = 0; i < msg->knots.size(); ++i) {
      knots(i) = msg->knots[i];
    }
    for (int i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(i, 0) = msg->pos_pts[i].x;
      pos_pts(i, 1) = msg->pos_pts[i].y;
      pos_pts(i, 2) = msg->pos_pts[i].z;
    }

    //这里的0.1是节点的internal，是随便给的，setKnot会将这里设置的0.1替换
    NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj
    Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    for (int i = 0; i < msg->yaw_pts.size(); ++i) {
      yaw_pts(i, 0) = msg->yaw_pts[i];
    }
    NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_.push_back(yaw_traj);
    traj_.push_back(yaw_traj.getDerivative());

    traj_duration_ = traj_[0].getTimeSum();
    // mpcPtr_->setBsplineTraj(traj_);   //在mpc的循环里面才设置轨迹
    // double traj_duration_yaw = yaw_traj.getTimeSum();
    //虽然控制点的节点向量不同，但是duration是相同的
    // std::cout << "position duration: " << traj_duration_ << ", yaw duration: " << traj_duration_yaw << std::endl;
    receive_traj_ = true;
    init_path_set_ = false;

    arrive_goal = false;//每次有新的路径进来，说明就没有到达终点
    mpc_pause = false;
    get_new_traj = 1;
    // ROS_WARN("Receive New Traj!!");
    return;
  }

  //收到replan信号之后，通过更改traj_duration_结束对当前轨迹的跟踪
  void replanCallback(std_msgs::Empty msg) 
  {
    /* reset duration */
    const double time_out = 0.01;
    ros::Time time_now = ros::Time::now();
    double t_stop = (time_now - start_time_).toSec() + time_out;
    traj_duration_ = min(t_stop, traj_duration_);
  }
  //清空内存
  void newCallback(std_msgs::Empty msg) 
  {
    traj_cmd_.clear();
    traj_real_.clear();
  }

  void pauseCallback(std_msgs::Empty msg) 
  {
    mpc_pause = true;
    ROS_WARN("Now mpc pause, waiting ...");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getPrivateNodeHandle());
    std::cout << "now init Car Mpc" << std::endl;
    mpcPtr_ = std::make_shared<CarMpc>(nh);
    nh.getParam("car_mpc/dt", dt_);
    nh.getParam("car_mpc/delta_max", delta_max_);
    nh.getParam("car_mpc/wheel_base", wheel_base_);
    nh.getParam("car_mpc/v_max", vel_max_);
    nh.getParam("car_mpc/k_vel", k_vel_);
    nh.getParam("car_mpc/k_omega", k_omega_);
    nh.getParam("car_mpc/k_p", kp_);
    nh.getParam("car_mpc/k_d", kd_);
    nh.getParam("car_mpc/use_teb", use_teb);
    std::cout << "vel_max_: " << vel_max_ << ", kp_: " << kp_ << ", kd_: " << kd_ <<  std::endl;
    vel_odom_ = 0.0;
    // Debug    
#ifdef DEBUG_MODE
    {
      dt = 0.04;
    }
#endif

    mpcplan_timer_ = nh.createTimer(ros::Duration(dt_sim_), &Nodelet::mpcplan_timer_callback, this);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odometry", 10, &Nodelet::odomCallbck, this);
    cmd_pub_ = nh.advertise<car_msgs::CarCmd>("cmd", 100);
    cmd_ros_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_ros", 100);
    get_new_traj_pub_ = nh.advertise<std_msgs::Float64>("/car_simulator/get_new_traj", 10);
    state_vel_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/car_simulator/state_vel_mpc", 10);
    // replan_sub_的信号会先发布，收到之后就把traj_duration_设当前时间+0.01s，让对当前轨迹的跟踪停下来
    // 然后才会重新收到新的bspline，用重置一遍开始跟踪新的traj
    bspline_sub_ = nh.subscribe<plan_manage::Bspline>("/planning/bspline",10, 
                   &Nodelet::bsplineCallback, this, ros::TransportHints().tcpNoDelay());
    replan_sub_  = nh.subscribe<std_msgs::Empty>("/planning/replan",10, 
                   &Nodelet::replanCallback, this, ros::TransportHints().tcpNoDelay());
    // new_sub_     = nh.subscribe<std_msgs::Empty>("/planning/new",10, 
    //                &Nodelet::newCallback, this, ros::TransportHints().tcpNoDelay());   
    mpc_pause_sub_ = nh.subscribe<std_msgs::Empty>("/car_simulator/mpc_pause",10, 
                   &Nodelet::pauseCallback, this, ros::TransportHints().tcpNoDelay()); 
    std::cout << "car mpc init success!-----" << std::endl;        
  }
};
}  // namespace car_mpc

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(car_mpc::Nodelet, nodelet::Nodelet);