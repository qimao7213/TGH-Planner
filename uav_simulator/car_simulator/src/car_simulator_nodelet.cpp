#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <deque>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "common_srvs/set_init_pose.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>


bool use_teb_ = false;
double car_height_;
struct Car {
  double l;
  // state: x, y, phi, v
  // input: a, delta  
  Eigen::Vector4d state;

  inline void setInitialState(const Eigen::Vector4d& s) {
    state = s;
  }
  //状态方程
  inline Eigen::Vector4d diff(const Eigen::Vector4d& s,
                              const Eigen::Vector2d& input) const {
      Eigen::Vector4d ds;
      if(use_teb_) // 使用teb用这个
      {
        double phi = s(2); // 航向角
        double v = input(0); // 输入的线速度
        double omega = input(1); // 输入的角速度
        // if(v < 0) omega = -omega;

        ds(0) = v * cos(phi);   // x方向的速度分量
        ds(1) = v * sin(phi);   // y方向的速度分量
        ds(2) = omega;          // 航向角的变化由角速度直接给出
        ds(3) = 0;              // 速度变化为0，因为线速度是直接输入的        
      }
      else // 自己写的mpc用这个
      {
        double phi = s(2);
        double v = s(3);
        double a = input(0);
        double delta = input(1);
        ds(0) = v * cos(phi);
        ds(1) = v * sin(phi);
        ds(2) = v / l * tan(delta);
        ds(3) = a;        
      }
      return ds;
  }

  //用龙格库塔来算数值解
  void step(const Eigen::Vector2d& input, const double dt) {
    // Runge–Kutta
    Eigen::Vector4d k1 = diff(state, input);
    Eigen::Vector4d k2 = diff(state + k1 * dt / 2, input);
    Eigen::Vector4d k3 = diff(state + k2 * dt / 2, input);//////ERROR？？
    Eigen::Vector4d k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }
};

namespace car_simulator {
class Nodelet : public nodelet::Nodelet {
 private:
  Car car;
  double delay_ = 0.0;
  double odom_rate_;
  ros::Time last_cmd_time_;
  Eigen::Vector2d input_;    //如果是mpc，那么是加速度和转角；如果是teb，那么是线速度和角速度
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber path_sub_;
  ros::Timer sim_timer_, sim_timer2_;
  ros::ServiceServer set_init_pose_srv_;
  ros::Publisher mpc_pause_pub_;

  bool has_path_ = false;
  Eigen::Vector4d initS_; //x, y, yaw, vel
  struct DelayedMsg {
    ros::Time t;
    double a, delta;
    DelayedMsg() {}
    DelayedMsg(const ros::Time& _t, double _a, double _delta) : t(_t), a(_a), delta(_delta) {}
  };
  std::deque<DelayedMsg> delayedMsgs_;

  void cmd_callback(const car_msgs::CarCmd::ConstPtr& msg) {
    // delayedMsgs_.emplace_back(ros::Time::now(), msg->a, msg->delta);
    input_(0) = msg->a;
    input_(1) = msg->delta * 1.0;
    last_cmd_time_ = ros::Time::now();
  }
  void cmdTebVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
      input_(0) = msg->linear.x;
      input_(1) = msg->angular.z;
      car.state(3) = msg->linear.x;
      last_cmd_time_ = ros::Time::now();
  }

  void timer_callback(const ros::TimerEvent& event) {
    //原程序
    // if(has_path_ == false) return;
    // if (!delayedMsgs_.empty()) {
    //   auto& msg = delayedMsgs_.front();
    //   //只有当当前时间减去msg.t（控制量产生的时间）大于延迟了，才说明该控制量可以起效果
    //   if ((ros::Time::now() - msg.t).toSec() > delay_) {
    //     input_(0) = msg.a;
    //     input_(1) = msg.delta;
    //     delayedMsgs_.pop_front();
    //   }
    // }
    // else
    // {
    //   input_(0) = input_(1) = 0.0;
    // }
    // std::cout << "-----|输入：----|" << input_(0) << ", " << input_(1) << std::endl;
    // std::cout << "-----|step前：----|" << car.state.transpose() << std::endl;
    if((ros::Time::now() - last_cmd_time_) > ros::Duration(0.1))
    {
      input_.setZero();// 如果有0.1s没有收到新的cmd，则值令设定为0
    }
      
    car.step(input_, 1.0 / odom_rate_);
    // std::cout << "-----|step后：----|" << car.state.transpose() << std::endl;
    //将car的状态作为Odometry数据发送出来可视化
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.position.x = car.state(0);
    odom_msg.pose.pose.position.y = car.state(1);
    odom_msg.pose.pose.position.z = car_height_;
    double phi = car.state(2);
    double v = car.state(3);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(phi / 2);
    odom_msg.pose.pose.orientation.w = cos(phi / 2);

    odom_msg.twist.twist.linear.x = v * cos(phi);
    odom_msg.twist.twist.linear.y = v * sin(phi);
    odom_msg.twist.twist.linear.z = 0.0;

    odom_pub_.publish(odom_msg);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(car.state(0), car.state(1), 0));

    static tf::TransformBroadcaster transform_broadcaster;
    tf::Quaternion q;
    q.setX(odom_msg.pose.pose.orientation.x);
    q.setY(odom_msg.pose.pose.orientation.y);
    q.setZ(odom_msg.pose.pose.orientation.z);
    q.setW(odom_msg.pose.pose.orientation.w);
    transform.setRotation(q);

    transform_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                              ros::Time::now(), "/world",
                                                              "/car_model"));
  }

  // 用来打印实时状态
  void timer_callback2(const ros::TimerEvent& event) {
  // ROS_INFO_STREAM("-----|STATE:|" << car.state.transpose() << "-----|INPUT:|" << input_.transpose());
    if((ros::Time::now() - last_cmd_time_) > ros::Duration(0.1))
    {
      // ROS_WARN_STREAM("No cmd for 0.1s.");
    }
  }

  bool set_pose(common_srvs::set_init_pose::Request &req, 
                common_srvs::set_init_pose::Response &res)
  {
    std_msgs::Empty emt;
    mpc_pause_pub_.publish(emt);
    input_ = Eigen::Vector2d::Zero();
    Eigen::Vector4d newPose;
    if(req.yaw >= 360) newPose = initS_;
    else 
    {
      newPose(0) = req.x;
      newPose(1) = req.y;
      newPose(2) = req.yaw * M_PI / 180.0;
      newPose(3) = 0.0;
    }
    car.setInitialState(newPose);
    ROS_WARN_STREAM("Set New Robot Pose: " << newPose.transpose());
    res.success = true;
    return true;
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getPrivateNodeHandle());
    nh.getParam("simulator/wheel_base", car.l);

    initS_.setZero();
    nh.getParam("simulator/init_state_x", initS_(0));
    nh.getParam("simulator/init_state_y", initS_(1));
    nh.getParam("simulator/init_state_z", car_height_);
    nh.getParam("simulator/phi", initS_(2));
    initS_(2) *= M_PI/180.0;
    nh.getParam("simulator/init_vel", initS_(3));
    nh.getParam("simulator/delay", delay_);
    input_.setZero();
    car.setInitialState(initS_);
    last_cmd_time_ = ros::Time::now();

    nh.param("rate/odom", odom_rate_, 100.0);
    std::cout << "odom_rate_: " << odom_rate_ << std::endl;

    nh.param("simulator/use_teb", use_teb_, false);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_test", 100); //这个需要改掉
    if(use_teb_)
      cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 100, &Nodelet::cmdTebVelCallback, this, ros::TransportHints().tcpNoDelay());
    else
      cmd_sub_ = nh.subscribe<car_msgs::CarCmd>("cmd", 100, &Nodelet::cmd_callback, this, ros::TransportHints().tcpNoDelay());
    // 以400hz的频率一直在更新state。
    // 每次来的cmd都放到queue里面，如果queue里面有cmd则拿出来作为新的input使用，都则就还是用上一次的cmd更新state
    sim_timer_ = nh.createTimer(ros::Duration(1.0 / odom_rate_), &Nodelet::timer_callback, this);
    sim_timer2_ = nh.createTimer(ros::Duration(0.1), &Nodelet::timer_callback2, this);
    set_init_pose_srv_ = nh.advertiseService("set_init_pose", &Nodelet::set_pose, this);
    mpc_pause_pub_     = nh.advertise<std_msgs::Empty>("mpc_pause", 10);

  }
};
}  // namespace car_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(car_simulator::Nodelet, nodelet::Nodelet);