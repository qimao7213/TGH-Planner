#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>

class RvizPoseListener {
public:
    RvizPoseListener(ros::NodeHandle& nh, 
                     const std::string& pose_estimate_topic, 
                     const std::string& nav_goal_topic)
    {
        // 订阅 Pose Estimate（初始位姿）话题
        pose_estimate_sub_ = nh.subscribe(
            pose_estimate_topic, 200, &RvizPoseListener::poseEstimateCallback, this);

        // 订阅 Nav Goal（目标位姿）话题
        nav_goal_sub_ = nh.subscribe(
            nav_goal_topic, 200, &RvizPoseListener::navGoalCallback, this);
    }

    // 获取最新的 Pose Estimate 消息
    geometry_msgs::PoseWithCovarianceStamped getLatestPose() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (latest_pose_) {
            return *latest_pose_;
        } else {
            // 返回默认值
            return geometry_msgs::PoseWithCovarianceStamped();
        }
    }

    // 获取最新的 Nav Goal 消息
    geometry_msgs::PoseStamped getLatestGoal() {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (latest_goal_) {
            return *latest_goal_;
        } else {
            // 返回默认值
            return geometry_msgs::PoseStamped();
        }
    }

    bool newMsg()
    {
        bool have_new = newGoal_ || newStart_;
        newGoal_ = false;
        newStart_ = false;
        return have_new;
    }
    typedef shared_ptr<RvizPoseListener> Ptr;
private:
    // 回调函数：处理 Pose Estimate 消息
    void poseEstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = msg; // 更新最新的 Pose Estimate 消息
        newGoal_ = true;
    }

    // 回调函数：处理 Nav Goal 消息
    void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        latest_goal_ = msg; // 更新最新的 Nav Goal 消息
        newStart_ = true;
    }

private:
    ros::Subscriber pose_estimate_sub_;
    ros::Subscriber nav_goal_sub_;

    std::mutex pose_mutex_; // 用于保护最新 Pose Estimate 消息
    std::mutex goal_mutex_; // 用于保护最新 Nav Goal 消息

    // 存储最新的消息
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr latest_pose_;
    geometry_msgs::PoseStamped::ConstPtr latest_goal_;

    bool newGoal_ = false;
    bool newStart_ = false;

};

