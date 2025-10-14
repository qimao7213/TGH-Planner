#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <fstream>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>

#include <dvr/voronoi_layer.h>

using namespace std;

int last_topo_voronoi_path_num_ = 0;
ros::Publisher voro_path_pub_;
Eigen::Vector4d getColor(double h, double alpha) {
  if (h < 0.0 || h > 1.0) {
    std::cout << "h out of range" << std::endl;
    h = 0.0;
  }

  double          lambda;
  Eigen::Vector4d color1, color2;
  if (h >= -1e-4 && h < 1.0 / 6) {
    lambda = (h - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);

  } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);

  } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);

  } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);

  } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);

  } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
    lambda = (h - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3)              = alpha;

  return fcolor;
}

void displayLineList(const vector<Eigen::Vector3d>& list1,
                                            const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  voro_path_pub_.publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  voro_path_pub_.publish(mk);

  ros::Duration(0.001).sleep();
}

void drawTopoVoronoiPaths(vector<vector<Eigen::Vector3d>>& paths, double size) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_voronoi_path_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, size, color1, i % 100, 12);
  }

  last_topo_voronoi_path_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, size, getColor(double(i) / (last_topo_voronoi_path_num_), 0.8),
                    i % 100, 12);
  }
}






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
        bool have_new = newGoal_ && newStart_;
        if (have_new) {
            // std::cout << "New goal and start received." << std::endl;
            newGoal_ = false;
            newStart_ = false; 
        }
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
    


class VoronoiPlanner{

public:
    VoronoiPlanner(/* args */) {
    }
    ~VoronoiPlanner() {
    }
    void init(ros::NodeHandle& nh)
    {  
        listener_ = std::make_shared<RvizPoseListener>(nh, "/initialpose", "/move_base_simple/goal");
        start_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/start_pose", 10);
        end_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);
        voro_path_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/voronoi_topo_path", 1);
        voronoi_layer_ = std::make_shared<DynaVoro::VoronoiLayer>(nh);
        std::cout << "------ VoronoiPlanner init success ------" << std::endl; 
    }


    void run()
    {
        while (listener_->newMsg())
        {
            Eigen::Vector3d start_state, goal_state;
            auto start_pose = listener_->getLatestPose();
            auto goal_pose = listener_->getLatestGoal();
            if(start_pose.header.stamp != ros::Time(0))
            {
                start_state << start_pose.pose.pose.position.x, 
                               start_pose.pose.pose.position.y,
                               tf::getYaw(start_pose.pose.pose.orientation);
            }else{
                // start_state << -8.86, -1.3, 0.0; // map_dense
                // start_state << -10.8, 8.0, 0.0; // map_maze
                continue;
            }
            if(goal_pose.header.stamp != ros::Time(0))
            {
                goal_state << goal_pose.pose.position.x, 
                               goal_pose.pose.position.y,
                               tf::getYaw(goal_pose.pose.orientation);
            }
            else
            {
                // goal_state << 9.54, -0.3, 0.0;     // map_dense 
                // goal_state << 12.3, -8.3, 0.0; // map_maze     
                continue;                     
            }
            // start_state(0) -= 60.0; start_state(1) -= 60.0;
            // goal_state(0) -= 60.0;  goal_state(1) -= 60.0;


            publishStartPose(start_state(0), start_state(1), 0.2, start_state(2));
            publishEndPose(goal_state(0), goal_state(1), 0.2, goal_state(2));
            std::cout << "start_state: " << start_state.transpose() << ", goal_state: " << goal_state.transpose() << std::endl;
            voronoi_layer_->plan(start_state, goal_state, start_state(2));

            std::vector<std::vector<Eigen::Vector3d>> voro_paths = voronoi_layer_->getVoroPaths(0.5);
            drawTopoVoronoiPaths(voro_paths, 0.1);
        }
    }

    typedef shared_ptr<VoronoiPlanner> Ptr;
    RvizPoseListener::Ptr listener_;

private:

    Eigen::Vector3d last_goal_;
    std::ofstream record_file_;
    std::shared_ptr<DynaVoro::VoronoiLayer> voronoi_layer_;
public:
    void publishStartPose(double x, double y, double z, double yaw) {
        geometry_msgs::PoseStamped start_pose;
        start_pose.header.stamp = ros::Time::now();
        start_pose.header.frame_id = "world";

        start_pose.pose.position.x = x;
        start_pose.pose.position.y = y;
        start_pose.pose.position.z = z;

        start_pose.pose.orientation = createQuaternionFromYaw(yaw);

        // Publish
        start_publisher_.publish(start_pose);
        // ROS_INFO_STREAM("Published Start Pose: [" << x << ", " << y << ", " << z << ", yaw: " << yaw << "]");
    }

    void publishEndPose(double x, double y, double z, double yaw) {
        geometry_msgs::PoseStamped end_pose;
        end_pose.header.stamp = ros::Time::now();
        end_pose.header.frame_id = "world";

        end_pose.pose.position.x = x;
        end_pose.pose.position.y = y;
        end_pose.pose.position.z = z;

        end_pose.pose.orientation = createQuaternionFromYaw(yaw);

        // Publish
        end_publisher_.publish(end_pose);
        // ROS_INFO_STREAM("Published End Pose: [" << x << ", " << y << ", " << z << ", yaw: " << yaw << "]");
    }
    void closeSaveFile()
    {
        this->voronoi_layer_->closeSaveFile();
    }

private:
    ros::Publisher start_publisher_;
    ros::Publisher end_publisher_;

    // Helper function to create a quaternion from a yaw angle
    geometry_msgs::Quaternion createQuaternionFromYaw(double yaw) {
        geometry_msgs::Quaternion q;
        q.w = cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw * 0.5);
        return q;
    }
};



