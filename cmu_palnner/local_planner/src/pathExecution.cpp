#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

class LookAheadWaypointNode {
public:
    LookAheadWaypointNode(ros::NodeHandle& nh)
        : nh_(nh), lookahead_distance_(2.0) {
        odom_sub_ = nh_.subscribe("/car_odom", 1, &LookAheadWaypointNode::odomCallback, this);
        path_sub_ = nh_.subscribe("/TopoPlan/guide_path", 1, &LookAheadWaypointNode::pathCallback, this);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/path_execution_node/look_ahead_goal", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_execution_node/look_ahead_marker", 1);
        timer_ = nh_.createTimer(ros::Duration(0.05), &LookAheadWaypointNode::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, path_sub_;
    ros::Publisher waypoint_pub_, marker_pub_;
    ros::Timer timer_;

    geometry_msgs::Pose current_pose_;
    std::vector<geometry_msgs::PoseStamped> dense_path_;
    pcl::KdTreeFLANN<pcl::PointXY> path_kdtree_;
    pcl::PointCloud<pcl::PointXY>::Ptr path_cloud_{new pcl::PointCloud<pcl::PointXY>};
    double lookahead_distance_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
    }

    // 简单稠密化函数：插值每段小于0.1m
    std::vector<geometry_msgs::PoseStamped> densifyPath(const nav_msgs::Path& path, double step = 0.1) {
        std::vector<geometry_msgs::PoseStamped> result;
        if (path.poses.empty()) return result;
        result.push_back(path.poses.front());
        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto& p0 = path.poses[i-1].pose.position;
            const auto& p1 = path.poses[i].pose.position;
            double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
            int n = std::max(1, static_cast<int>(dist / step));
            for (int j = 1; j <= n; ++j) {
                geometry_msgs::PoseStamped pt;
                pt.header = path.poses[i].header;
                double ratio = static_cast<double>(j) / n;
                pt.pose.position.x = p0.x + (p1.x - p0.x) * ratio;
                pt.pose.position.y = p0.y + (p1.y - p0.y) * ratio;
                pt.pose.position.z = p0.z + (p1.z - p0.z) * ratio;
                pt.pose.orientation = path.poses[i].pose.orientation;
                result.push_back(pt);
            }
        }
        return result;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        dense_path_ = densifyPath(*msg, 0.1);

        // 构建KdTree
        path_cloud_->clear();
        for (const auto& pose : dense_path_) {
            pcl::PointXY pt;
            pt.x = pose.pose.position.x;
            pt.y = pose.pose.position.y;
            path_cloud_->push_back(pt);
        }
        if (!path_cloud_->empty())
            path_kdtree_.setInputCloud(path_cloud_);
    }

    void timerCallback(const ros::TimerEvent&) {
        if (dense_path_.empty() || path_cloud_->empty()) return;

        // 当前车辆位置
        pcl::PointXY query;
        query.x = current_pose_.position.x;
        query.y = current_pose_.position.y;

        // 查找最近点索引
        std::vector<int> idx(1);
        std::vector<float> dist(1);
        int nearest_idx = 0;
        if (path_kdtree_.nearestKSearch(query, 1, idx, dist) > 0)
            nearest_idx = idx[0];

        // 沿路径查找lookahead点
        double accumulated = 0.0;
        geometry_msgs::PoseStamped lookahead;
        bool found = false;
        for (size_t i = nearest_idx + 1; i < dense_path_.size(); ++i) {
            const auto& p1 = dense_path_[i-1].pose.position;
            const auto& p2 = dense_path_[i].pose.position;
            accumulated += std::hypot(p2.x - p1.x, p2.y - p1.y);
            if (accumulated >= lookahead_distance_) {
                lookahead = dense_path_[i];
                found = true;
                break;
            }
        }
        if (!found) lookahead = dense_path_.back();

        // 发布lookahead点
        geometry_msgs::PointStamped goal;
        goal.header = lookahead.header;
        goal.point = lookahead.pose.position;
        waypoint_pub_.publish(goal);
        publishMarker(lookahead);
    }

    void publishMarker(const geometry_msgs::PoseStamped& pose) {
        visualization_msgs::Marker marker;
        marker.header = pose.header;
        marker.ns = "lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose.pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);
        marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lookahead_waypoint_node");
    ros::NodeHandle nh;
    LookAheadWaypointNode node(nh);
    ros::spin();
    return 0;
}