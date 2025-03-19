#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <fstream>
#include <string>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <path_searching/topo_prm.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <plan_manage/plan_container.hpp>
#include "rviz_subscriber.h"
#include <traj_utils/planning_visualization.h>

using namespace fast_planner;
using namespace std;

const double origin_x_ = -4;
const double origin_y_ = -4;
const int imgHeight = 800;

Eigen::Vector2i worldToPixel(double world_x, double world_y, double resolution) {
    int pixel_x = static_cast<int>((world_x - origin_x_) / resolution);
    int pixel_y = imgHeight - static_cast<int>((world_y - origin_y_) / resolution);
    return Eigen::Vector2i(pixel_x, pixel_y);
}

void readTrajectory(std::ifstream& inFile, std::vector<Eigen::Vector2i>& points) {
    double x, y, t;
    std::string line;
    
    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        double temp;
        // 读取前三个数据
        if (iss >> t >> x >> y) {
            points.push_back(worldToPixel(x, y, 0.02));
        } else {
            // 处理读取错误或数据不足的情况
            std::cerr << "Error reading data from line: " << line << std::endl;
        }
        // 忽略该行剩余的数据
        while (iss >> temp) {
            // 仅忽略剩余数据，不做处理
        }
    }
    inFile.close();
    std::cout << "Read " << points.size() << " points." << std::endl;
}



void saveRecordData(const RecordData& record_date, std::ofstream& file)
{
    int path_num = 0;
    double path_length_sum = 0.0;
    double path_length_mean = 0.0;
    for(int i = 0; i < record_date.path_lengths.size(); ++i)
    {
        if(record_date.path_lengths[i] > 1e-3)
        {
            path_num++;
            path_length_sum += record_date.path_lengths[i];
        }
    }
    if(path_num != 0)
        path_length_mean = path_length_sum / path_num;
    else path_length_mean = -1;
    file << record_date.sample_num << " " << record_date.path_num << " " << record_date.path_lengths[0] << " " << 
    record_date.path_lengths[1] << " " << record_date.path_lengths[2] << " " << record_date.path_lengths[3] << " " << 
    record_date.path_lengths[4] << " " << path_length_mean << " " << record_date.run_time << " " << record_date.preprocess_time << std::endl;
    // std::cout << record_date.sample_num << " " << record_date.path_num << " " << record_date.path_lengths[0] << " " << 
    // record_date.path_lengths[1] << " " << record_date.path_lengths[2] << " " << record_date.path_lengths[3] << " " << 
    // record_date.path_lengths[4] << " " << path_length_mean << " " << record_date.run_time << std::endl;
}


class TestTopoRpm{

public:
    TestTopoRpm(/* args */) {
    }
    ~TestTopoRpm() {
    }
    void init(ros::NodeHandle& nh)
    {
        // nh_ = nh;
        // SDFMap::Ptr sdf_map_;
        sdf_map_.reset(new SDFMap);
        sdf_map_->initMapFromCostMap(nh); // 这里订阅了地图
        sdf_map_->setNeed2DMap(true);
        edt_environment_.reset(new EDTEnvironment);
        edt_environment_->setMap(sdf_map_);  
        visualization_.reset(new PlanningVisualization(nh));
        topo_prm_.reset(new TopologyPRM);
        topo_prm_->setEnvironment(edt_environment_);
        topo_prm_->initForTest(nh);
        topo_prm_->setOnly2D(true);     
        listener_ = std::make_shared<RvizPoseListener>(nh, "/initialpose", "/move_base_simple/goal");
        start_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/start_pose", 10);
        end_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);

        record_file_.open("/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_manage/maps/record_data/record_data.txt", std::ios::out);
        if(!record_file_.is_open())
        {
            ROS_ERROR("Unable to open file to save traj info!");
        }
        std::cout << "------ TestTopoRpm init success ------" << std::endl; 
    }


    // 不管重新选rviz的起点or终点都会触发规划
    // 起点根据选择的来，终点是固定的
    void run()
    {
        while (listener_->newMsg())
        {
            Eigen::Vector3d start_state, goal_state;
            auto start_pose = listener_->getLatestPose();
            auto goal_pose = listener_->getLatestGoal();
            if(start_pose.header.stamp != ros::Time(0)){
                start_state << start_pose.pose.pose.position.x, 
                               start_pose.pose.pose.position.y,
                               tf::getYaw(start_pose.pose.pose.orientation);
            }else{
                // start_state << -8.86, -1.3, 0.0; // map_dense
                start_state << -10.0, 7.7, 0.0; // map_maze
            }
            if(goal_pose.header.stamp != ros::Time(0)){
                goal_state << goal_pose.pose.position.x, 
                               goal_pose.pose.position.y,
                               tf::getYaw(goal_pose.pose.orientation);
            }
            // else
            {
                // goal_state << 9.54, -0.3, 0.0;     // map_dense 
                goal_state << 12.3, -8.3, 0.0; // map_maze                          
            }


            publishStartPose(start_state(0), start_state(1), -0.5, start_state(2));
            publishEndPose(goal_state(0), goal_state(1), -0.5, goal_state(2));

            if((goal_state - last_goal_).norm() > 0.25) topo_prm_->resetPathContainer();


            std::vector<Eigen::Vector3d> start_change;
            start_change.emplace_back(Eigen::Vector3d(start_state(0), start_state(1), -0.5));
            // topo_prm_->setStartChange(start_change);

            list<GraphNode::Ptr>            graph;
            vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
            topo_prm_->findTopoPaths(start_state, goal_state, vector<Eigen::Vector3d>{}, vector<Eigen::Vector3d>{}, graph,
                               raw_paths, filtered_paths, select_paths);
            vector<Eigen::Vector3d> dubins_shot_path;
            ros::Time t1, t2;
            t1 = ros::Time::now();
            // dubins_shot_path = topo_prm_->findDubinsShots(start_state, 0.75);
            t2 = ros::Time::now();
            // std::cout << "Dubins shot time: " << (t2 - t1).toSec() * 1000 << " ms." << std::endl;
            visualization_->drawTopoGraph(graph, 0.2, 0.05, Eigen::Vector4d(1.0, 0, 0.0, 1.0), 
                                  Eigen::Vector4d(0.0, 1, 0.0, 1.0), Eigen::Vector4d(0.0, 1, 0.0, 1.0));
            vector<vector<Eigen::Vector3d>> path_front= topo_prm_->getPathContainer(1);
            vector<vector<Eigen::Vector3d>> path_back= topo_prm_->getPathContainer(2);           
            // vector<vector<Eigen::Vector3d>> path_container_vis(path_container.begin(), path_container.begin() + 4);
            visualization_->drawTopoPathsPhase1(path_back, 0.05);
            visualization_->drawTopoPathsPhase2(path_front, 0.05);
            // visualization_->drawTopoSampleArea(topo_prm_->getSampleArea(), 0.05, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));
            visualization_->drawGuidePath(dubins_shot_path, 0.075, Eigen::Vector4d(0.5, 0.5, 0.0, 1.0));

            last_goal_ = goal_state;
            RecordData record_data = topo_prm_->getRecordData();
            saveRecordData(record_data, record_file_);
        }
    }

    typedef shared_ptr<TestTopoRpm> Ptr;
    RvizPoseListener::Ptr listener_;

private:
    SDFMap::Ptr sdf_map_;
    EDTEnvironment::Ptr edt_environment_;
    ros::NodeHandle nh_;
    unique_ptr<TopologyPRM> topo_prm_;
    GlobalTrajData global_data_; //给topo用的

    PlanParameters pp_;                       // 规划相关的参数
    MidPlanData plan_data_;                   // kino用来存放轨迹，和yaw角的规划，但是放yaw角的规划在这里由什么用呢？

    PlanningVisualization::Ptr visualization_;
    Eigen::Vector3d last_goal_;
    std::ofstream record_file_;

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

    void close_file(){record_file_.close();}
    void calTrajESDF(std::string trajFileName)
    {
        std::string inFileName = trajFileName + ".txt";
        std::string outFileName = trajFileName + "_esdf.txt";

        std::ofstream outFile;
        std::ifstream inFile;

        outFile.open(outFileName, std::ios::out);
        inFile.open(inFileName);
        if(!outFile.is_open() || !inFile.is_open())
        {
            ROS_ERROR("Unable to open file to save traj info!");
        }
        std::vector<Eigen::Vector2i> trajPoints; //
        readTrajectory(inFile, trajPoints);
        std::vector<double> trajPointESDF;
        for(int i = 0; i < trajPoints.size(); ++i)
        {
            double esdf_v = sdf_map_->getDistance2D(trajPoints[i]);
            trajPointESDF.push_back(esdf_v);
        }
        topo_prm_->publishTestPath(trajPoints, 1);
        for (int i = 0; i < trajPointESDF.size(); ++i)
        {
            outFile << trajPointESDF[i] << std::endl;
        }
        // 关闭文件
        outFile.close();
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



