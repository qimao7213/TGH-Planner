#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string trajFile;
string runtimeFile;

double transInterval = 0.2;
double yawInterval = 10.0;

pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = true;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, timeDuration = 0;
std::vector<float> runtimes; // gvd的update time和egvg的generate time

ros::Publisher *pubTrajectoryPtr = NULL;

FILE *runtimeFilePtr = NULL;
FILE *trajFilePtr = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  systemTime = odom->header.stamp.toSec();
  
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
  }

  if (dis < transInterval && dYaw < yawInterval) {
    return;
    
  }
  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  travelingDis += dis;

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;
  // ROS_WARN_STREAM ("time: " << timeDuration << "  dis: " << travelingDis << "  x: " << vehicleX << "  y: " << vehicleY << "  z: " << vehicleZ);
  fprintf(trajFilePtr, "%f %f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, travelingDis, timeDuration);
  std::cout << "TimeDuration: " << timeDuration << std::endl;
  // ROS_WARN("xxx");
  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "world";
  pubTrajectoryPtr->publish(trajectory2);
}


void runtimeHandler(const std_msgs::Float32MultiArray::ConstPtr& runtimeIn)
{
  runtimes = runtimeIn->data;
  fprintf(runtimeFilePtr, "%f %f %f %f\n", runtimes[0], runtimes[1], (runtimes[0] + runtimes[1]), timeDuration);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualizationTools");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("trajFile", trajFile);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/car_odom", 5, odometryHandler);

  ros::Subscriber subRuntime = nh.subscribe<std_msgs::Float32MultiArray> ("/egvg_runtime", 5, runtimeHandler);

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2> ("/planning/trajectory_real", 5);
  pubTrajectoryPtr = &pubTrajectory;


  std::string file_path;
  std::string default_path = "ChangeToYourPath";
  nhPrivate.param("FilePath", file_path, default_path);
  std::string traj_real_file_path = file_path + "Utils/traj_analysis/traj_real_egvg.txt";
  std::string runtime_file_path = file_path + "Utils/traj_analysis/egvg_runtime.txt";

  trajFile = traj_real_file_path;
  trajFilePtr = fopen(trajFile.c_str(), "w");

  if (!trajFilePtr) {
      ROS_ERROR_STREAM("Unable to open file to save traj info! at: " << traj_real_file_path);
  } else {
      ROS_INFO_STREAM("Successfully opened traj file: " << traj_real_file_path);
  }

  runtimeFile = runtime_file_path;
  runtimeFilePtr = fopen(runtimeFile.c_str(), "w");

  if (!runtimeFilePtr) {
      ROS_ERROR_STREAM("Unable to open file to save traj info! at: " << runtime_file_path);
  } else {
      ROS_INFO_STREAM("Successfully opened traj file: " << runtime_file_path);
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();


    status = ros::ok();
    rate.sleep();
  }

  fclose(runtimeFilePtr);
  fclose(trajFilePtr);

  return 0;
}
