cmake_minimum_required(VERSION 3.0.2)
project(gazebo_simulator)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")
find_package(OpenCV 4.7.0 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_ros
  gazebo_msgs
  cv_bridge_1
  image_transport
  tf2_ros
  pcl_ros
)
catkin_package(
  DEPENDS Eigen
)
include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
                    "/usr/include/eigen3")

add_executable(set_robot_pose src/set_robot_pose.cc)
target_link_libraries(set_robot_pose ${catkin_LIBRARIES})

add_executable(record_gazebo_data src/record_gazebo_data.cc)
target_link_libraries(record_gazebo_data ${catkin_LIBRARIES})
