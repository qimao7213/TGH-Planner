cmake_minimum_required(VERSION 2.8.3)
project(save_map_point)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  pcl_ros
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (save_map_point src/save_map_point.cpp )
target_link_libraries(save_map_point
  ${catkin_LIBRARIES}
  )

add_executable (egvg_vis src/egvg_vis.cpp )
target_link_libraries(egvg_vis
  ${catkin_LIBRARIES}
  )

