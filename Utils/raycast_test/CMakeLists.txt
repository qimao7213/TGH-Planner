cmake_minimum_required(VERSION 2.8.3)
project(raycast_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  plan_env
)
catkin_package(
 LIBRARIES raycast_test
 CATKIN_DEPENDS roscpp std_msgs plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall")


add_executable(raycast_test
    raycast_test.cpp 
    )
target_link_libraries(raycast_test ${catkin_LIBRARIES})