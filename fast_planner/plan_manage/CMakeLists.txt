cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -msse")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -msse")
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# find_package(glog REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  rosparam
  common_srvs
  dynavoro
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt traj_utils message_runtime dynavoro
#  DEPENDS system_lib
)

include_directories(
  include
  "/src/topo_test/rviz_subscriber.h"
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(fast_planner_node
  src/fast_planner_node.cpp 
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(fast_planner_node 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  # glog::glog
  )

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(test_topo_rpm src/topo_test/test_topo_rpm.cc)
target_link_libraries(test_topo_rpm ${catkin_LIBRARIES})
add_dependencies(test_topo_rpm ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(publish_nav_goal src/topo_test/publish_nav_goal.cc)
target_link_libraries(publish_nav_goal ${catkin_LIBRARIES})
add_dependencies(publish_nav_goal ${${PROJECT_NAME}_EXPORTED_TARGETS})
