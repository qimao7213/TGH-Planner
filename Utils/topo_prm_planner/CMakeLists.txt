cmake_minimum_required(VERSION 2.8.3)
project(topo_prm_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 3.1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  plan_env
  path_searching
  traj_utils
  cv_bridge
)

  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 LIBRARIES topo_prm_planner
 CATKIN_DEPENDS plan_env path_searching traj_utils
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(topo_prm_planner src/topo_prm_plan_node.cc)
target_link_libraries(topo_prm_planner ${catkin_LIBRARIES})
add_dependencies(topo_prm_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(publish_nav_goal src/publish_nav_goal.cc)
target_link_libraries(publish_nav_goal ${catkin_LIBRARIES})
add_dependencies(publish_nav_goal ${${PROJECT_NAME}_EXPORTED_TARGETS})
