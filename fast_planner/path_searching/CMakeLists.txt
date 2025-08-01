cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  nav_msgs
  dynavoro
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS plan_env dynavoro
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG_MODE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native -msse")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0 -march=native -msse")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -msse")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -msse")
endif()

add_library( path_searching 
    src/kinodynamic_astar.cpp 
    src/kinodynamic_astar_2D.cpp 
    src/astar.cpp
    src/astar_2D.cpp
    src/topo_prm.cpp
    src/dubins.cpp
    src/jump_point_search.cpp
    )
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    )  
