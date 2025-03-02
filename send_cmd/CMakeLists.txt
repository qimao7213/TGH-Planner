cmake_minimum_required(VERSION 3.0.2)
project(send_cmd)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG_MODE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall")
endif()

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  bspline
  # plan_manage
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp bspline
  DEPENDS EIGEN3
)

include_directories(
  include
  "include/tcp_communication.h"
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


add_executable(send_cmd
  src/send_cmd.cpp
  src/tcp_communication.cpp
  )
target_link_libraries(send_cmd 
  ${catkin_LIBRARIES}
  )
add_dependencies(send_cmd
  ${catkin_EXPORTED_TARGETS}
  )