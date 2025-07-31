cmake_minimum_required(VERSION 3.0.2)
project(car_mpc)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG_MODE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -march=native -msse")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -march=native -msse")
endif()

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  car_msgs
  bspline
  pluginlib
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp nodelet car_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  "include/car_mpc.hpp"
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  osqp::osqp 
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)
