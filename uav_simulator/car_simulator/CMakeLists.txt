cmake_minimum_required(VERSION 3.0.2)
project(car_simulator)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  car_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp nodelet car_msgs
  DEPENDS EIGEN3
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)
