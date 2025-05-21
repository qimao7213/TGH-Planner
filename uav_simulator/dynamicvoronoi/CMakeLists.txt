cmake_minimum_required(VERSION 3.12.2)
project(dynavoro)

## 找到 catkin 的构建工具
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs
)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -msse")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -msse")

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

## 包含头文件目录
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(dynavoro
    src/voronoi_layer.cpp
    src/dynamicvoronoi.cpp
)   

target_link_libraries( dynavoro
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${OPNECV_LIBSRARIES}
    )  


add_executable(mainLauMethod
    src/mainLauMethod.cpp 
    )
target_link_libraries(mainLauMethod 
    ${catkin_LIBRARIES} dynavoro)

