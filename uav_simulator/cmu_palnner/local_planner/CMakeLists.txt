cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(localPlanner src/localPlanner.cpp)
add_executable(pathFollower src/pathFollower.cpp)
add_executable(pathExecution src/pathExecution.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localPlanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pathFollower ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pathExecution ${catkin_LIBRARIES} ${PCL_LIBRARIES})



#install(TARGETS localPlanner pathFollower
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
#install(DIRECTORY paths
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paths
#)
