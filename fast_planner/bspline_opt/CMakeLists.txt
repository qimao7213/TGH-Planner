cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge_1
#  nlopt
  plan_env
  bspline
)

set (NLOPT_DIR "/home/bhrqhb/ThirdLibrary/nlopt-2.7.1/")
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/nlopt")
list(APPEND CMAKE_FIND_ROOT_PATH "/usr/local/lib/cmake/nlopt")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    # ${NLOPT_LIBRARIES}
    /usr/local/lib/libnlopt.so
    glog::glog
    )  

add_executable( nlopt_test src/nlopt_test.cpp)
target_link_libraries(nlopt_test 
    ${catkin_LIBRARIES} 
    # ${NLOPT_LIBRARIES}
    /usr/local/lib/libnlopt.so
    glog::glog)
