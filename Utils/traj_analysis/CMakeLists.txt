cmake_minimum_required(VERSION 3.10)  # 指定CMake的最小版本要求

project(traj_analysis)  # 项目名称

find_package(PythonLibs 3.8 REQUIRED)  # 查找Python库
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(NumPy REQUIRED)  # 查找NumPy库
find_package(catkin REQUIRED)
catkin_package()
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")

# 包含matplotlibcpp头文件
include_directories(
    ${NUMPY_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    "/usr/include/numpy"
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    )  # 替换为matplotlibcpp.h的实际路径

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
# 添加可执行文件
add_executable(calPathInfo src/calPathInfo.cpp)  # 替换src/main.cpp为你的源文件路径
target_link_libraries(calPathInfo ${PYTHON_LIBRARIES})

# 添加可执行文件
add_executable(calTrajInfo src/calTrajInfo.cpp)  # 替换src/main.cpp为你的源文件路径
target_link_libraries(calTrajInfo ${PYTHON_LIBRARIES})

# 如果需要链接NumPy库
# target_link_libraries(MyExecutable ${NUMPY_LIBRARIES})

# 如果你的系统中有多个Python版本，你可能需要指定版本
# set(PYTHON_VERSION 3.7)  # 替换为你的Python版本
# find_package(PythonLibs ${PYTHON_VERSION} EXACT REQUIRED)
add_executable(drawTrajMap src/drawTrajMap.cpp)  # 替换src/main.cpp为你的源文件路径
target_link_libraries(drawTrajMap ${PYTHON_LIBRARIES} ${OpenCV_LIBS})