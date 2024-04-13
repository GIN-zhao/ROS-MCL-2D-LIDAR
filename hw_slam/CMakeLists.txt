cmake_minimum_required(VERSION 2.8.3)
project(hw_slam)
set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    cv_bridge
)

catkin_package(
)


include_directories(
  tool
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

set(tool_source
    tool/tool.cpp
    tool/tool.h
)

# MCL
set(rs_mcl_source
    mcl_/src/mcl.cpp
    mcl_/src/mcl.h
)

add_executable(mcl_ros mcl_/mcl_ros.cpp ${rs_mcl_source} ${tool_source})
target_link_libraries(mcl_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
