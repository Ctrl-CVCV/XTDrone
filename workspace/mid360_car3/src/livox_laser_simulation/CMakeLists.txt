cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_conversions 
  pcl_ros
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-11/gazebo)

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        )

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)
# NOTE(removed): target_link_libraries(... libprotobuf.so.9) -- 容器为 protobuf 3.6.1 (.so.17)，且插件源码未实际使用 protobuf，删除该链库

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
        ${PCL_LIBRARIES}
)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL
        LIBRARIES livox_laser_simulation
        CATKIN_DEPENDS tf
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} RayPlugin)
