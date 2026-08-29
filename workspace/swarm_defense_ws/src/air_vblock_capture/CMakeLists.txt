cmake_minimum_required(VERSION 3.0.2)
project(air_vblock_capture)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  gazebo_msgs
  mavros_msgs
  visualization_msgs
)

catkin_package()

install(PROGRAMS
  scripts/vblock_capture_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
