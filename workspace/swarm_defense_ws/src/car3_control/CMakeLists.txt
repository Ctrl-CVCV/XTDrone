cmake_minimum_required(VERSION 3.0.2)
project(car3_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  actionlib
  actionlib_msgs
  move_base_msgs
  sensor_msgs
  angles
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2_ros
  actionlib actionlib_msgs move_base_msgs sensor_msgs angles
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mecanum_controller_node src/mecanum_controller_node.cpp)
target_link_libraries(mecanum_controller_node ${catkin_LIBRARIES})

add_executable(ground_truth_odom_node src/ground_truth_odom_node.cpp)
target_link_libraries(ground_truth_odom_node ${catkin_LIBRARIES})

add_executable(nav_to_pose_node src/nav_to_pose_node.cpp)
target_link_libraries(nav_to_pose_node ${catkin_LIBRARIES})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(car3_mecanum_grip_plugin SHARED src/car3_mecanum_grip_plugin.cpp)
target_link_libraries(car3_mecanum_grip_plugin
  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(car3_mecanum_grip_plugin ${catkin_EXPORTED_TARGETS})

install(TARGETS mecanum_controller_node ground_truth_odom_node nav_to_pose_node car3_mecanum_grip_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/car3_keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
