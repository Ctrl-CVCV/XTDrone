cmake_minimum_required(VERSION 3.0.2)
project(car3_swarm)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs gazebo_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 1.6: /gazebo/model_states -> /carN/shared_pose (Odometry, map frame)
add_executable(car_state_broadcaster src/car_state_broadcaster.cpp)
target_link_libraries(car_state_broadcaster ${catkin_LIBRARIES})

# 1.7: inject other cars' virtual boxes into this car's scan
add_executable(virtual_obstacle_node src/virtual_obstacle_node.cpp)
target_link_libraries(virtual_obstacle_node ${catkin_LIBRARIES})

install(TARGETS car_state_broadcaster virtual_obstacle_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
