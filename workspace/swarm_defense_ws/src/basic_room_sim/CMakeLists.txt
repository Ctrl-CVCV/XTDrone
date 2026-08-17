cmake_minimum_required(VERSION 3.0.2)
project(basic_room_sim)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
