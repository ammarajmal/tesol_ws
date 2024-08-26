cmake_minimum_required(VERSION 3.0.2)
project(tesol_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  fiducial_msgs
  cv_bridge
  image_transport
  message_runtime
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/nodelet_april.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
