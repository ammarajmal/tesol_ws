cmake_minimum_required(VERSION 3.0.2)
project(sony_cam)
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp sensor_msgs
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node node/sony_cam.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
