cmake_minimum_required(VERSION 2.8.3)

# Set CMP0048 policy to NEW
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(gsim_urdf_pkg VERSION 1.0.0)

# C++ Standard
set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  gazebo_ros
  urdf
  xacro
  tf
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  /opt/ros/noetic/include
)

link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    sensor_msgs
    message_runtime 
    geometry_msgs 
    urdf
    gazebo_ros
    xacro 
    tf
)

# Install files
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Plugin source file
add_library(shaking_table_plugin src/plugins/shaking_table_plugin.cpp)
target_link_libraries(shaking_table_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
