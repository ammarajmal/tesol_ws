cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_plugins)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs gazebo_ros)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add the plugin
add_library(shaking_plugin SHARED shaking_plugin.cpp)

# Link against Gazebo and ROS libraries
target_link_libraries(shaking_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
