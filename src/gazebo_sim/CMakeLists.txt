cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim)

# Find and include catkin and Gazebo
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Add plugin subdirectory
add_subdirectory(plugins)
