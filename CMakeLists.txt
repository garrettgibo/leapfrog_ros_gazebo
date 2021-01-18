cmake_minimum_required(VERSION 3.0.2)
project(leapfrog_ros_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(gazebo_controller src/gazebo_controller.cpp)
target_link_libraries(gazebo_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
