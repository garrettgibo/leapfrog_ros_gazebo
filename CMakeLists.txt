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
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(gazebo_interface src/gazebo_interface.cpp)
target_link_libraries(gazebo_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
