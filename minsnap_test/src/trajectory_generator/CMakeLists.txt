cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  quadrotor_msgs
  traj_utils
  nav_msgs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs traj_utils mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(minsnapCloseform
  include/min_snap/min_snap_closeform.h
  src/min_snap_closeform.cpp
)

target_link_libraries(minsnapCloseform ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/trajectory_generator.cpp)

target_link_libraries(${PROJECT_NAME}_node minsnapCloseform ${catkin_LIBRARIES})

