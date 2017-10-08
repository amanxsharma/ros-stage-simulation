cmake_minimum_required(VERSION 2.8.3)
project(pd_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pd_controller src/pd_controller.cpp)

target_link_libraries(pd_controller ${catkin_LIBRARIES})

