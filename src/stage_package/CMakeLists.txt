cmake_minimum_required(VERSION 2.8.3)
project(stage_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(stage_exe src/stage_source.cpp)

target_link_libraries(stage_exe ${catkin_LIBRARIES})

