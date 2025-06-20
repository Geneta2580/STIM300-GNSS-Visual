cmake_minimum_required(VERSION 3.0.2)
project(stim300_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES STIM300_pkg
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs serial
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stim300 src/stim300_driver.cpp)
target_link_libraries(stim300
  ${catkin_LIBRARIES}
)