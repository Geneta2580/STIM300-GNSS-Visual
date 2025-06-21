cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

## Compile as C++11, supported in ROS Kinetic and newer
## You might need C++14 or C++17 if using std::filesystem
## For C++17 (e.g., for std::filesystem, common in Noetic):
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
## For C++11:
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  camera_info_manager
)

# Needed if using std::filesystem with C++17 and linking against stdc++fs
# if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.12" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "8")
#   list(APPEND LINKER_LIBS "-lstdc++fs")
# elseif(CMAKE_VERSION VERSION_GREATER_EQUAL "3.12" AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "7")
#   list(APPEND LINKER_LIBS "-lc++fs")
# endif()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME} # If you were building a library
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs camera_info_manager
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stereo_cam_node src/camera_driver.cpp)
target_link_libraries(stereo_cam_node
  ${catkin_LIBRARIES}
  # ${OpenCV_LIBRARIES} # cv_bridge should bring this in
  # ${LINKER_LIBS} # If using std::filesystem
)

# If you have launch files or config files
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )