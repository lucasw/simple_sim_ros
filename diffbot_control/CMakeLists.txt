cmake_minimum_required(VERSION 2.8.11.2)
project(diffbot_control)

find_package(catkin REQUIRED
  COMPONENTS
  ros_control_boilerplate
  std_msgs
)

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES
  # CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# RRBot Hardware Interface
add_library(diffbot_hw_interface
  src/diffbot_hw_interface.cpp
)
target_link_libraries(diffbot_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(diffbot_hw_main src/diffbot_hw_main.cpp)
target_link_libraries(diffbot_hw_main
  diffbot_hw_interface
  ${catkin_LIBRARIES}
)
