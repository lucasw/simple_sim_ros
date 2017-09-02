cmake_minimum_required(VERSION 2.8.3)
project(sim_clock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(timer_test src/timer_test.cpp)
target_link_libraries(timer_test ${catkin_LIBRARIES})
