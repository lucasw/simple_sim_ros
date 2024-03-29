cmake_minimum_required(VERSION 3.0.2)
project(sim_clock)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rosgraph_msgs
  roslint
)

roslint_cpp()
roslint_python()

generate_dynamic_reconfigure_options(
  cfg/Clock.cfg
)

catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  roscpp
  rosgraph_msgs
)

add_executable(clock src/clock.cpp)
add_dependencies(clock ${PROJECT_NAME}_gencfg)
target_link_libraries(clock ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(timer_test src/timer_test.cpp)
target_link_libraries(timer_test ${catkin_LIBRARIES})

install(TARGETS
  clock
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/clock.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
