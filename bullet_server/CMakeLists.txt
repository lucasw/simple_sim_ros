cmake_minimum_required(VERSION 3.5)
project(bullet_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Bullet REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Anchor.msg"
  "msg/Body.msg"
  "msg/Constraint.msg"
  "msg/Face.msg"
  "msg/Impulse.msg"
  "msg/Heightfield.msg"
  "msg/Line.msg"
  "msg/Link.msg"
  "msg/Material.msg"
  "msg/Node.msg"
  "msg/SetMaterial.msg"
  "msg/SoftBody.msg"
  "msg/SoftConfig.msg"
  "msg/Tetra.msg"
  "srv/AddBody.srv"
  "srv/AddCompound.srv"
  "srv/AddConstraint.srv"
  "srv/AddImpulse.srv"
  "srv/AddHeightfield.srv"
  "srv/AddLaserScan.srv"
  "srv/AddRaycast.srv"
  "srv/SetTransform.srv"
  # "srv/SoftBody.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

include_directories(
  include
  ${BULLET_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if (false)
add_executable(bullet_server_node
  src/body.cpp
  src/bullet_server.cpp
  src/constraint.cpp
  src/raycast.cpp
  src/soft_body.cpp
)

ament_target_dependencies(bullet_server_node
  "geometry_msgs"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "shape_msgs"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "visualization_msgs"
)

rosidel_target_interfaces(bullet_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(bullet_server_node
  ${catkin_LIBRARIES}
  ${BULLET_LIBRARIES}
)
install(TARGETS bullet_server_node DESTINATION lib/${PROJECT_NAME})
endif(false)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# TODO(lucasw) how to install this
# catkin_install_python(PROGRAMS src/bullet_server/utility.py
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
