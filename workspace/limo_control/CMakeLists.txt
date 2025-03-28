cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
 
add_executable(control_node p-control.cpp)
ament_target_dependencies(control_node rclcpp geometry_msgs nav_msgs tf2)

install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()