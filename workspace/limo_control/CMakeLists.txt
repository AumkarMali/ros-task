cmake_minimum_required(VERSION 3.8)
project(limo_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(goto_controller src/goto_controller.cpp)
ament_target_dependencies(goto_controller
  rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

install(TARGETS goto_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
