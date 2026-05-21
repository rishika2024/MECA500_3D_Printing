cmake_minimum_required(VERSION 3.22)
project(meca500_demos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(trajectory meca500_demos/trajectory.cpp)
ament_target_dependencies(trajectory
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  moveit_visual_tools
  visualization_msgs
  control_msgs
)
install(TARGETS trajectory DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()