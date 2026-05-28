cmake_minimum_required(VERSION 3.20)
project(meca500_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ───
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gcode REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── service interfaces ────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Table.srv"
  "srv/Goal.srv"
  DEPENDENCIES geometry_msgs
  
)

# ── planningscene node ───
add_executable(planningscene src/planningscene.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(planningscene PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_planning_scene_interface
  ${cpp_typesupport_target}
)

install(TARGETS planningscene DESTINATION lib/${PROJECT_NAME})

# ── trajectory node ───
add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${control_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_visual_tools::moveit_visual_tools
  gcode::gcode
  Eigen3::Eigen
  ${cpp_typesupport_target}
)
install(TARGETS trajectory DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ── tests ───
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
