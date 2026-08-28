cmake_minimum_required(VERSION 3.20)
project(msr_meca500_print_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ───
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# ros-kilted-moveit-ros-planning 2.14.3's exported CMake target lists
# "tl::expected" in its INTERFACE_LINK_LIBRARIES, but ros-kilted-tl-expected
# only defines the target as tl_expected::tl_expected -- an upstream
# packaging mismatch between the two. Alias it before pulling in
# moveit_ros_planning_interface so its find_package() below doesn't fail
# trying to resolve a target that was never actually exported under that name.
find_package(tl_expected REQUIRED)
if(NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(msr_gcode REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── service interfaces ────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Table.srv"
  "srv/Goal.srv"
  "srv/GcodeFile.srv"
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
  moveit_ros_planning_interface::moveit_move_group_interface
  Eigen3::Eigen
  ${cpp_typesupport_target}
)

install(TARGETS planningscene DESTINATION lib/${PROJECT_NAME})

# ── bed pose setter (default, or fit from nozzle touch poses) ───
add_executable(bed_from_touches src/bed_from_touches.cpp)
target_link_libraries(bed_from_touches PUBLIC
  rclcpp::rclcpp
  Eigen3::Eigen
  ${cpp_typesupport_target}
)
install(TARGETS bed_from_touches DESTINATION lib/${PROJECT_NAME})

# ── gcode print executor node ───
add_executable(gcode_print_executor src/gcode_print_executor.cpp)
target_link_libraries(gcode_print_executor PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${control_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_visual_tools::moveit_visual_tools
  msr_gcode::msr_gcode
  Eigen3::Eigen
  ${cpp_typesupport_target}
)
install(TARGETS gcode_print_executor DESTINATION lib/${PROJECT_NAME})

# ── reachability node ───
add_executable(reachability src/reachability.cpp)
target_link_libraries(reachability PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  Eigen3::Eigen
)
install(TARGETS reachability DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  src/start_print_sequence.py
  src/end_print_sequence.py
  DESTINATION share/${PROJECT_NAME})

# ── tests ───
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
