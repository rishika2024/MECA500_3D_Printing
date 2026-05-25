cmake_minimum_required(VERSION 3.8)
project(gcode)

find_package(ament_cmake REQUIRED)

add_library(gcode src/gcode.cpp)

target_include_directories(gcode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS gcode
  EXPORT gcodeTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  EXPORT gcodeTargets
  FILE gcodeTargets.cmake
  NAMESPACE gcode::
  DESTINATION share/${PROJECT_NAME}/cmake
)


# Generate and install gcodeConfig.cmake for downstream find_package support
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcodeConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/gcodeConfig.cmake
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/gcodeConfig.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(gcodeTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)

ament_package()