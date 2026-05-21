cmake_minimum_required(VERSION 3.22)
project(gcode)

find_package(ament_cmake REQUIRED)

add_library(gcode src/gcode.cpp)
target_include_directories(gcode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS gcode EXPORT gcodeTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES include/gcode.hpp DESTINATION include)
install(EXPORT gcodeTargets
  FILE gcodeTargets.cmake
  NAMESPACE gcode::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(gcodeTargets HAS_LIBRARY_TARGET)

ament_package()
