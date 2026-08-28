cmake_minimum_required(VERSION 3.8)
project(msr_gcode)

find_package(ament_cmake REQUIRED)

add_library(msr_gcode src/gcode.cpp)

target_include_directories(msr_gcode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS msr_gcode
  EXPORT msr_gcodeTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS src/gcode_parser.py
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT msr_gcodeTargets
  FILE msr_gcodeTargets.cmake
  NAMESPACE msr_gcode::
  DESTINATION share/${PROJECT_NAME}/cmake
)


# Generate and install msr_gcodeConfig.cmake for downstream find_package support
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/msr_gcodeConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/msr_gcodeConfig.cmake
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/msr_gcodeConfig.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(msr_gcodeTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)

if(BUILD_TESTING)
  find_package(Catch2 REQUIRED)
  add_executable(test_parser test/test_parser.cpp)
  target_link_libraries(test_parser PRIVATE msr_gcode Catch2::Catch2WithMain)
  add_test(NAME test_parser COMMAND test_parser)
endif()

ament_package()