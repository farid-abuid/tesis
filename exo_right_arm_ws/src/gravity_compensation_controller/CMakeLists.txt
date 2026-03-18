cmake_minimum_required(VERSION 3.8)
project(gravity_compensation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Library
add_library(gravity_compensation_controller SHARED
  src/gravity_compensation_controller.cpp
)

target_include_directories(gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gravity_compensation_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
)

# Export plugin
pluginlib_export_plugin_description_file(
  controller_interface
  gravity_compensation_controller_plugin.xml
)

# Install
install(
  TARGETS gravity_compensation_controller
  EXPORT export_gravity_compensation_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES gravity_compensation_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()