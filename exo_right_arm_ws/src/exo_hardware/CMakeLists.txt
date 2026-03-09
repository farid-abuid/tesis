cmake_minimum_required(VERSION 3.8)
project(exo_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
#find_package(serial REQUIRED)

add_library(exo_hardware SHARED
  src/exo_hardware.cpp
)

target_include_directories(exo_hardware PUBLIC
  include
)

ament_target_dependencies(exo_hardware
  rclcpp
  pluginlib
  hardware_interface
  #serial
)

#target_link_libraries(serial)

pluginlib_export_plugin_description_file(
  hardware_interface
  exo_hardware_plugins.xml
)

install(
  TARGETS exo_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()