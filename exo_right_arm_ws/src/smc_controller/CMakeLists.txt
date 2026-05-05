cmake_minimum_required(VERSION 3.8)
project(smc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(exo_control_msgs REQUIRED)
find_package(exo_utils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(smc_controller SHARED
  src/smc_controller.cpp
)

target_include_directories(smc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(smc_controller PROPERTIES
  BUILD_RPATH "${CMAKE_BINARY_DIR}/../exo_utils/lib"
  INSTALL_RPATH "$ORIGIN/../../exo_utils/lib"
)

ament_target_dependencies(smc_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
  exo_control_msgs
  trajectory_msgs
  realtime_tools
  ament_index_cpp
)

target_link_libraries(smc_controller
  Eigen3::Eigen
  exo_utils::exo_utils
)

pluginlib_export_plugin_description_file(
  controller_interface
  smc_controller_plugin.xml
)

install(
  TARGETS smc_controller
  EXPORT export_smc_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES smc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
