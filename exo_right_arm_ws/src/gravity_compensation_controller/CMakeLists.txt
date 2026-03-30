cmake_minimum_required(VERSION 3.8)
project(gravity_compensation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_library(RBDL_LIBRARY NAMES rbdl RBDL DOC "RBDL shared library")
find_path(RBDL_INCLUDE_DIR NAMES rbdl/rbdl.h DOC "RBDL headers")

set(HAVE_RBDL FALSE)
if(RBDL_LIBRARY AND RBDL_INCLUDE_DIR)
  find_package(urdf REQUIRED)
  set(HAVE_RBDL TRUE)
  message(STATUS "gravity_compensation_controller: RBDL backend enabled (${RBDL_LIBRARY})")
else()
  message(STATUS "gravity_compensation_controller: RBDL not found; only Pinocchio backend available")
endif()

add_library(gravity_compensation_controller SHARED
  src/gravity_compensation_controller.cpp
  src/dynamics_backend.cpp
)

target_include_directories(gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(gravity_compensation_controller PRIVATE GRAVITY_COMP_HAVE_PINOCCHIO)

if(HAVE_RBDL)
  target_compile_definitions(gravity_compensation_controller PRIVATE GRAVITY_COMP_HAVE_RBDL)
  target_include_directories(gravity_compensation_controller PRIVATE ${RBDL_INCLUDE_DIR})
endif()

ament_target_dependencies(gravity_compensation_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  trajectory_msgs
  realtime_tools
  ament_index_cpp
)

# Plain signature: ament_target_dependencies uses plain target_link_libraries; CMake
# disallows mixing that with PUBLIC/PRIVATE on the same target.
target_link_libraries(gravity_compensation_controller
  pinocchio::pinocchio
  Eigen3::Eigen
)

if(HAVE_RBDL)
  target_link_libraries(gravity_compensation_controller
    ${RBDL_LIBRARY}
    urdf::urdf
  )
endif()

pluginlib_export_plugin_description_file(
  controller_interface
  gravity_compensation_controller_plugin.xml
)

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
