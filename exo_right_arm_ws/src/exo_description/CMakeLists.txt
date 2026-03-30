cmake_minimum_required(VERSION 3.8)
project(exo_description)

find_package(ament_cmake REQUIRED)

find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(_exo_build_xacro "${CMAKE_CURRENT_BINARY_DIR}/exo_build.xacro")
set(_exo_dynamics_urdf "${CMAKE_CURRENT_BINARY_DIR}/exo_dynamics.urdf")
add_custom_command(
  OUTPUT ${_exo_build_xacro}
  COMMAND ${CMAKE_COMMAND}
    -DIN=${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.xacro
    -DOUT=${_exo_build_xacro}
    -DURDF_DIR=${CMAKE_CURRENT_SOURCE_DIR}/urdf
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rewrite_xacro_paths.cmake
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rewrite_xacro_paths.cmake
  VERBATIM
)
add_custom_command(
  OUTPUT ${_exo_dynamics_urdf}
  COMMAND ${XACRO_EXECUTABLE} ${_exo_build_xacro} hardware:=real -o ${_exo_dynamics_urdf}
  DEPENDS
    ${_exo_build_xacro}
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/materials.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.trans
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.gazebo
  VERBATIM
)
add_custom_target(generate_exo_dynamics_urdf ALL DEPENDS ${_exo_dynamics_urdf})

install(
  DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${_exo_dynamics_urdf}
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME exo_dynamics.urdf
)

ament_package()
