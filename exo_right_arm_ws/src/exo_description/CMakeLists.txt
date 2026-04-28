cmake_minimum_required(VERSION 3.8)
project(exo_description)

find_package(ament_cmake REQUIRED)

find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(_exo_build_xacro "${CMAKE_CURRENT_BINARY_DIR}/exo_build.xacro")
# exo_dynamics.urdf is the right-arm single config (kept as the default name for backwards
# compatibility with controllers that fall back to it).
set(_exo_dynamics_urdf "${CMAKE_CURRENT_BINARY_DIR}/exo_dynamics.urdf")
set(_exo_dynamics_left_urdf "${CMAKE_CURRENT_BINARY_DIR}/exo_dynamics_left.urdf")
set(_exo_dynamics_dual_urdf "${CMAKE_CURRENT_BINARY_DIR}/exo_dynamics_dual.urdf")
add_custom_command(
  OUTPUT ${_exo_build_xacro}
  COMMAND ${CMAKE_COMMAND}
    -DIN=${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.xacro
    -DOUT=${_exo_build_xacro}
    -DURDF_DIR=${CMAKE_CURRENT_SOURCE_DIR}/urdf
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rewrite_xacro_paths.cmake
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_right.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_left.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rewrite_xacro_paths.cmake
  VERBATIM
)
add_custom_command(
  OUTPUT ${_exo_dynamics_urdf}
  COMMAND ${XACRO_EXECUTABLE} ${_exo_build_xacro} hardware:=real arms:=right -o ${_exo_dynamics_urdf}
  DEPENDS
    ${_exo_build_xacro}
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/materials.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_right.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_left.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.gazebo
  VERBATIM
)
add_custom_command(
  OUTPUT ${_exo_dynamics_left_urdf}
  COMMAND ${XACRO_EXECUTABLE} ${_exo_build_xacro} hardware:=real arms:=left -o ${_exo_dynamics_left_urdf}
  DEPENDS
    ${_exo_build_xacro}
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/materials.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_right.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_left.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.gazebo
  VERBATIM
)
add_custom_command(
  OUTPUT ${_exo_dynamics_dual_urdf}
  COMMAND ${XACRO_EXECUTABLE} ${_exo_build_xacro} hardware:=real arms:=dual -o ${_exo_dynamics_dual_urdf}
  DEPENDS
    ${_exo_build_xacro}
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/materials.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_right.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo_arm_left.xacro
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/exo.gazebo
  VERBATIM
)
add_custom_target(generate_exo_dynamics_urdf ALL DEPENDS
  ${_exo_dynamics_urdf} ${_exo_dynamics_left_urdf} ${_exo_dynamics_dual_urdf})

install(
  DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${_exo_dynamics_urdf}
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME exo_dynamics.urdf
)
install(
  FILES ${_exo_dynamics_left_urdf}
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME exo_dynamics_left.urdf
)
install(
  FILES ${_exo_dynamics_dual_urdf}
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME exo_dynamics_dual.urdf
)

ament_package()
