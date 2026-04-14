cmake_minimum_required(VERSION 3.16)
project(exo_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_library(RBDL_LIBRARY NAMES rbdl RBDL DOC "RBDL shared library")
find_path(RBDL_INCLUDE_DIR NAMES rbdl/rbdl.h DOC "RBDL headers")

set(HAVE_RBDL FALSE)
if(RBDL_LIBRARY AND RBDL_INCLUDE_DIR)
  find_package(urdf REQUIRED)
  set(HAVE_RBDL TRUE)
  message(STATUS "exo_utils: RBDL backend enabled (${RBDL_LIBRARY})")
else()
  message(STATUS "exo_utils: RBDL not found; only Pinocchio backend available")
endif()

add_library(exo_utils SHARED
  src/dynamics/dynamics_backend.cpp
  src/kinematics/kinematics_utils.cpp
)

target_include_directories(exo_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(exo_utils PRIVATE EXO_UTILS_HAVE_PINOCCHIO)

if(HAVE_RBDL)
  target_compile_definitions(exo_utils PRIVATE EXO_UTILS_HAVE_RBDL)
  target_include_directories(exo_utils PRIVATE ${RBDL_INCLUDE_DIR})
endif()

target_link_libraries(exo_utils
  PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
)

if(HAVE_RBDL)
  target_link_libraries(exo_utils PRIVATE ${RBDL_LIBRARY} urdf::urdf)
endif()

install(
  TARGETS exo_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(
  EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}Export.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 pinocchio)
if(HAVE_RBDL)
  ament_export_dependencies(urdf)
endif()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/exo_data_logger
  scripts/exo_plot_run
  scripts/exo_dc_motor_id
  scripts/exo_teensy_serial_rtt
  scripts/exo_trajectory_recorder
  scripts/exo_trajectory_player
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
