cmake_minimum_required(VERSION 3.4)
project(franka_gazebo)

execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(${ARCHITECTURE} STREQUAL "armv7l")
  message(WARNING "franka_gazebo was never tested on ARM. Use at own risk!")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load catkin and all dependencies required for this package.
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  roscpp
  std_msgs
  gazebo_ros_control
  kdl_parser
  boost_sml
  control_toolbox
  controller_manager
  controller_interface
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  eigen_conversions
  urdf
  angles
  franka_hw
  franka_msgs 
)

# Use the locally installed libfranka 0.13.3.
find_package(Franka 0.13.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

# Use the currently installed SDFormat 9 package.
find_package(sdformat9 REQUIRED CONFIG)

# gazebo_dev on this machine exports stale SDFormat 9.8 paths even though
# SDFormat 9.10.1 is installed. Remove those stale paths before creating
# targets and link the imported SDFormat target explicitly.
list(REMOVE_ITEM catkin_INCLUDE_DIRS
  "/usr/include/sdformat-9.8"
  "/usr/include/sdformat-9.8/sdf/.."
)

list(REMOVE_ITEM catkin_LIBRARIES
  "/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0"
)

# Keep the workspace and libfranka include directories in the correct order.
# Prefer headers from the selected libfranka installation.
if(NOT Franka_INCLUDE_DIRS)
  message(FATAL_ERROR "Franka_INCLUDE_DIRS is empty")
endif()

list(REMOVE_ITEM catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS})
list(INSERT catkin_INCLUDE_DIRS 0 ${Franka_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_msgs
    std_msgs
    gazebo_ros_control
    kdl_parser
    boost_sml
    controller_manager
    controller_interface
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    eigen_conversions
    urdf
    angles
    franka_hw
    franka_msgs 
  INCLUDE_DIRS
    include
  LIBRARIES
    franka_hw_sim
  DEPENDS
    Franka
    orocos_kdl
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Franka Gazebo hardware simulation library.
add_library(franka_hw_sim
  src/franka_hw_sim.cpp
  src/joint.cpp
  src/model_kdl.cpp
  src/controller_verifier.cpp
)

if(Franka_VERSION GREATER_EQUAL 0.9)
  target_compile_definitions(
    franka_hw_sim
    PUBLIC ENABLE_BASE_ACCELERATION
  )
endif()

target_link_libraries(franka_hw_sim
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  sdformat9::sdformat9
)

target_include_directories(franka_hw_sim
  SYSTEM PUBLIC
    ${Franka_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../franka_example_controllers/include
)

target_include_directories(franka_hw_sim
  PUBLIC
    include
)

add_dependencies(franka_hw_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Gazebo Classic transport -> ROS contact diagnostics (no model modification).
add_executable(contact_diagnostics_bridge src/contact_diagnostics_bridge.cpp)
add_dependencies(contact_diagnostics_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(contact_diagnostics_bridge ${catkin_LIBRARIES} sdformat9::sdformat9)
install(TARGETS contact_diagnostics_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

## Install headers.
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install only the Franka hardware simulation plugin.
install(
  FILES
    franka_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install only the Franka hardware simulation library.
install(
  TARGETS
    franka_hw_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools.
include(
  ${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake
  OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)

if(CLANG_TOOLS)
  file(
    GLOB_RECURSE SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  )

  file(
    GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )

  add_format_target(
    franka_gazebo
    FILES ${SOURCES} ${HEADERS}
  )

  add_tidy_target(
    franka_gazebo
    FILES ${SOURCES}
    DEPENDS franka_hw_sim
  )
endif()