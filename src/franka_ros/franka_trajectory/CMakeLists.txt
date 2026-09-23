cmake_minimum_required(VERSION 3.4)
project(franka_trajectory)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  controller_manager_msgs
  tf2
  trajectory_msgs
  urdf
  dynamic_reconfigure
  franka_msgs
  geometry_msgs
  message_generation
  roscpp
  std_srvs
  std_msgs
)

add_service_files(
  FILES
  SetLinearCommand.srv
  InitializeJointPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# merge Franka + catkin INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager_msgs
    tf2
    trajectory_msgs
    urdf
    dynamic_reconfigure
    franka_msgs
    geometry_msgs
    message_runtime
    roscpp
    std_srvs
    std_msgs
)

set(EXECUTABLES circular chirp square_wave hold tension linear)

foreach(executable ${EXECUTABLES})
  add_executable(${executable}
    src/${executable}.cpp
  )
  add_dependencies(${executable}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(${executable} PRIVATE include)
  target_include_directories(${executable} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${executable} PUBLIC
    ${Franka_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endforeach()
target_sources(linear PRIVATE src/joint_initializer.cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/real_initialization_ros_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/real_initialization.test)
  install(FILES test/real_initialization.test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
  add_executable(initialization_policy_test test/initialization_policy_test.cpp)
  target_include_directories(initialization_policy_test PRIVATE include)
  add_test(NAME initialization_policy_test COMMAND initialization_policy_test)
endif()
message(STATUS "Looking for trajectories")
## Installation
install(TARGETS ${EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
##install(DIRECTORY config
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
##)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_trajectory FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_trajectory
    FILES ${SOURCES}
    DEPENDS ${EXECUTABLES}
  )
endif()
