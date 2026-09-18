cmake_minimum_required(VERSION 3.4)
project(franka_trajectory)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  franka_msgs
  geometry_msgs
  message_generation
  roscpp
)

add_service_files(
  FILES
  SetLinearCommand.srv
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
    dynamic_reconfigure
    franka_msgs
    geometry_msgs
    message_runtime
    roscpp
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
  target_include_directories(${executable} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${executable} PUBLIC
    ${Franka_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endforeach()
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
