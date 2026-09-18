cmake_minimum_required(VERSION 3.4)
project(franka_example_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  franka_msgs
  geometry_msgs
  hardware_interface
  joint_limits_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  urdf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.13.3 REQUIRED)

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(FILES
  JointTorqueComparison.msg
)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
  cfg/dual_arm_compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    franka_example_controllers
    osqp-cpp
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    franka_msgs
    geometry_msgs
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
  DEPENDS
    Franka
)

add_library(franka_example_controllers
  src/cartesian_impedance_example_controller.cpp
  src/cartesian_impedance_cbf_controller.cpp
  src/cartesian_impedance_dead_zone_cbf_controller.cpp
  src/cartesian_impedance_cbf_interaction_controller.cpp
  src/cartesian_impedance_cbf_interaction_power_controller.cpp
  src/cartesian_impedance_dead_zone_cbf_interaction_controller.cpp
  src/cartesian_impedance_directional_kinetic_energy_cbf_controller.cpp
)


add_dependencies(franka_example_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_example_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

# qpsolver

#find_package(osqp REQUIRED)
#target_link_libraries(franka_example_controllers PRIVATE osqp::osqp)

#add_library(osqp-cpp ../../../libs/osqp-cpp/src/osqp++.cc)
#target_link_libraries(osqp-cpp)
#target_include_directories(osqp-cpp PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
#message(STATUS "osqp-cpp: Added osqp-cpp library.")

# sort of almost there attempt:
#add_subdirectory(../../../libs/osqp-cpp ../../../libs/osqp-cpp/build)
#target_link_libraries(franka_example_controllers PUBLIC osqp-cpp)
#target_include_directories(franka_example_controllers PUBLIC "../../../libs/osqp-cpp/build" "../../../libs/osqp-cpp")

#attempt 3: https://stackoverflow.com/questions/26807329/no-rule-to-make-target-error-in-cmake-when-linking-to-shared-library
#add_library(osqp-cpp SHARED IMPORTED REQUIRED)
#set_target_properties(osqp-cpp PROPERTIES IMPORTED_LOCATION ../../../libs/osqp-cpp/build/libosqp-cpp.a)
#target_link_libraries(franka_example_controllers PRIVATE osqp-cpp)

#attempt 4: https://stackoverflow.com/questions/6984479/cmake-how-to-link-a-library-without-automatic-search-function-find-package
#target_link_libraries(franka_example_controllers PRIVATE ../../../libs/osqp-cpp)

set(OSQP-CPP_BUILD_TESTS OFF CACHE BOOL
    "Disable embedded osqp-cpp tests" FORCE)

set(ABSL_BUILD_TESTING OFF CACHE BOOL
    "Disable embedded Abseil tests" FORCE)

add_subdirectory(lib)

target_include_directories(franka_example_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_example_controllers PUBLIC
  include
)

## Installation
install(TARGETS franka_example_controllers osqp-cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_example_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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
  add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_example_controllers
    FILES ${SOURCES}
    DEPENDS franka_example_controllers
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()

catkin_install_python(
  PROGRAMS
    scripts/interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  add_executable(directional_cbf_task_state_test test/directional_cbf_task_state_test.cpp)
  target_include_directories(directional_cbf_task_state_test PRIVATE include)
  add_test(NAME directional_cbf_task_state_test COMMAND directional_cbf_task_state_test)
endif()