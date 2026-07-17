# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.3)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(osqp-populate NONE)

include(ExternalProject)
ExternalProject_Add(osqp-populate
                     "UPDATE_DISCONNECTED" "False" "GIT_REPOSITORY" "https://github.com/oxfordcontrol/osqp.git" "GIT_TAG" "v0.6.0"
                    SOURCE_DIR          "/media/sf_Master_Thesis_DDLogmans/Software/ROS_environment/Franka_WS/src/franka_ros/franka_example_controllers/lib/osqp-cpp/build/_deps/osqp-src"
                    BINARY_DIR          "/media/sf_Master_Thesis_DDLogmans/Software/ROS_environment/Franka_WS/src/franka_ros/franka_example_controllers/lib/osqp-cpp/build/_deps/osqp-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
