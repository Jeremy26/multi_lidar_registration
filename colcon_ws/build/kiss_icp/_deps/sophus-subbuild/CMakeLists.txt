# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.27.5)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(sophus-populate NONE)



include(ExternalProject)
ExternalProject_Add(sophus-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "TRUE" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "URL" "https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz" "PATCH_COMMAND" "patch" "-p1" "<" "/workspace/colcon_ws/src/kiss-icp/cpp/kiss_icp/3rdparty/sophus/sophus.patch" "UPDATE_DISCONNECTED" "1"
                    SOURCE_DIR          "/workspace/colcon_ws/build/kiss_icp/_deps/sophus-src"
                    BINARY_DIR          "/workspace/colcon_ws/build/kiss_icp/_deps/sophus-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


