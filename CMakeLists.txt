# This file is a part of the TChecker project.
#
# See files AUTHORS and LICENSE for copyright details.

cmake_minimum_required(VERSION 2.8.12)

set(LOCAL_CONFIG "${CMAKE_SOURCE_DIR}/localconfig-${CMAKE_BUILD_TYPE}.cmake")
if(EXISTS ${LOCAL_CONFIG})
    cmake_policy(SET CMP0077 NEW)
    message(STATUS "Loading ${LOCAL_CONFIG} file.")
    include(${LOCAL_CONFIG})
else()
    message(STATUS "no ${LOCAL_CONFIG} file.")
endif()

project(tchecker LANGUAGES CXX)

set(TCK_CMAKE_SCRIPT_DIR "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${TCK_CMAKE_SCRIPT_DIR}")

include(CheckCompilerFlags)
include(TcheckerUtils)

# The project version number.
set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   8   CACHE STRING "Project minor version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR)

add_subdirectory(include)
add_subdirectory(src)

enable_testing()
add_subdirectory(test)
